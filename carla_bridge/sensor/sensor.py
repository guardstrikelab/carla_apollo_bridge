#!/usr/bin/env python
#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla sensors
"""

from __future__ import print_function

from carla_bridge.actor.actor import Actor
import carla_bridge.utils.transforms as trans

try:
    import queue
except ImportError:
    import Queue as queue
from abc import abstractmethod
from threading import Lock

from cyber.python.cyber_py3 import cyber_time
from modules.common_msgs.sensor_msgs.conti_radar_pb2 import ContiRadar
from modules.common_msgs.sensor_msgs.pointcloud_pb2 import PointCloud
from modules.common_msgs.transform_msgs.transform_pb2 import TransformStamped


class Sensor(Actor):
    """
    Actor implementation details for sensors
    """

    def __init__(
        self,
        uid,
        name,
        parent,
        relative_spawn_pose,
        node,
        carla_actor,
        synchronous_mode,
        is_event_sensor=False,  # only relevant in synchronous_mode:
        # if a sensor only delivers data on special events,
        # do not wait for it. That means you might get data from a
        # sensor, that belongs to a different frame
    ):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: bridge.Parent
        :param relative_spawn_pose: the relative spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: bridge.CarlaCyberBridge
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        :param prefix: the topic prefix to be used for this actor
        :type prefix: string
        """
        super().__init__(
            uid=uid, name=name, parent=parent, node=node, carla_actor=carla_actor
        )

        self.relative_spawn_pose = relative_spawn_pose
        self.synchronous_mode = synchronous_mode
        self.queue = queue.Queue()
        self.next_data_expected_time = None
        self.sensor_tick_time = None
        self.is_event_sensor = is_event_sensor
        self._callback_active = Lock()
        try:
            self.sensor_tick_time = float(carla_actor.attributes["sensor_tick"])
        except (KeyError, ValueError):
            self.sensor_tick_time = None

        # self._tf_broadcaster = tf2_cyber.TransformBroadcaster()

    def get_cyber_transform(self, pose, timestamp):  # pylint: disable=W0613
        if self.synchronous_mode:
            if not self.relative_spawn_pose:
                self.log.info(f"{self.get_prefix()}: No relative spawn pose defined")
                return
            pose = self.relative_spawn_pose
            child_frame_id = self.get_prefix()
            if self.parent is not None:
                frame_id = self.parent.get_prefix()
            else:
                frame_id = "map"

        else:
            child_frame_id = self.get_prefix()
            frame_id = "map"

        transform = TransformStamped()

        transform.header.timestamp_sec = cyber_time.Time.now().to_sec()

        transform.header.frame_id = frame_id
        transform.child_frame_id = child_frame_id

        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z

        transform.transform.rotation.qx = pose.orientation.qx
        transform.transform.rotation.qy = pose.orientation.qy
        transform.transform.rotation.qz = pose.orientation.qz
        transform.transform.rotation.qw = pose.orientation.qw

        return transform

    def write_tf(self, pose, timestamp):
        _ = self.get_cyber_transform(pose, timestamp)
        # self._tf_broadcaster.sendTransform(transform)

    def listen(self):
        self.carla_actor.listen(self._callback_sensor_data)

    def destroy(self):
        """
        Function (override) to destroy this object.

        Stop listening to the carla.Sensor actor.
        Finally forward call to super class.

        :return:
        """
        with self._callback_active:
            if self.carla_actor.is_listening:
                self.carla_actor.stop()
        super().destroy()

    def _callback_sensor_data(self, carla_sensor_data):
        """
        Callback function called whenever new sensor data is received

        :param carla_sensor_data: carla sensor data object
        :type carla_sensor_data: carla.SensorData
        """
        if not self._callback_active.acquire(False):
            # if acquire fails, sensor is currently getting destroyed
            return
        if self.synchronous_mode:
            if self.sensor_tick_time:
                self.next_data_expected_time = carla_sensor_data.timestamp + float(
                    self.sensor_tick_time
                )
            self.queue.put(carla_sensor_data)
        else:
            self.write_tf(
                trans.carla_transform_to_cyber_pose(carla_sensor_data.transform),
                carla_sensor_data.timestamp,
            )

            self.sensor_data_updated(carla_sensor_data)

        self._callback_active.release()

    @abstractmethod
    def sensor_data_updated(self, carla_sensor_data):
        """
        Pure-virtual function to transform the received carla sensor data
        into a corresponding Cyber message

        :param carla_sensor_data: carla sensor data object
        :type carla_sensor_data: carla.SensorData
        """
        raise NotImplementedError(
            "This function has to be implemented by the derived classes"
        )

    def _update_synchronous_event_sensor(
        self, frame, timestamp
    ):  # pylint: disable=W0613
        while True:
            try:
                carla_sensor_data = self.queue.get(block=False)
                if carla_sensor_data.frame != frame:
                    self.log.info(
                        f"{self.__class__.__name__,}({self.get_id()}): \
                            Received event for frame {carla_sensor_data.frame} \
                            (expected {frame}). Process it anyways."
                    )
                self.log.info(
                    f"{self.__class__.__name__}({self.get_id()}): process {frame}"
                )
                self.sensor_data_updated(carla_sensor_data)
            except queue.Empty:
                return

    def _update_synchronous_sensor(self, frame, timestamp):
        while not self.next_data_expected_time or (
            not self.queue.empty()
            or self.next_data_expected_time
            and self.next_data_expected_time < timestamp
        ):
            while True:
                try:
                    carla_sensor_data = self.queue.get(timeout=0.001)
                    if carla_sensor_data.frame == frame:
                        # self.log.info("{}({}): process {}".format(self.__class__.__name__,
                        #                                                self.get_id(), frame))
                        self.write_tf(
                            trans.carla_transform_to_cyber_pose(
                                carla_sensor_data.transform
                            ),
                            timestamp,
                        )
                        self.sensor_data_updated(carla_sensor_data)
                        return
                    elif carla_sensor_data.frame < frame:
                        pass
                        # self.log.info("{}({}): skipping old frame {}, expected {}".format(
                        #     self.__class__.__name__,
                        #     self.get_id(),
                        #     carla_sensor_data.frame,
                        #     frame))
                except queue.Empty:
                    #     self.log.info("{}({}): Expected Frame {} not received".format(
                    #         self.__class__.__name__, self.get_id(), frame))
                    return

    def update(self, frame, timestamp):
        if self.synchronous_mode:
            if self.is_event_sensor:
                self._update_synchronous_event_sensor(frame, timestamp)
            else:
                self._update_synchronous_sensor(frame, timestamp)

        super().update(frame, timestamp)


# http://docs.cyber.org/indigo/api/sensor_msgs/html/point__cloud2_8py_source.html


def create_cloud(header, points):
    """
    Create a L{PointCloud} message.
    @param header: The point cloud header.
    @type  header: L{std_msgs.msg.Header}
    @param fields: The point cloud fields.
    @type  fields: iterable of L{sensor_msgs.msg.PointField}
    @param points: The point cloud points.
    @type  points: list of iterables, i.e. one iterable for each point, with the
                   elements of each iterable being the values of the fields for
                   that point (in the same order as the fields parameter)
    @return: The point cloud.
    @rtype:  L{PointCloud}
    """

    return PointCloud(
        header=header,
        frame_id=header.frame_id,
        is_dense=False,
        point=points,
        width=len(points),
        height=1,
    )


def create_radar(header, contiobs):
    return ContiRadar(header=header, contiobs=contiobs)
