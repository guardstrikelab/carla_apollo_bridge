#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla gnsss
"""

from cyber.python.cyber_py3 import cyber_time
from modules.common_msgs.localization_msgs.gps_pb2 import Gps
from modules.common_msgs.sensor_msgs.gnss_best_pose_pb2 import GnssBestPose
from modules.common_msgs.sensor_msgs.heading_pb2 import Heading
from modules.common_msgs.sensor_msgs.ins_pb2 import InsStat

from carla_bridge.sensor.sensor import Sensor
import carla_bridge.utils.transforms as trans


class Gnss(Sensor):

    """
    Actor implementation details for gnss sensor
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
    ):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :param relative_spawn_pose: the relative spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super().__init__(
            uid=uid,
            name=name,
            parent=parent,
            relative_spawn_pose=relative_spawn_pose,
            node=node,
            carla_actor=carla_actor,
            synchronous_mode=synchronous_mode,
        )

        self.gnss_navsatfix_writer = node.new_writer(
            self.get_topic_prefix() + "/best_pose", GnssBestPose, qos_depth=10
        )
        self.gnss_odometry_writer = node.new_writer(
            self.get_topic_prefix() + "/odometry", Gps, qos_depth=10
        )
        self.gnss_heading_writer = node.new_writer(
            self.get_topic_prefix() + "/heading", Heading, qos_depth=10
        )
        self.gnss_status_writer = node.new_writer(
            self.get_topic_prefix() + "/ins_stat", InsStat, qos_depth=10
        )
        self.listen()

    # def destroy(self):
    #     super(Gnss, self).destroy()

    def get_topic_prefix(self):
        """
        get the topic name of the current entity.

        :return: the final topic name of this object
        :rtype: string
        """
        return "/apollo/sensor/" + self.name

    def sensor_data_updated(self, carla_gnss_measurement):  # pylint: disable=W0237
        """
        Function to transform a received gnss event into a ROS NavSatFix message

        :param carla_gnss_measurement: carla gnss measurement object
        :type carla_gnss_measurement: carla.GnssMeasurement
        """
        now_cyber_time = cyber_time.Time.now().to_sec()
        frame_id = "ego_vehicle/gnss"

        gnss_navsatfix_msg = GnssBestPose()
        gnss_navsatfix_msg.header.timestamp_sec = now_cyber_time
        gnss_navsatfix_msg.header.module_name = "gnss"
        gnss_navsatfix_msg.header.frame_id = frame_id
        gnss_navsatfix_msg.latitude = carla_gnss_measurement.latitude
        gnss_navsatfix_msg.longitude = carla_gnss_measurement.longitude
        gnss_navsatfix_msg.height_msl = carla_gnss_measurement.altitude
        self.gnss_navsatfix_writer.write(gnss_navsatfix_msg)

        gnss_odometry_msg = Gps()
        gnss_odometry_msg.header.timestamp_sec = now_cyber_time
        gnss_odometry_msg.header.module_name = "gnss"
        gnss_odometry_msg.header.frame_id = frame_id
        gnss_odometry_msg.localization.CopyFrom(self.parent.get_current_cyber_pose())
        self.gnss_odometry_writer.write(gnss_odometry_msg)

        gnss_heading_msg = Heading()
        gnss_heading_msg.header.timestamp_sec = now_cyber_time
        gnss_heading_msg.header.module_name = "gnss"
        gnss_heading_msg.header.frame_id = frame_id
        gnss_heading_msg.measurement_time = now_cyber_time
        _, _, yaw = trans.carla_rotation_to_rpy(
            self.carla_actor.get_transform().rotation
        )
        gnss_heading_msg.heading = yaw
        self.gnss_heading_writer.write(gnss_heading_msg)

        gnss_status_msg = InsStat()
        gnss_status_msg.header.timestamp_sec = carla_gnss_measurement.timestamp
        gnss_status_msg.header.module_name = "gnss"
        # gnss_status_msg.solution_completed = True
        # gnss_status_msg.solution_status = 0
        # gnss_status_msg.position_type = 56
        # gnss_status_msg.num_sats = 3
        gnss_status_msg.ins_status = 0
        gnss_status_msg.pos_type = 56
        self.gnss_status_writer.write(gnss_status_msg)
