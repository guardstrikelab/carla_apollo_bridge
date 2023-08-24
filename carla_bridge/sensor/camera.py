#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Class to handle Carla camera sensors
"""

import math
import os
from abc import abstractmethod

import carla
import numpy
import transforms3d
import cv2

from carla_bridge.sensor.sensor import Sensor, create_cloud

from modules.common_msgs.sensor_msgs.sensor_image_pb2 import Image, CompressedImage
from modules.common_msgs.sensor_msgs.pointcloud_pb2 import PointXYZIT, PointCloud

from carla_bridge.core.carmera_info import CameraInfo


class Camera(Sensor):

    """
    Sensor implementation details for cameras
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode, is_event_sensor=False):  # pylint: disable=too-many-arguments
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(Camera, self).__init__(uid=uid,
                                     name=name,
                                     parent=parent,
                                     relative_spawn_pose=relative_spawn_pose,
                                     node=node,
                                     carla_actor=carla_actor,
                                     synchronous_mode=synchronous_mode,
                                     is_event_sensor=is_event_sensor)

        if self.__class__.__name__ == "Camera":
            self.node.logwarn("Created Unsupported Camera Actor"
                              "(id={}, type={}, attributes={})".format(self.get_id(),
                                                                       self.carla_actor.type_id,
                                                                       self.carla_actor.attributes))
        else:
            self._build_camera_info()

        self.camera_image_writer = node.new_writer(self.get_topic_prefix() + "/image",
                                                   Image,
                                                   qos_depth=10)
        self.camera_compressed_image_writer = node.new_writer(self.get_topic_prefix() + "/image/compressed",
                                                              CompressedImage,
                                                              qos_depth=10)

    def destroy(self):
        super(Camera, self).destroy()

    def get_topic_prefix(self):
        """
        get the topic name of the current entity.

        :return: the final topic name of this object
        :rtype: string
        """
        return "/apollo/sensor/camera/" + self.name

    def _build_camera_info(self):
        """
        Private function to compute camera info

        camera info doesn't change over time
        """
        camera_info = CameraInfo()
        # store info without header
        camera_info.width = int(self.carla_actor.attributes['image_size_x'])
        camera_info.height = int(self.carla_actor.attributes['image_size_y'])
        camera_info.distortion_model = 'plumb_bob'
        cx = camera_info.width / 2.0
        cy = camera_info.height / 2.0
        fx = camera_info.width / (
            2.0 * math.tan(float(self.carla_actor.attributes['fov']) * math.pi / 360.0))
        fy = fx
        camera_info.K.extend([fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0])
        camera_info.D.extend([0.0, 0.0, 0.0, 0.0, 0.0])
        camera_info.R.extend([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        camera_info.P.extend([fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0])
        self._camera_info = camera_info

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_camera_data):
        """
        Function (override) to transform the received carla camera data
        into a ROS image message
        """
        image_data_array = self.get_image_data_array(carla_camera_data)

        cam_img = Image()
        cam_img.header.CopyFrom(self.parent.get_msg_header())
        cam_img.header.frame_id = self.carla_actor.attributes['role_name']
        cam_img.frame_id = cam_img.header.frame_id
        cam_img.measurement_time = cam_img.header.timestamp_sec
        cam_img.height = image_data_array.shape[0]
        cam_img.width = image_data_array.shape[1]
        #cam_img.encoding = 'BAYER_RGGB8'
        cam_img.encoding = 'rgb8'
        cam_img.data = cv2.imencode('.jpg', image_data_array)[1].tostring()
        # self.camera_image_writer.write(cam_img)

        cam_compressed_img = CompressedImage()
        cam_compressed_img.header.CopyFrom(self.parent.get_msg_header())
        cam_compressed_img.header.frame_id = self.carla_actor.attributes['role_name']
        cam_compressed_img.frame_id = cam_img.header.frame_id
        cam_compressed_img.format = 'jpeg'
        cam_compressed_img.measurement_time = cam_img.header.timestamp_sec
        cam_compressed_img.data = cv2.imencode('.jpg', image_data_array)[1].tostring()
        self.camera_compressed_image_writer.write(cam_compressed_img)

    def get_cyber_transform(self, pose, timestamp):
        """
        Function (override) to modify the tf messages sent by this camera.
        The camera transformation has to be altered to look at the same axis
        as the opencv projection in order to get easy depth cloud for RGBD camera
        :return: the filled tf message
        :rtype: geometry_msgs.msg.TransformStamped
        """
        tf_msg = super(Camera, self).get_cyber_transform(pose, timestamp)
        rotation = tf_msg.transform.rotation

        quat = [rotation.qw, rotation.qx, rotation.qy, rotation.qz]
        quat_swap = transforms3d.quaternions.mat2quat(numpy.matrix(
            [[0, 0, 1],
             [-1, 0, 0],
             [0, -1, 0]]))
        quat = transforms3d.quaternions.qmult(quat, quat_swap)

        tf_msg.transform.rotation.qw = quat[0]
        tf_msg.transform.rotation.qx = quat[1]
        tf_msg.transform.rotation.qy = quat[2]
        tf_msg.transform.rotation.qz = quat[3]

        return tf_msg

    def get_image_data_array(self, carla_camera_data):
        """
        Function to transform the received carla camera data into a ROS image message
        """
        if ((carla_camera_data.height != self._camera_info.height) or
                (carla_camera_data.width != self._camera_info.width)):
            self.node.logerr(
                "Camera{} received image not matching configuration".format(self.get_prefix()))
        image_data_array, encoding = self.get_carla_image_data_array(
            carla_camera_data)

        return image_data_array

    @abstractmethod
    def get_carla_image_data_array(self, carla_camera_data):
        """
        Virtual function to convert the carla camera data to a numpy data array

        :return tuple (numpy data array containing the image information, encoding)
        :rtype tuple(numpy.ndarray, string)
        """
        raise NotImplementedError(
            "This function has to be re-implemented by derived classes")


class RgbCamera(Camera):

    """
    Camera implementation details for rgb camera
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
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
        super(RgbCamera, self).__init__(uid=uid,
                                        name=name,
                                        parent=parent,
                                        relative_spawn_pose=relative_spawn_pose,
                                        node=node,
                                        carla_actor=carla_actor,
                                        synchronous_mode=synchronous_mode)

        self.listen()

    def get_carla_image_data_array(self, carla_image):
        """
        Function (override) to convert the carla image to a numpy data array

        The RGB camera provides a 4-channel int8 color format (bgra).

        :param carla_image: carla image object
        :type carla_image: carla.Image
        :return tuple (numpy data array containing the image information, encoding)
        :rtype tuple(numpy.ndarray, string)
        """

        carla_image_data_array = numpy.ndarray(
            shape=(carla_image.height, carla_image.width, 4),
            dtype=numpy.uint8, buffer=carla_image.raw_data)

        return carla_image_data_array, 'bgra8'


class DepthCamera(Camera):

    """
    Camera implementation details for depth camera
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
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
        super(DepthCamera, self).__init__(uid=uid,
                                          name=name,
                                          parent=parent,
                                          relative_spawn_pose=relative_spawn_pose,
                                          node=node,
                                          carla_actor=carla_actor,
                                          synchronous_mode=synchronous_mode)

        self.listen()

    def get_carla_image_data_array(self, carla_image):
        """
        Function (override) to convert the carla image to a numpy data array

        The depth camera raw image is converted to a linear depth image
        having 1-channel float32.

        :param carla_image: carla image object
        :type carla_image: carla.Image
        :return tuple (numpy data array containing the image information, encoding)
        :rtype tuple(numpy.ndarray, string)
        """

        # color conversion within C++ code is broken, when transforming a
        #  4-channel uint8 color pixel into a 1-channel float32 grayscale pixel
        # therefore, we do it on our own here
        #
        # @todo: After fixing https://github.com/carla-simulator/carla/issues/1041
        # the final code in here should look like:
        #
        # carla_image.convert(carla.ColorConverter.Depth)
        #
        # carla_image_data_array = numpy.ndarray(
        #    shape=(carla_image.height, carla_image.width, 1),
        #    dtype=numpy.float32, buffer=carla_image.raw_data)
        #
        bgra_image = numpy.ndarray(
            shape=(carla_image.height, carla_image.width, 4),
            dtype=numpy.uint8, buffer=carla_image.raw_data)

        # Apply (R + G * 256 + B * 256 * 256) / (256**3 - 1) * 1000
        # according to the documentation:
        # https://carla.readthedocs.io/en/latest/cameras_and_sensors/#camera-depth-map
        scales = numpy.array([65536.0, 256.0, 1.0, 0]) / (256**3 - 1) * 1000
        depth_image = numpy.dot(bgra_image, scales).astype(numpy.float32)

        # actually we want encoding '32FC1'
        # which is automatically selected by cv bridge with passthrough
        return depth_image, 'passthrough'


class SemanticSegmentationCamera(Camera):

    """
    Camera implementation details for segmentation camera
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
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
        super(
            SemanticSegmentationCamera, self).__init__(uid=uid,
                                                       name=name,
                                                       parent=parent,
                                                       relative_spawn_pose=relative_spawn_pose,
                                                       node=node,
                                                       synchronous_mode=synchronous_mode,
                                                       carla_actor=carla_actor)

        # self.listen()

    def get_carla_image_data_array(self, carla_image):
        """
        Function (override) to convert the carla image to a numpy data array

        The segmentation camera raw image is converted to the city scapes palette image
        having 4-channel uint8.

        :param carla_image: carla image object
        :type carla_image: carla.Image
        :return tuple (numpy data array containing the image information, encoding)
        :rtype tuple(numpy.ndarray, string)
        """

        carla_image.convert(carla.ColorConverter.CityScapesPalette)
        carla_image_data_array = numpy.ndarray(
            shape=(carla_image.height, carla_image.width, 4),
            dtype=numpy.uint8, buffer=carla_image.raw_data)
        return carla_image_data_array, 'bgra8'


class DVSCamera(Camera):

    """
    Sensor implementation details for dvs cameras
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):  # pylint: disable=too-many-arguments
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
        super(DVSCamera, self).__init__(uid=uid,
                                        name=name,
                                        parent=parent,
                                        relative_spawn_pose=relative_spawn_pose,
                                        node=node,
                                        carla_actor=carla_actor,
                                        synchronous_mode=synchronous_mode,
                                        is_event_sensor=True)

        self._dvs_events = None
        self.dvs_camera_writer = node.new_writer(self.get_topic_prefix() + '/events',
                                                 PointCloud,
                                                 qos_depth=10)

        # self.listen()

    def destroy(self):
        super(DVSCamera, self).destroy()

    def sensor_data_updated(self, carla_dvs_event_array):
        """
        Function to transform the received DVS event array into a ROS message

        :param carla_dvs_event_array: dvs event array object
        :type carla_image: carla.DVSEventArray
        """
        super(DVSCamera, self).sensor_data_updated(carla_dvs_event_array)

        header = self.get_msg_header(timestamp=carla_dvs_event_array.timestamp)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.UINT16, count=1),
            PointField(name='y', offset=2, datatype=PointField.UINT16, count=1),
            PointField(name='t', offset=4, datatype=PointField.FLOAT64, count=1),
            PointField(name='pol', offset=12, datatype=PointField.INT8, count=1)
        ]

        dvs_events_msg = create_cloud(header, fields, self._dvs_events.tolist())
        # self.dvs_camera_writer.write(dvs_events_msg)

    def get_carla_image_data_array(self, carla_dvs_event_array):
        """
        Function (override) to convert the carla dvs event array to a numpy data array

        The carla.DVSEventArray is converted into a 3-channel int8 color image format (bgr).

        :param carla_dvs_event_array: dvs event array object
        :type carla_dvs_event_array: carla.DVSEventArray
        :return tuple (numpy data array containing the image information, encoding)
        :rtype tuple(numpy.ndarray, string)
        """
        self._dvs_events = numpy.frombuffer(carla_dvs_event_array.raw_data,
                                            dtype=numpy.dtype([
                                                ('x', numpy.uint16),
                                                ('y', numpy.uint16),
                                                ('t', numpy.int64),
                                                ('pol', numpy.bool)
                                            ]))
        carla_image_data_array = numpy.zeros(
            (carla_dvs_event_array.height, carla_dvs_event_array.width, 3),
            dtype=numpy.uint8)
        # Blue is positive, red is negative
        carla_image_data_array[self._dvs_events[:]['y'], self._dvs_events[:]['x'],
                               self._dvs_events[:]['pol'] * 2] = 255

        return carla_image_data_array, 'bgr8'
