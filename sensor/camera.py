#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

import cv2
import math
import numpy as np

from utils import log

from modules.common_msgs.basic_msgs.header_pb2 import Header
from modules.common_msgs.sensor_msgs.sensor_image_pb2 import CompressedImage


class CameraInfo:
    def __init__(self):
        self.height = 0
        self.width = 0
        self.distortion_model = ''
        self.D = []
        self.K = []
        self.R = []
        self.P = []
        self.binning_x = 0
        self.binning_y = 0


class Camera:
    """
    Sensor implementation details for cameras
    """
    def __init__(self, sensor_obj, camera_actor, node):
        self.node = node
        self.actor = camera_actor
        self.sensor_obj = sensor_obj

        self._build_camera_info()
        self.compressed_6mm_writer = node.create_writer(
            "/apollo/sensor/camera/front_6mm/image/compressed",
            CompressedImage,
            qos_depth=10
        )

    def _build_camera_info(self):
        """
        Private function to compute camera info

        camera info doesn't change over time
        """
        camera_info = CameraInfo()
        camera_info.width = int(self.actor.attributes['image_size_x'])
        camera_info.height = int(self.actor.attributes['image_size_y'])
        log.info("Camera{}: image size is: x {}, y {}".format(self.actor.id, camera_info.width, camera_info.height))

        camera_info.distortion_model = 'plumb_bob'
        cx = camera_info.width / 2.0
        cy = camera_info.height / 2.0
        fx = camera_info.width / (2.0 * math.tan(float(self.actor.attributes['fov']) * math.pi / 360.0))
        fy = fx
        camera_info.K.extend([fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0])
        camera_info.D.extend([0.0, 0.0, 0.0, 0.0, 0.0])
        camera_info.R.extend([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        camera_info.P.extend([fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0])
        self._camera_info = camera_info

    def _get_image_data_array(self, carla_camera_data):
        """
        Function to transform the received carla camera data into a numpy data array

        The RGB camera provides a 4-channel int8 color format (bgra).
        """
        if ((carla_camera_data.height != self._camera_info.height) or
                (carla_camera_data.width != self._camera_info.width)):
            log.error("Camera{} received image not matching configuration")
            log.info(f"Expected: {carla_camera_data.width}x{carla_camera_data.height}")

        carla_image_data_array = np.ndarray(
            shape=(carla_camera_data.height, carla_camera_data.width, 4),
            dtype=np.uint8,
            buffer=carla_camera_data.raw_data
        )

        return carla_image_data_array

    def get_msg_header(self, frame_id=None, timestamp=None):
        header = Header()
        if frame_id:
            header.frame_id = frame_id
        else:
            header.frame_id = "rgb"

        if not timestamp:
            timestamp = self.node.get_time()
        header.timestamp_sec = timestamp
        return header

    def update(self):
        """
        Function (override) to transform the received carla camera data
        into a Cyber image message
        """
        carla_camera_data = self.sensor_obj.get_sensor_data()
        if len(carla_camera_data.raw_data) == 0:
            return

        image_data_array = self._get_image_data_array(carla_camera_data)

        cam_compressed_img = CompressedImage()
        cam_compressed_img.header.CopyFrom(self.get_msg_header())
        cam_compressed_img.format = 'jpeg'
        cam_compressed_img.measurement_time = cam_compressed_img.header.timestamp_sec
        cam_compressed_img.data = cv2.imencode('.jpg', image_data_array)[1].tostring()

        cam_compressed_img.header.frame_id = "front_6mm"
        cam_compressed_img.frame_id = cam_compressed_img.header.frame_id
        self.compressed_6mm_writer.write(cam_compressed_img)
