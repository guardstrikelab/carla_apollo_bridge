#!/usr/bin/env python

#
# Copyright (c) 2019-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla Radar
"""

import numpy as np

from carla_cyber_bridge.sensor import Sensor, create_cloud

from modules.drivers.proto.pointcloud_pb2 import PointXYZIT, PointCloud


class Radar(Sensor):

    """
    Actor implementation details of Carla RADAR
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_cyber_bridge.Parent
        :param relative_spawn_pose: the spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(Radar, self).__init__(uid=uid,
                                    name=name,
                                    parent=parent,
                                    relative_spawn_pose=relative_spawn_pose,
                                    node=node,
                                    carla_actor=carla_actor,
                                    synchronous_mode=synchronous_mode)

        self.radar_writer = node.new_writer(self.get_topic_prefix(), PointCloud, qos_depth=10)
        self.listen()

    def destroy(self):
        super(Radar, self).destroy()

    def get_topic_prefix(self):
        """
        get the topic name of the current entity.

        :return: the final topic name of this object
        :rtype: string
        """
        return "/apollo/sensor/" + self.name

    def sensor_data_updated(self, carla_radar_measurement):
        """
        Function to transform the a received Radar measurement into a ROS message
        :param carla_radar_measurement: carla Radar measurement object
        :type carla_radar_measurement: carla.RadarMeasurement
        """
        points = []
        for detection in carla_radar_measurement:
            point = PointXYZIT()
            point.x = detection.depth * np.cos(detection.azimuth) * np.cos(-detection.altitude)
            point.y = detection.depth * np.sin(-detection.azimuth) * np.cos(detection.altitude)
            point.z = detection.depth * np.sin(detection.altitude)
            point.intensity = long(detection.depth)
            points.append(point)

        radar_msg = create_cloud(self.get_msg_header(
            timestamp=carla_radar_measurement.timestamp), points)
        self.radar_writer.write(radar_msg)
