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
import math

from carla_bridge.sensor.sensor import Sensor, create_radar

from modules.common_msgs.sensor_msgs.conti_radar_pb2 import ContiRadar, ContiRadarObs


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

        self.radar_writer = node.new_writer(self.get_topic_prefix(), ContiRadar, qos_depth=10)
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
        # points = []
        # for detection in carla_radar_measurement:
        #     point = PointXYZIT()
        #     point.x = detection.depth * np.cos(detection.azimuth) * np.cos(-detection.altitude)
        #     point.y = detection.depth * np.sin(-detection.azimuth) * np.cos(detection.altitude)
        #     point.z = detection.depth * np.sin(detection.altitude)
        #     point.intensity = long(detection.depth)
        #     points.append(point)
        #
        # radar_msg = create_cloud(self.get_msg_header(
        #     timestamp=carla_radar_measurement.timestamp), points)
        # self.radar_writer.write(radar_msg)

        radars = []

        for detection in carla_radar_measurement:
            radarObs = ContiRadarObs()
            azi_rad = math.radians(detection.azimuth)
            x = detection.depth * math.cos(azi_rad)
            y = detection.depth * math.sin(azi_rad)
            # Shift the x and y coordinates so that they are centered around the radar position
            x -= carla_radar_measurement.transform.location.x
            y -= carla_radar_measurement.transform.location.y

            velocity = detection.velocity
            # Use the x-axis of the radar as the positive direction
            vel_x = velocity * math.sin(-azi_rad)
            vel_y = velocity * math.cos(-azi_rad)

            radarObs.longitude_dist = x
            radarObs.lateral_dist = y
            radarObs.longitude_vel = vel_x
            radarObs.lateral_vel = vel_y
            radars.append(radarObs)

        radar_msg = create_radar(self.get_msg_header(
            timestamp=carla_radar_measurement.timestamp), radars)
        self.radar_writer.write(radar_msg)
