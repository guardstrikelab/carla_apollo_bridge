#!/usr/bin/env python

import math
from utils.transforms import carla_transform_to_cyber_pose

from modules.common_msgs.sensor_msgs.ins_pb2 import InsStat
from modules.common_msgs.localization_msgs.gps_pb2 import Gps
from modules.common_msgs.sensor_msgs.heading_pb2 import Heading
from modules.common_msgs.sensor_msgs.gnss_best_pose_pb2 import GnssBestPose

'''
carla gnss sensor output

Sensor data attribute	Type	        Description
frame	                int	            Frame number when the measurement took place.
timestamp	            double	        Simulation time of the measurement in seconds since the beginning of the episode.
transform	            carla.Transform	Location and rotation in world coordinates of the sensor at the time of the measurement.
latitude	            double	        Latitude of the actor.
longitude	            double	        Longitude of the actor.
altitude	            double	        Altitude of the actor.

'''


class GNSS:
    def __init__(self, actor, ego_vehicle, node):
        self.actor = actor

        self.node = node
        self.ego_vehicle = ego_vehicle

        self.gnss_odometry_writer = self.node.create_writer("/apollo/sensor/gnss/odometry", Gps, qos_depth=10)
        self.gnss_heading_writer = self.node.create_writer("/apollo/sensor/gnss/heading", Heading, qos_depth=10)
        self.gnss_status_writer = self.node.create_writer("/apollo/sensor/gnss/ins_stat", InsStat, qos_depth=10)
        self.gnss_best_pose_writer = self.node.create_writer("/apollo/sensor/gnss/best_pose", GnssBestPose,
                                                             qos_depth=10)

    def update(self):
        carla_gnss_measurement = self.actor.get_sensor_data()

        now_cyber_time = self.node.get_time()
        frame_id = "ego_vehicle/gnss"

        gnss_odometry_msg = Gps()
        gnss_odometry_msg.header.timestamp_sec = now_cyber_time
        gnss_odometry_msg.header.module_name = "gnss"
        gnss_odometry_msg.header.frame_id = frame_id
        cyber_pose = carla_transform_to_cyber_pose(self.ego_vehicle.get_transform())
        gnss_odometry_msg.localization.CopyFrom(cyber_pose)
        vel = self.ego_vehicle.get_velocity()
        gnss_odometry_msg.localization.linear_velocity.x = vel.x
        gnss_odometry_msg.localization.linear_velocity.y = -vel.y
        gnss_odometry_msg.localization.linear_velocity.z = vel.z
        self.gnss_odometry_writer.write(gnss_odometry_msg)

        gnss_heading_msg = Heading()
        gnss_heading_msg.header.timestamp_sec = now_cyber_time
        gnss_heading_msg.header.module_name = "gnss"
        gnss_heading_msg.header.frame_id = frame_id
        gnss_heading_msg.measurement_time = now_cyber_time
        # _, _, yaw = carla_rotation_to_rpy(self.ego_vehicle.get_transform().rotation)

        ego_transform = self.ego_vehicle.get_transform()
        gnss_heading_msg.heading = -math.radians(ego_transform.rotation.yaw)
        self.gnss_heading_writer.write(gnss_heading_msg)

        gnss_best_pose_msg = GnssBestPose()
        gnss_best_pose_msg.header.timestamp_sec = now_cyber_time
        gnss_best_pose_msg.header.module_name = "gnss"
        gnss_best_pose_msg.header.frame_id = frame_id
        gnss_best_pose_msg.latitude = carla_gnss_measurement.latitude
        gnss_best_pose_msg.longitude = carla_gnss_measurement.longitude
        gnss_best_pose_msg.height_msl = carla_gnss_measurement.altitude
        self.gnss_best_pose_writer.write(gnss_best_pose_msg)

        gnss_status_msg = InsStat()
        gnss_status_msg.header.timestamp_sec = now_cyber_time
        gnss_status_msg.header.module_name = "gnss"
        gnss_status_msg.ins_status = 0
        gnss_status_msg.pos_type = 56
        self.gnss_status_writer.write(gnss_status_msg)
