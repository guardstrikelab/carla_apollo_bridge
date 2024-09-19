#!/usr/bin/env python
#
# SYNKROTRON Confidential
# Copyright (C) 2023 SYNKROTRON Inc. All rights reserved.
# The source code for this program is not published
# and protected by copyright controlled
#

import math

import numpy as np
from carla import VehicleControl
from modules.common_msgs.chassis_msgs.chassis_pb2 import Chassis
from modules.common_msgs.control_msgs.control_cmd_pb2 import ControlCommand
from modules.common_msgs.basic_msgs.geometry_pb2 import Point3D
from modules.common_msgs.localization_msgs.localization_pb2 import LocalizationEstimate, LocalizationStatus
from modules.common_msgs.transform_msgs.transform_pb2 import TransformStamped, TransformStampeds

from utils import log
from utils import transforms as trans


class EgoVehicle:
    def __init__(self, ego_vehicle, node):
        self.node = node
        self.ego_vehicle = ego_vehicle

        self.left_turn_ratio = 1  # Left turn amplitude correction
        self.right_turn_ratio = 1  # Right turn amplitude correction
        self.restart_control_flag = False

        self.vehicle_chassis_writer = self.node.create_writer("/apollo/canbus/chassis",
                                                              Chassis,
                                                              qos_depth=10)
        self.vehicle_pose_writer = self.node.create_writer("/apollo/localization/pose",
                                                           LocalizationEstimate,
                                                           qos_depth=10)
        self.localization_status_writer = self.node.create_writer("/apollo/localization/msf_status",
                                                                  LocalizationStatus,
                                                                  qos_depth=10)
        self.tf_writer = self.node.create_writer("/tf", TransformStampeds)

        self.control_reader = self.node.create_reader("/apollo/control",
                                                      ControlCommand,
                                                      self._control_command_updated)

    def _get_tf_msg(self):
        pose = trans.carla_transform_to_cyber_pose(self.ego_vehicle.get_transform())

        tf_msg = TransformStamped()
        tf_msg.header.timestamp_sec = self.node.get_time()
        tf_msg.header.frame_id = "world"
        tf_msg.header.module_name = "oasis_bridge"
        tf_msg.child_frame_id = "localization"

        tf_msg.transform.translation.x = pose.position.x
        tf_msg.transform.translation.y = pose.position.y
        tf_msg.transform.translation.z = pose.position.z

        tf_msg.transform.rotation.qx = pose.orientation.qx
        tf_msg.transform.rotation.qy = pose.orientation.qy
        tf_msg.transform.rotation.qz = pose.orientation.qz
        tf_msg.transform.rotation.qw = pose.orientation.qw

        return tf_msg

    def _send_vehicle_msgs(self):
        """
        send messages related to vehicle status

        :return:
        """
        vehicle_chassis = Chassis()
        vehicle_chassis.header.timestamp_sec = self.node.get_time()
        vehicle_chassis.header.frame_id = "ego_vehicle"
        vehicle_chassis.engine_started = 1
        vehicle_chassis.speed_mps = self._get_vehicle_speed_abs(self.ego_vehicle)
        vehicle_chassis.throttle_percentage = self.ego_vehicle.get_control().throttle * 100.0
        vehicle_chassis.brake_percentage = self.ego_vehicle.get_control().brake * 100.0
        vehicle_chassis.steering_percentage = -self.ego_vehicle.get_control().steer * 100.0
        vehicle_chassis.parking_brake = self.ego_vehicle.get_control().hand_brake
        vehicle_chassis.driving_mode = Chassis.DrivingMode.COMPLETE_AUTO_DRIVE
        self.vehicle_chassis_writer.write(vehicle_chassis)

        localization_status = LocalizationStatus()
        localization_status.header.timestamp_sec = self.node.get_time()
        localization_status.fusion_status = 0  # OK = 0
        localization_status.measurement_time = self.node.get_time()
        localization_status.state_message = ""
        self.localization_status_writer.write(localization_status)

        tf_stampeds = TransformStampeds()
        tf_stampeds.transforms.append(self._get_tf_msg())
        self.tf_writer.write(tf_stampeds)

        self._write_localization()

    def _write_localization(self):
        transform = self.ego_vehicle.get_transform()
        linear_vel = self.ego_vehicle.get_velocity()
        angular_vel = self.ego_vehicle.get_angular_velocity()
        accel = self.ego_vehicle.get_acceleration()

        localization_estimate = LocalizationEstimate()
        localization_estimate.header.timestamp_sec = self.node.get_time()
        localization_estimate.header.frame_id = "novatel"

        cyber_pose = trans.carla_transform_to_cyber_pose(transform)
        localization_estimate.pose.position.x = cyber_pose.position.x
        localization_estimate.pose.position.y = cyber_pose.position.y
        localization_estimate.pose.position.z = cyber_pose.position.z

        localization_estimate.pose.orientation.qx = cyber_pose.orientation.qx
        localization_estimate.pose.orientation.qy = cyber_pose.orientation.qy
        localization_estimate.pose.orientation.qz = cyber_pose.orientation.qz
        localization_estimate.pose.orientation.qw = cyber_pose.orientation.qw

        cyber_twist = trans.carla_velocity_to_cyber_twist(linear_vel, angular_vel)
        localization_estimate.pose.linear_velocity.x = cyber_twist.linear.x
        localization_estimate.pose.linear_velocity.y = cyber_twist.linear.y
        localization_estimate.pose.linear_velocity.z = cyber_twist.linear.z

        localization_estimate.pose.angular_velocity.x = cyber_twist.angular.x
        localization_estimate.pose.angular_velocity.y = cyber_twist.angular.y
        localization_estimate.pose.angular_velocity.z = cyber_twist.angular.z

        cyber_line_accel = trans.carla_acceleration_to_cyber_accel(accel)
        localization_estimate.pose.linear_acceleration.x = cyber_line_accel.linear.x
        localization_estimate.pose.linear_acceleration.y = cyber_line_accel.linear.y
        localization_estimate.pose.linear_acceleration.z = cyber_line_accel.linear.z

        localization_estimate.pose.heading = -math.radians(transform.rotation.yaw)
        # self.log.info(f"heading: {localization_estimate.pose.heading}")
        # roll, pitch, yaw = trans.cyber_quaternion_to_cyber_euler(cyber_pose.orientation)
        #
        # enu_accel_velocity = trans.n2b(pitch, roll, yaw,
        #     np.array(
        #         [
        #             cyber_line_accel.linear.x,
        #             cyber_line_accel.linear.y,
        #             cyber_line_accel.linear.z,
        #         ]
        #     ),
        # )
        # localization_estimate.pose.linear_acceleration_vrf.x = enu_accel_velocity[0, 0]
        # localization_estimate.pose.linear_acceleration_vrf.y = enu_accel_velocity[0, 1]
        # localization_estimate.pose.linear_acceleration_vrf.z = enu_accel_velocity[0, 2]
        #
        # enu_angular_velocity = trans.n2b(pitch, roll, yaw,
        #     np.array(
        #         [cyber_twist.angular.x, cyber_twist.angular.y, cyber_twist.angular.z]
        #     ),
        # )
        # localization_estimate.pose.angular_velocity_vrf.x = enu_angular_velocity[0, 0]
        # localization_estimate.pose.angular_velocity_vrf.y = enu_angular_velocity[0, 1]
        # localization_estimate.pose.angular_velocity_vrf.z = enu_angular_velocity[0, 2]

        local_accel_velocity = trans.transform_world_vector_to_local(transform, accel)
        localization_estimate.pose.linear_acceleration_vrf.x = local_accel_velocity.x
        localization_estimate.pose.linear_acceleration_vrf.y = -local_accel_velocity.y
        localization_estimate.pose.linear_acceleration_vrf.z = local_accel_velocity.z

        local_angular_velocity = trans.transform_world_vector_to_local(transform, angular_vel)
        localization_estimate.pose.angular_velocity_vrf.x = local_angular_velocity.x
        localization_estimate.pose.angular_velocity_vrf.y = -local_angular_velocity.y
        localization_estimate.pose.angular_velocity_vrf.z = local_angular_velocity.z

        localization_estimate.pose.euler_angles.x = math.radians(transform.rotation.roll)
        localization_estimate.pose.euler_angles.y = -math.radians(transform.rotation.pitch)
        localization_estimate.pose.euler_angles.z = -math.radians(transform.rotation.yaw)

        self.vehicle_pose_writer.write(localization_estimate)

    def update(self):
        """
        Function (override) to update this object.

        On update ego vehicle calculates and sends the new values for VehicleControl()

        :return:
        """
        self._send_vehicle_msgs()

    def _control_command_updated(self, cyber_vehicle_control):
        """
        Receive a ControlCommand msg and send to CARLA

        This function gets called whenever a ControlCommand is received.
        If the mode is valid (either normal or manual), the received ROS message is
        converted into carla.VehicleControl command and sent to CARLA.
        This bridge is not responsible for any restrictions on velocity or steering.
        It's just forwarding the ROS input to CARLA

        :param cyber_vehicle_control: current vehicle control input received via ROS
        :type cyber_vehicle_control: ControlCommand
        :return:
        """
        vehicle_control = VehicleControl()
        vehicle_control.throttle = cyber_vehicle_control.throttle / 100.0
        vehicle_control.brake = cyber_vehicle_control.brake / 100.0
        # rate = cyber_vehicle_control.steering_rate / 100.0

        if cyber_vehicle_control.steering_target < 0:
            cyber_vehicle_control.steering_target = cyber_vehicle_control.steering_target * self.right_turn_ratio

        else:
            cyber_vehicle_control.steering_target = cyber_vehicle_control.steering_target * self.left_turn_ratio

        vehicle_control.steer = -cyber_vehicle_control.steering_target / 100.0
        vehicle_control.hand_brake = cyber_vehicle_control.parking_brake
        vehicle_control.reverse = cyber_vehicle_control.gear_location == Chassis.GearPosition.GEAR_REVERSE

        status = cyber_vehicle_control.header.status
        '''
        正常输出
        status:
            error_code: OK

        异常输出情况 1：
        status:
            error_code: OK
            msg: "estop for empty planning trajectory,
                planning headers: timestamp_sec: 1700027960.1997719 module_name: "planning" sequence_num: 0"

        异常输出情况 2：
        status:
            error_code: OK
            msg: msg: "planning has no trajectory point. planning_seq_num:0"
        '''
        # 如果 control 模块输出异常情况 1，则重启 control 且只重启一次
        if "estop" in str(status) and not self.restart_control_flag:
            self.node.restart_apollo_control()
            self.restart_control_flag = True
        self.ego_vehicle.apply_control(vehicle_control)

    @staticmethod
    def _get_vehicle_speed_abs(carla_vehicle):
        """
        Get the absolute speed of a carla vehicle
        :param carla_vehicle: the carla vehicle
        :type carla_vehicle: carla.Vehicle
        :return: speed of a carla vehicle [m/s >= 0]
        :rtype: float64
        """
        velocity_vector = carla_vehicle.get_velocity()

        speed = math.sqrt(velocity_vector.x ** 2 + velocity_vector.y ** 2 + velocity_vector.z ** 2)
        return speed

