#!/usr/bin/env python

#
# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla vehicles
"""
import math

import numpy as np
import carla
from carla import VehicleControl, Location

from modules.common_msgs.localization_msgs.localization_pb2 import LocalizationEstimate, LocalizationStatus
from modules.common_msgs.chassis_msgs.chassis_pb2 import Chassis
from modules.common_msgs.control_msgs.control_cmd_pb2 import ControlCommand
from modules.common_msgs.transform_msgs.transform_pb2 import TransformStamped, TransformStampeds

from carla_bridge.actor.vehicle import Vehicle
import carla_bridge.utils.transforms as trans


class EgoVehicle(Vehicle):
    """
    Vehicle implementation details for the ego vehicle
    """

    def __init__(self, uid, name, parent, node, carla_actor, world):
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
        """
        super(EgoVehicle, self).__init__(uid=uid,
                                         name=name,
                                         parent=parent,
                                         node=node,
                                         carla_actor=carla_actor)
        self.world = world

        self.vehicle_info_writed = False
        self.vehicle_control_override = False
        self.vehicle_loc_set = False
        
        self.right_turn_ratio = 0.7
        self.left_turn_ratio = 0.85

        self.vehicle_chassis_writer = node.new_writer(
            "/apollo/canbus/chassis",
            Chassis,
            qos_depth=10)
        self.vehicle_pose_writer = node.new_writer(
            "/apollo/localization/pose",
            LocalizationEstimate,
            qos_depth=10)
        self.localization_status_writer = node.new_writer(
            "/apollo/localization/msf_status",
            LocalizationStatus,
            qos_depth=10)
        self.tf_writer = node.new_writer("/tf", TransformStampeds)

        self.control_reader = node.new_reader(
            "/apollo/control",
            ControlCommand,
            lambda data: self.control_command_updated(data, manual_override=False))

    def get_tf_msg(self):
        pose = self.get_current_cyber_pose()

        tf_msg = TransformStamped()
        tf_msg.header.timestamp_sec = self.node.get_time()
        tf_msg.header.frame_id = 'world'
        tf_msg.child_frame_id = 'localization'

        tf_msg.transform.translation.x = pose.position.x
        tf_msg.transform.translation.y = pose.position.y
        tf_msg.transform.translation.z = pose.position.z

        tf_msg.transform.rotation.qx = pose.orientation.qx
        tf_msg.transform.rotation.qy = pose.orientation.qy
        tf_msg.transform.rotation.qz = pose.orientation.qz
        tf_msg.transform.rotation.qw = pose.orientation.qw

        return tf_msg

    def send_vehicle_msgs(self):
        """
        send messages related to vehicle status

        :return:
        """
        vehicle_chassis = Chassis()
        vehicle_chassis.header.timestamp_sec = self.node.get_time()
        vehicle_chassis.header.frame_id = 'ego_vehicle'
        vehicle_chassis.engine_started = True
        vehicle_chassis.speed_mps = self.get_vehicle_speed_abs(self.carla_actor)
        vehicle_chassis.throttle_percentage = self.carla_actor.get_control().throttle * 100.0
        vehicle_chassis.brake_percentage = self.carla_actor.get_control().brake * 100.0
        vehicle_chassis.steering_percentage = -self.carla_actor.get_control().steer * 100.0
        vehicle_chassis.parking_brake = self.carla_actor.get_control().hand_brake
        vehicle_chassis.driving_mode = Chassis.DrivingMode.COMPLETE_AUTO_DRIVE
        self.vehicle_chassis_writer.write(vehicle_chassis)

        transform = self.carla_actor.get_transform()
        spectator = self.world.get_spectator()
        spectator.set_transform(
            carla.Transform(transform.location + carla.Location(x=-10 * math.cos(math.radians(transform.rotation.yaw)),
                                                                y=-10 * math.sin(math.radians(transform.rotation.yaw)),
                                                                z=15),
                            carla.Rotation(pitch=-45, yaw=transform.rotation.yaw)))

        tf_stampeds = TransformStampeds()
        tf_stampeds.transforms.append(self.get_tf_msg())
        self.tf_writer.write(tf_stampeds)
        self.write_localization()
    
    def write_localization(self):
        transform = self.carla_actor.get_transform()
        linear_vel = self.carla_actor.get_velocity()
        angular_vel = self.carla_actor.get_angular_velocity()
        accel = self.carla_actor.get_acceleration()

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

        roll, pitch, yaw = trans.cyber_quaternion_to_cyber_euler(cyber_pose.orientation)

        enu_accel_velocity = trans.n2b(
            pitch,
            roll,
            yaw,
            np.array(
                [
                    cyber_line_accel.linear.x,
                    cyber_line_accel.linear.y,
                    cyber_line_accel.linear.z,
                ]
            ),
        )
        localization_estimate.pose.linear_acceleration_vrf.x = enu_accel_velocity[0, 0]
        localization_estimate.pose.linear_acceleration_vrf.y = enu_accel_velocity[0, 1]
        localization_estimate.pose.linear_acceleration_vrf.z = enu_accel_velocity[0, 2]

        enu_angular_velocity = trans.n2b(
            pitch,
            roll,
            yaw,
            np.array(
                [cyber_twist.angular.x, cyber_twist.angular.y, cyber_twist.angular.z]
            ),
        )
        localization_estimate.pose.angular_velocity_vrf.x = enu_angular_velocity[0, 0]
        localization_estimate.pose.angular_velocity_vrf.y = enu_angular_velocity[0, 1]
        localization_estimate.pose.angular_velocity_vrf.z = enu_angular_velocity[0, 2]

        localization_estimate.pose.euler_angles.x = (
          transform.rotation.roll / 180 * math.pi
        )
        localization_estimate.pose.euler_angles.y = (
          transform.rotation.pitch / 180 * math.pi
        )
        localization_estimate.pose.euler_angles.z = (
          transform.rotation.yaw / 180 * math.pi
        )

        localization_estimate.pose.heading = math.radians(-transform.rotation.yaw)
        self.vehicle_pose_writer.write(localization_estimate)

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.

        On update ego vehicle calculates and sends the new values for VehicleControl()

        :return:
        """
        self.send_vehicle_msgs()
        super(EgoVehicle, self).update(frame, timestamp)

    def destroy(self):
        """
        Function (override) to destroy this object.

        Terminate ROS readers
        Finally forward call to super class.

        :return:
        """
        super(EgoVehicle, self).destroy()

    def control_command_override(self, enable):
        """
        Set the vehicle control mode according to cyber topic
        """
        self.vehicle_control_override = enable.data

    def control_command_updated(self, cyber_vehicle_control, manual_override):
        """
        Receive a ControlCommand msg and send to CARLA

        This function gets called whenever a ControlCommand is received.
        If the mode is valid (either normal or manual), the received ROS message is
        converted into carla.VehicleControl command and sent to CARLA.
        This bridge is not responsible for any restrictions on velocity or steering.
        It's just forwarding the ROS input to CARLA

        :param manual_override: manually override the vehicle control command
        :param cyber_vehicle_control: current vehicle control input received via ROS
        :type cyber_vehicle_control: ControlCommand
        :return:
        """
        if manual_override == self.vehicle_control_override:
            vehicle_control = VehicleControl()
            vehicle_control.throttle = cyber_vehicle_control.throttle / 100.0
            vehicle_control.brake = cyber_vehicle_control.brake / 100.0
            rate = cyber_vehicle_control.steering_rate / 100.0
            if cyber_vehicle_control.steering_target < 0:
                cyber_vehicle_control.steering_target = (
                        cyber_vehicle_control.steering_target * self.right_turn_ratio
                )
            else:
                cyber_vehicle_control.steering_target = (
                        cyber_vehicle_control.steering_target * self.left_turn_ratio
                )
            vehicle_control.steer = -cyber_vehicle_control.steering_target / 100.0
            vehicle_control.hand_brake = cyber_vehicle_control.parking_brake
            vehicle_control.reverse = cyber_vehicle_control.gear_location == Chassis.GearPosition.GEAR_REVERSE
            self.carla_actor.apply_control(vehicle_control)

    def enable_autopilot_updated(self, enable_auto_pilot):
        """
        Enable/disable auto pilot

        :param enable_auto_pilot: should the autopilot be enabled?
        :type enable_auto_pilot: BoolResult
        :return:
        """
        self.carla_actor.set_autopilot(enable_auto_pilot.value)

    @staticmethod
    def get_vector_length_squared(carla_vector):
        """
        Calculate the squared length of a carla_vector
        :param carla_vector: the carla vector
        :type carla_vector: carla.Vector3D
        :return: squared vector length
        :rtype: float64
        """
        return carla_vector.x * carla_vector.x + \
               carla_vector.y * carla_vector.y + \
               carla_vector.z * carla_vector.z

    @staticmethod
    def get_vehicle_speed_squared(carla_vehicle):
        """
        Get the squared speed of a carla vehicle
        :param carla_vehicle: the carla vehicle
        :type carla_vehicle: carla.Vehicle
        :return: squared speed of a carla vehicle [(m/s)^2]
        :rtype: float64
        """
        return EgoVehicle.get_vector_length_squared(carla_vehicle.get_velocity())

    @staticmethod
    def get_vehicle_speed_abs(carla_vehicle):
        """
        Get the absolute speed of a carla vehicle
        :param carla_vehicle: the carla vehicle
        :type carla_vehicle: carla.Vehicle
        :return: speed of a carla vehicle [m/s >= 0]
        :rtype: float64
        """
        speed = math.sqrt(EgoVehicle.get_vehicle_speed_squared(carla_vehicle))
        return speed
