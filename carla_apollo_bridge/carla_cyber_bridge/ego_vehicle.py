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
import os
import time

import numpy
import carla
from carla import VehicleControl

from carla_cyber_bridge.vehicle import Vehicle

from cyber.proto.parameter_pb2 import BoolResult
import carla_common.transforms as trans
from cyber.carla_bridge.carla_proto.proto.carla_ego_vehicle_pb2 import (
    CarlaEgoVehicleInfo,
    CarlaEgoVehicleInfoWheel,
    CarlaEgoVehicleStatus
)
from cyber.carla_bridge.carla_proto.proto.carla_marker_pb2 import ColorRGBA

from modules.localization.proto.localization_pb2 import LocalizationEstimate, LocalizationStatus
from modules.localization.proto.gps_pb2 import Gps
from modules.canbus.proto.chassis_pb2 import Chassis
from modules.control.proto.control_cmd_pb2 import ControlCommand
from modules.planning.proto.planning_pb2 import ADCTrajectory
from modules.common.proto.error_code_pb2 import ErrorCode
from modules.routing.proto.routing_pb2 import RoutingResponse
from modules.map.relative_map.proto.relative_map_config_pb2 import NavigationLaneConfig
from modules.perception.proto.traffic_light_detection_pb2 import TrafficLightDetection
from modules.transform.proto.transform_pb2 import TransformStamped, TransformStampeds
from modules.common.proto.geometry_pb2 import Point3D
from cyber.carla_bridge.carla_proto.proto.carla_geometry_pb2 import Twist, Accel
import os, sys
import json

import ctypes
#from ctypes import *

# import redis
# r = redis.Redis(host="172.16.19.58", port=6379, db=1, password="123456", decode_responses=True)

class EgoVehicle(Vehicle):

    """
    Vehicle implementation details for the ego vehicle
    """

    def __init__(self, uid, name, parent, node, carla_actor, world, vehicle_control_applied_callback):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_cyber_bridge.Parent
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
        self.vehicle_begin_run = False
        self._vehicle_control_applied_callback = vehicle_control_applied_callback

        self.vehicle_chassis_writer = node.new_writer(
            "/apollo/canbus/chassis",
            Chassis,
            qos_depth=10)
        self.vehicle_info_writer = node.new_writer(
            "/apollo/vehicle_info",
            CarlaEgoVehicleInfo,
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

        self.enable_autopilot_reader = node.new_reader(
            self.get_topic_prefix() + "/enable_autopilot",
            BoolResult,
            self.enable_autopilot_updated)



    def get_marker_color(self):
        """
        Function (override) to return the color for marker messages.

        The ego vehicle uses a different marker color than other vehicles.

        :return: the color used by a ego vehicle marker
        :rtpye : std_msgs.msg.ColorRGBA
        """
        color = ColorRGBA()
        color.r = 0.0
        color.g = 255.0
        color.b = 0.0
        return color

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

    def send_vehicle_msgs(self, frame, timestamp):
        """
        send messages related to vehicle status

        :return:
        """
        vehicle_chassis = Chassis()
        vehicle_chassis.header.timestamp_sec = self.node.get_time()
        vehicle_chassis.header.frame_id = 'ego_vehicle'
        vehicle_chassis.engine_started = True
        vehicle_chassis.speed_mps = self.get_vehicle_speed_abs(self.carla_actor)
        # vehicle_chassis.speed_mps = self.get_vehicle_speed_abs(self.carla_actor) / 3.6

        vehicle_chassis.throttle_percentage = self.carla_actor.get_control().throttle * 100.0
        vehicle_chassis.brake_percentage = self.carla_actor.get_control().brake * 100.0
        vehicle_chassis.steering_percentage = -self.carla_actor.get_control().steer * 100.0
        # if vehicle_chassis.steering_percentage < 1e-4:
        #     vehicle_chassis.steering_percentage = 0


        # vehicle_chassis.brake_percentage = 0
        # vehicle_chassis.throttle_percentage = 0

        vehicle_chassis.parking_brake = self.carla_actor.get_control().hand_brake
        vehicle_chassis.driving_mode = Chassis.DrivingMode.COMPLETE_AUTO_DRIVE
        self.vehicle_chassis_writer.write(vehicle_chassis)
        self.node.loginfo("write chassis module ===throttle is {}, brake is {},  steer is {}".format(
            vehicle_chassis.throttle_percentage, vehicle_chassis.brake_percentage, vehicle_chassis.steering_percentage))

        # only send vehicle once (in latched-mode)
        if not self.vehicle_info_writed:
	        self.vehicle_info_writed = True
	        vehicle_info = CarlaEgoVehicleInfo()
	        vehicle_info.id = self.carla_actor.id
	        vehicle_info.type = self.carla_actor.type_id
	        vehicle_info.rolename = self.carla_actor.attributes.get('role_name')
	        vehicle_physics = self.carla_actor.get_physics_control()

	        for wheel in vehicle_physics.wheels:
	            wheel_info = CarlaEgoVehicleInfoWheel()
	            wheel_info.tire_friction = wheel.tire_friction
	            wheel_info.damping_rate = wheel.damping_rate
	            wheel_info.max_steer_angle = math.radians(wheel.max_steer_angle)
	            wheel_info.radius = wheel.radius
	            wheel_info.max_brake_torque = wheel.max_brake_torque
	            wheel_info.max_handbrake_torque = wheel.max_handbrake_torque

	            inv_T = numpy.array(self.carla_actor.get_transform().get_inverse_matrix(), dtype=float)
	            wheel_pos_in_map = numpy.array([wheel.position.x/100.0,
	                                    wheel.position.y/100.0,
	                                    wheel.position.z/100.0,
	                                    1.0])
	            wheel_pos_in_ego_vehicle = numpy.matmul(inv_T, wheel_pos_in_map)
	            wheel_info.position.x = wheel_pos_in_ego_vehicle[0]
	            wheel_info.position.y = -wheel_pos_in_ego_vehicle[1]
	            wheel_info.position.z = wheel_pos_in_ego_vehicle[2]
	            vehicle_info.wheels.append(wheel_info)

	        vehicle_info.max_rpm = vehicle_physics.max_rpm
	        vehicle_info.max_rpm = vehicle_physics.max_rpm
	        vehicle_info.moi = vehicle_physics.moi
	        vehicle_info.damping_rate_full_throttle = vehicle_physics.damping_rate_full_throttle
	        vehicle_info.damping_rate_zero_throttle_clutch_engaged = \
	            vehicle_physics.damping_rate_zero_throttle_clutch_engaged
	        vehicle_info.damping_rate_zero_throttle_clutch_disengaged = \
	            vehicle_physics.damping_rate_zero_throttle_clutch_disengaged
	        vehicle_info.use_gear_autobox = vehicle_physics.use_gear_autobox
	        vehicle_info.gear_switch_time = vehicle_physics.gear_switch_time
	        vehicle_info.clutch_strength = vehicle_physics.clutch_strength
	        vehicle_info.mass = vehicle_physics.mass
	        vehicle_info.drag_coefficient = vehicle_physics.drag_coefficient
	        vehicle_info.center_of_mass.x = vehicle_physics.center_of_mass.x
	        vehicle_info.center_of_mass.y = vehicle_physics.center_of_mass.y
	        vehicle_info.center_of_mass.z = vehicle_physics.center_of_mass.z

	        self.vehicle_info_writer.write(vehicle_info)

        # # '''
        # # Mock locaization estimate.
        # # '''
        # id = self.carla_actor.id
        # print "####carla_actor Id:%s" % id
        # transform = self.carla_actor.get_transform()
        # carla_linear_velocity = self.carla_actor.get_velocity()
        # carla_angular_velocity = self.carla_actor.get_angular_velocity()
        # carla_line_accel = self.carla_actor.get_acceleration()
        #
        # # spectator = self.world.get_spectator()
        # # spectator.set_transform(carla.Transform(transform.location + carla.Location(z=15), carla.Rotation(pitch=-90)))
        #
        # my_maps = self.world.get_map()
        # waypoint = my_maps.get_waypoint(transform.location, project_to_road=True, lane_type=(carla.LaneType.Driving))
        # print "waypoint"
        # print waypoint
        #
        # localization_estimate = LocalizationEstimate()
        # localization_estimate.header.timestamp_sec = self.node.get_time()
        # localization_estimate.header.frame_id = 'novatel'
        #
        # cyber_pose = trans.carla_transform_to_cyber_pose(transform)
        # localization_estimate.pose.position.x = cyber_pose.position.x
        # localization_estimate.pose.position.y = cyber_pose.position.y
        # localization_estimate.pose.position.z = cyber_pose.position.z
        # self.node.loginfo("position.x is {}, position.y is {}, position.z is {}".format(cyber_pose.position.x,
        #                                                                                 cyber_pose.position.y,
        #                                                                                 cyber_pose.position.z))
        # # add orientation
        # localization_estimate.pose.orientation.qx = cyber_pose.orientation.qx
        # localization_estimate.pose.orientation.qy = cyber_pose.orientation.qy
        # localization_estimate.pose.orientation.qz = cyber_pose.orientation.qz
        # localization_estimate.pose.orientation.qw = cyber_pose.orientation.qw
        #
        # # localization_estimate.pose.linear_velocity.x = linear_vel.x
        # # localization_estimate.pose.linear_velocity.y = linear_vel.y
        # # localization_estimate.pose.linear_velocity.z = linear_vel.z
        # roll, pitch, yaw = trans.cyber_quaternion_to_cyber_euler(cyber_pose.orientation)
        # self.node.loginfo("roll is {}, pitch is {}, yaw is {}".format(roll, pitch, yaw))
        #
        # cyber_twist = Twist()
        # cyber_twist = trans.carla_velocity_to_cyber_twist(carla_linear_velocity, carla_angular_velocity)
        # localization_estimate.pose.linear_velocity.x = cyber_twist.linear.x
        # localization_estimate.pose.linear_velocity.y = cyber_twist.linear.y
        # localization_estimate.pose.linear_velocity.z = cyber_twist.linear.z
        #
        # self.node.loginfo("1 linear_velocity.x is {}, linear_velocity.y is {}, linear_velocity.z is {}".format(
        #     localization_estimate.pose.linear_velocity.x, localization_estimate.pose.linear_velocity.y,
        #     localization_estimate.pose.linear_velocity.z))
        #
        #
        # # enu_linear_velocity = trans.b2n(pitch, roll, yaw, [carla_linear_velocity.x, carla_linear_velocity.y, carla_linear_velocity.z])
        # # localization_estimate.pose.linear_velocity.x = enu_linear_velocity[0, 0]
        # # localization_estimate.pose.linear_velocity.y = enu_linear_velocity[0, 1]
        # # localization_estimate.pose.linear_velocity.z = enu_linear_velocity[0, 2]
        #
        # self.node.loginfo("2 linear_velocity.x is {}, linear_velocity.y is {}, linear_velocity.z is {}".format(
        #     localization_estimate.pose.linear_velocity.x, localization_estimate.pose.linear_velocity.y,
        #     localization_estimate.pose.linear_velocity.z))
        #
        #
        # cyber_line_accel = Accel()
        # cyber_line_accel = trans.carla_acceleration_to_cyber_accel(carla_line_accel)
        # localization_estimate.pose.linear_acceleration.x = cyber_line_accel.linear.x
        # localization_estimate.pose.linear_acceleration.y = cyber_line_accel.linear.y
        # localization_estimate.pose.linear_acceleration.z = cyber_line_accel.linear.z
        #
        # # self.node.loginfo("1 linear_acceleration.x is {}, linear_acceleration.y is {}, linear_acceleration.z is {}".format(
        # #     localization_estimate.pose.linear_acceleration.x, localization_estimate.pose.linear_acceleration.y,
        # #     localization_estimate.pose.linear_acceleration.z))
        #
        # # enu_accel_velocity = trans.b2n(pitch, roll, yaw, [carla_line_accel.x, carla_line_accel.y, carla_line_accel.z])
        # # localization_estimate.pose.linear_acceleration.x = enu_accel_velocity[0, 0]
        # # localization_estimate.pose.linear_acceleration.y = enu_accel_velocity[0, 1]
        # # localization_estimate.pose.linear_acceleration.z = enu_accel_velocity[0, 2]
        #
        #
        # self.node.loginfo("2 linear_acceleration.x is {}, linear_acceleration.y is {}, linear_acceleration.z is {}".format(
        #     localization_estimate.pose.linear_acceleration.x, localization_estimate.pose.linear_acceleration.y,
        #     localization_estimate.pose.linear_acceleration.z))
        #
        #
        # localization_estimate.pose.angular_velocity.x = cyber_twist.angular.x
        # localization_estimate.pose.angular_velocity.y = cyber_twist.angular.y
        # localization_estimate.pose.angular_velocity.z = cyber_twist.angular.z
        #
        # # mrf_angular_velocity = Point3D()
        # # mrf_angular_velocity.x = cyber_twist.angular.x
        # # mrf_angular_velocity.y = cyber_twist.angular.y
        # # mrf_angular_velocity.z = cyber_twist.angular.z
        # #
        # # localization_estimate.pose.angular_velocity.x = mrf_angular_velocity.x
        # # localization_estimate.pose.angular_velocity.y = mrf_angular_velocity.y
        # # localization_estimate.pose.angular_velocity.z = mrf_angular_velocity.z
        # #
        # self.node.loginfo("1 angular_velocity.x is {}, angular_velocity.y is {}, angular_velocity.z is {}".format(
        #     localization_estimate.pose.angular_velocity.x, localization_estimate.pose.angular_velocity.y,
        #     localization_estimate.pose.angular_velocity.z))
        #
        # # enu_angular_velocity = trans.b2n(pitch, roll, yaw, [carla_angular_velocity.x, carla_angular_velocity.y, carla_angular_velocity.z])
        # # localization_estimate.pose.angular_velocity.x = enu_angular_velocity[0, 0]
        # # localization_estimate.pose.angular_velocity.y = enu_angular_velocity[0, 1]
        # # localization_estimate.pose.angular_velocity.z = enu_angular_velocity[0, 2]
        #
        # self.node.loginfo("2 angular_velocity.x is {}, angular_velocity.y is {}, angular_velocity.z is {}".format(
        #     localization_estimate.pose.angular_velocity.x, localization_estimate.pose.angular_velocity.y,
        #     localization_estimate.pose.angular_velocity.z))
        #
        #
        # vrf_angular_velocity = Point3D()
        # # / apollo / cyber / carla_bridge / carla_cyber_bridge
        # command = "/apollo/cyber/carla_bridge/carla_cyber_bridge/sim_control_testmy %s %s %s %s %s %s %s" % (localization_estimate.pose.angular_velocity.x,\
        #                                                              localization_estimate.pose.angular_velocity.y,\
        #                                                              localization_estimate.pose.angular_velocity.z,\
        #                                                              cyber_pose.orientation.qx,\
        #                                                              cyber_pose.orientation.qy,\
        #                                                              cyber_pose.orientation.qz,\
        #                                                              cyber_pose.orientation.qw)
        # print "cmd:%s" % command
        # tmp = os.popen(command)
        # tmpres = tmp.readlines()
        # print "tmpres %s" %  tmpres
        # print "tmpres7 %s" % tmpres[7]
        # vj = json.loads(tmpres[7])
        # print "vj:%s" % vj
        # print "vj0:%s" % vj[0]
        # print "vj0:%s" % vj[1]
        # print "vj0:%s" % vj[2]
        #
        # vrf_angular_velocity.x = vj[0]
        # vrf_angular_velocity.y = vj[1]
        # vrf_angular_velocity.z = vj[2]
        #
        # localization_estimate.pose.angular_velocity_vrf.x = vrf_angular_velocity.x
        # localization_estimate.pose.angular_velocity_vrf.y = vrf_angular_velocity.y
        # localization_estimate.pose.angular_velocity_vrf.z = vrf_angular_velocity.z
        #
        # # localization_estimate.pose.linear_acceleration_vrf.x = accel.x
        # # localization_estimate.pose.linear_acceleration_vrf.y = accel.y
        # # localization_estimate.pose.linear_acceleration_vrf.z = accel.z
        #
        # mrf_linear_acceleration = Point3D()
        # # mrf_linear_acceleration.x = localization_estimate.pose.linear_acceleration.x
        # # mrf_linear_acceleration.y = localization_estimate.pose.linear_acceleration.y
        # # mrf_linear_acceleration.z = localization_estimate.pose.linear_acceleration.z
        #
        # vrf_linear_acceleration = Point3D()
        #
        # command2 = "/apollo/cyber/carla_bridge/carla_cyber_bridge/sim_control_testmy %s %s %s %s %s %s %s" % (localization_estimate.pose.linear_acceleration.x,\
        #                                                              localization_estimate.pose.linear_acceleration.y,\
        #                                                              localization_estimate.pose.linear_acceleration.z,\
        #                                                              cyber_pose.orientation.qx,\
        #                                                              cyber_pose.orientation.qy,\
        #                                                              cyber_pose.orientation.qz,\
        #                                                              cyber_pose.orientation.qw)
        # print "vrf_linear_acceleration cmd:%s" % command2
        # tmp2 = os.popen(command2)
        # tmpres = tmp2.readlines()
        # print "tmpres %s" %  tmpres
        # print "tmpres7 %s" % tmpres[7]
        # vj = json.loads(tmpres[7])
        # print "vj:%s" % vj
        # print "vj0:%s" % vj[0]
        # print "vj0:%s" % vj[1]
        # print "vj0:%s" % vj[2]
        #
        # vrf_linear_acceleration.x = vj[0]
        # vrf_linear_acceleration.y = vj[1]
        # vrf_linear_acceleration.z = vj[2]
        #
        # localization_estimate.pose.linear_acceleration_vrf.x = vrf_linear_acceleration.x
        # localization_estimate.pose.linear_acceleration_vrf.y = vrf_linear_acceleration.y
        # localization_estimate.pose.linear_acceleration_vrf.z = vrf_linear_acceleration.z
        #
        # # luhongliang
        # # localization_estimate.pose.heading = math.radians(-transform.rotation.yaw)
        # localization_estimate.pose.heading = math.radians(yaw)
        #
        #
        # self.vehicle_pose_writer.write(localization_estimate)
        #
        # localization_status = LocalizationStatus()
        # localization_status.header.timestamp_sec = self.node.get_time()
        # localization_status.fusion_status = 0  # OK = 0
        # localization_status.measurement_time = self.node.get_time()
        # localization_status.state_message = ""
        # self.localization_status_writer.write(localization_status)

        tf_stampeds = TransformStampeds()
        tf_stampeds.transforms.append(self.get_tf_msg())
        self.tf_writer.write(tf_stampeds)




    def update(self, frame, timestamp):
        """
        Function (override) to update this object.

        On update ego vehicle calculates and sends the new values for VehicleControl()

        :return:
        """
        self.send_vehicle_msgs(frame, timestamp)
        super(EgoVehicle, self).update(frame, timestamp)

    def destroy(self):
        """
        Function (override) to destroy this object.

        Terminate ROS readers
        Finally forward call to super class.

        :return:
        """
        self.node.logdebug("Destroy Vehicle(id={})".format(self.get_id()))
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
            vehicle_control.steer = -cyber_vehicle_control.steering_target / 100.0
            vehicle_control.hand_brake = cyber_vehicle_control.parking_brake
            vehicle_control.reverse = cyber_vehicle_control.gear_location == Chassis.GearPosition.GEAR_REVERSE
            self.carla_actor.apply_control(vehicle_control)
            self._vehicle_control_applied_callback(self.get_id())

    def enable_autopilot_updated(self, enable_auto_pilot):
        """
        Enable/disable auto pilot

        :param enable_auto_pilot: should the autopilot be enabled?
        :type enable_auto_pilot: BoolResult
        :return:
        """
        self.node.logdebug("Ego vehicle: Set autopilot to {}".format(enable_auto_pilot.value))
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
