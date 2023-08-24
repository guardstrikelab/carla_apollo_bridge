#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Tool functions to convert transforms from carla to Cyber coordinate system
"""

import math

import carla
import numpy
from modules.common_msgs.basic_msgs.geometry_pb2 import Point3D, PointENU, Quaternion
from modules.common_msgs.localization_msgs.pose_pb2 import Pose
from modules.common_msgs.transform_msgs.transform_pb2 import Transform
from modules.data.proto.frame_pb2 import Vector3
from transforms3d.euler import euler2mat, euler2quat, quat2euler
from transforms3d.quaternions import mat2quat, quat2mat

from carla_bridge.core.geometry import Twist, Accel


def carla_location_to_numpy_vector(carla_location):
    """
    Convert a carla location to a Cyber vector3

    Considers the conversion from left-handed system (unreal) to right-handed
    system (Cyber)

    :param carla_location: the carla location
    :type carla_location: carla.Location
    :return: a numpy.array with 3 elements
    :rtype: numpy.array
    """
    return numpy.array([carla_location.x - carla_location.y, carla_location.z])


def carla_location_to_cyber_vector3(carla_location):
    """
    Convert a carla location to a Cyber vector3

    Considers the conversion from left-handed system (unreal) to right-handed
    system (Cyber)

    :param carla_location: the carla location
    :type carla_location: carla.Location
    :return: a Cyber Vector3
    :rtype: Vector3
    """
    cyber_translation = Vector3()
    cyber_translation.x = carla_location.x
    cyber_translation.y = -carla_location.y
    cyber_translation.z = carla_location.z

    return cyber_translation


def carla_location_to_cyber_point3d(carla_location):
    """
    Convert a carla location to a Cyber vector3

    Considers the conversion from left-handed system (unreal) to right-handed
    system (Cyber)

    :param carla_location: the carla location
    :type carla_location: carla.Location
    :return: a Cyber Point3D
    :rtype: Point3D
    """
    cyber_translation = Point3D()
    cyber_translation.x = carla_location.x
    cyber_translation.y = -carla_location.y
    cyber_translation.z = carla_location.z

    return cyber_translation


def carla_location_to_cyber_point(carla_location):
    """
    Convert a carla location to a Cyber point

    Considers the conversion from left-handed system (unreal) to right-handed
    system (Cyber)

    :param carla_location: the carla location
    :type carla_location: carla.Location
    :return: a Cyber point
    :rtype: geometry_msgs.msg.Point
    """
    cyber_point = PointENU()
    cyber_point.x = carla_location.x
    cyber_point.y = -carla_location.y
    cyber_point.z = carla_location.z

    return cyber_point


def carla_rotation_to_rpy(carla_rotation):
    """
    Convert a carla rotation to a roll, pitch, yaw tuple

    Considers the conversion from left-handed system (unreal) to right-handed
    system (Cyber).
    Considers the conversion from degrees (carla) to radians (Cyber).

    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: a tuple with 3 elements (roll, pitch, yaw)
    :rtype: tuple
    """
    roll = math.radians(carla_rotation.roll)
    pitch = -math.radians(carla_rotation.pitch)
    yaw = -math.radians(carla_rotation.yaw + 90)

    return (roll, pitch, yaw)


def carla_rotation_to_cyber_quaternion(carla_rotation):
    """
    Convert a carla rotation to a Cyber quaternion

    Considers the conversion from left-handed system (unreal) to right-handed
    system (Cyber).
    Considers the conversion from degrees (carla) to radians (Cyber).

    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: a Cyber quaternion
    :rtype: geometry_msgs.msg.Quaternion
    """
    roll, pitch, yaw = carla_rotation_to_rpy(carla_rotation)
    quat = euler2quat(roll, pitch, yaw)
    cyber_quaternion = Quaternion(qx=quat[1], qy=quat[2], qz=quat[3], qw=quat[0])
    return cyber_quaternion


def carla_rotation_to_numpy_rotation_matrix(carla_rotation):
    """
    Convert a carla rotation to a Cyber quaternion

    Considers the conversion from left-handed system (unreal) to right-handed
    system (Cyber).
    Considers the conversion from degrees (carla) to radians (Cyber).

    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: a numpy.array with 3x3 elements
    :rtype: numpy.array
    """
    roll, pitch, yaw = carla_rotation_to_rpy(carla_rotation)
    numpy_array = euler2mat(roll, pitch, yaw)
    rotation_matrix = numpy_array[:3, :3]
    return rotation_matrix


def carla_rotation_to_directional_numpy_vector(carla_rotation):
    """
    Convert a carla rotation (as orientation) into a numpy directional vector

    cyber_quaternion = np_quaternion_to_cyber_quaternion(quat)
    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: a numpy.array with 3 elements as directional vector
        representation of the orientation
    :rtype: numpy.array
    """
    rotation_matrix = carla_rotation_to_numpy_rotation_matrix(carla_rotation)
    directional_vector = numpy.array([1, 0, 0])
    rotated_directional_vector = rotation_matrix.dot(directional_vector)
    return rotated_directional_vector


def carla_vector_to_cyber_vector_rotated(carla_vector, carla_rotation):
    """
    Rotate carla vector, return it as cyber vector

    :param carla_vector: the carla vector
    :type carla_vector: carla.Vector3D
    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: rotated cyber vector
    :rtype: Vector3
    """
    rotation_matrix = carla_rotation_to_numpy_rotation_matrix(carla_rotation)
    tmp_array = rotation_matrix.dot(
        numpy.array([carla_vector.x, carla_vector.y, carla_vector.z])
    )
    cyber_vector = Vector3()
    cyber_vector.x = tmp_array[0]
    cyber_vector.y = -tmp_array[1]
    cyber_vector.z = tmp_array[2]
    return cyber_vector


def carla_velocity_to_cyber_twist(
    carla_linear_velocity, carla_angular_velocity, carla_rotation=None
):
    """
    Convert a carla velocity to a Cyber twist

    Considers the conversion from left-handed system (unreal) to right-handed
    system (Cyber).

    :param carla_velocity: the carla velocity
    :param carla_angular_velocity: the carla angular velocity
    :param carla_rotation: the carla rotation. If None, no rotation is executed
    :return: a Cyber twist (with rotation)
    """
    cyber_twist = Twist()
    if carla_rotation:
        cyber_twist.linear.CopyFrom(
            carla_vector_to_cyber_vector_rotated(carla_linear_velocity, carla_rotation)
        )
    else:
        cyber_twist.linear.CopyFrom(
            carla_location_to_cyber_vector3(carla_linear_velocity)
        )
    cyber_twist.angular.x = math.radians(carla_angular_velocity.x)
    cyber_twist.angular.y = -math.radians(carla_angular_velocity.y)
    cyber_twist.angular.z = -math.radians(carla_angular_velocity.z)
    return cyber_twist


def carla_velocity_to_numpy_vector(carla_velocity):
    """
    Convert a carla velocity to a numpy array

    Considers the conversion from left-handed system (unreal) to right-handed
    system (Cyber)

    :param carla_velocity: the carla velocity
    :type carla_velocity: carla.Vector3D
    :return: a numpy.array with 3 elements
    :rtype: numpy.array
    """
    return numpy.array([carla_velocity.x, -carla_velocity.y, carla_velocity.z])


def carla_acceleration_to_cyber_accel(carla_acceleration):
    """
    Convert a carla acceleration to a Cyber accel

    Considers the conversion from left-handed system (unreal) to right-handed
    system (Cyber)
    The angular accelerations remain zero.

    :param carla_acceleration: the carla acceleration
    :type carla_acceleration: carla.Vector3D
    :return: a Cyber accel
    :rtype: geometry_msgs.msg.Accel
    """
    cyber_accel = Accel()
    cyber_accel.linear.x = carla_acceleration.x
    cyber_accel.linear.y = -carla_acceleration.y
    cyber_accel.linear.z = carla_acceleration.z

    return cyber_accel


def carla_transform_to_cyber_transform(carla_transform):
    """
    Convert a carla transform to a Cyber transform

    See carla_location_to_cyber_point3d() and carla_rotation_to_cyber_quaternion() for details

    :param carla_transform: the carla transform
    :type carla_transform: carla.Transform
    :return: a Cyber transform
    :rtype: geometry_msgs.msg.Transform
    """
    cyber_transform = Transform()

    cyber_transform.translation.CopyFrom(
        carla_location_to_cyber_point3d(carla_transform.location)
    )
    cyber_transform.rotation.CopyFrom(
        carla_rotation_to_cyber_quaternion(carla_transform.rotation)
    )

    return cyber_transform


def carla_transform_to_cyber_pose(carla_transform):
    """
    Convert a carla transform to a Cyber pose

    See carla_location_to_cyber_point() and carla_rotation_to_cyber_quaternion() for details

    :param carla_transform: the carla transform
    :type carla_transform: carla.Transform
    :return: a Cyber pose
    :rtype: geometry_msgs.msg.Pose
    """
    cyber_pose = Pose()

    cyber_pose.position.CopyFrom(
        carla_location_to_cyber_point(carla_transform.location)
    )
    cyber_pose.orientation.CopyFrom(
        carla_rotation_to_cyber_quaternion(carla_transform.rotation)
    )

    return cyber_pose


def carla_location_to_pose(carla_location):
    """
    Convert a carla location to a Cyber pose

    See carla_location_to_cyber_point() for details.
    pose quaternion remains zero.

    :param carla_location: the carla location
    :type carla_location: carla.Location
    :return: a Cyber pose
    :rtype: geometry_msgs.msg.Pose
    """
    cyber_pose = Pose()
    cyber_pose.position = carla_location_to_cyber_point(carla_location)
    cyber_pose.orientation.qw = 1.0
    return cyber_pose


def cyber_point_to_carla_location(cyber_point):
    return carla.Location(
        cyber_point.x, cyber_point.y * -1, cyber_point.z
    )


def rpy_to_carla_rotation(roll, pitch, yaw):
    return carla.Rotation(
        roll=math.degrees(roll), pitch=-math.degrees(pitch), yaw=-math.degrees(yaw)
    )


def cyber_quaternion_to_carla_rotation(cyber_quaternion):
    roll, pitch, yaw = quat2euler(
        [
            cyber_quaternion.qw,
            cyber_quaternion.qx,
            cyber_quaternion.qy,
            cyber_quaternion.qz,
        ]
    )
    return rpy_to_carla_rotation(roll, pitch, yaw)


def cyber_pose_to_carla_transform(cyber_pose):
    """
    Convert a Cyber pose a carla transform.
    """
    return carla.Transform(
        cyber_point_to_carla_location(cyber_pose.position),
        cyber_quaternion_to_carla_rotation(cyber_pose.orientation),
    )


def transform_matrix_to_cyber_pose(mat):
    """
    Convert a transform matrix to a Cyber pose.
    """
    quat = mat2quat(mat[:3, :3])
    msg = Pose()
    msg.position.CopyFrom(PointENU(x=mat[0, 3], y=mat[1, 3], z=mat[2, 3]))
    msg.orientation.CopyFrom(Quaternion(qw=quat[0], qx=quat[1], qy=quat[2], qz=quat[3]))
    return msg


def cyber_pose_to_transform_matrix(msg):
    """
    Convert a Cyber pose to a transform matrix
    """
    mat44 = numpy.eye(4)
    mat44[:3, :3] = quat2mat(
        [msg.orientation.qw, msg.orientation.qx, msg.orientation.qy, msg.orientation.qz]
    )
    mat44[0:3, -1] = [msg.position.x, msg.position.y, msg.position.z]
    return mat44


def cyber_quaternion_to_cyber_euler(cyber_quaternion):
    roll, pitch, yaw = quat2euler(
        [
            cyber_quaternion.qw,
            cyber_quaternion.qx,
            cyber_quaternion.qy,
            cyber_quaternion.qz,
        ]
    )
    return roll, pitch, yaw


def n2b(x_radius, y_radius, z_radius, b):
    x_matrix = numpy.array(
        [
            [1, 0, 0],
            [0, numpy.cos(x_radius), -numpy.sin(x_radius)],
            [0, numpy.sin(x_radius), numpy.cos(x_radius)],
        ]
    )

    y_matrix = numpy.array(
        [
            [numpy.cos(y_radius), 0, numpy.sin(y_radius)],
            [0, 1, 0],
            [-numpy.sin(y_radius), 0, numpy.cos(y_radius)],
        ]
    )

    z_matrix = numpy.array(
        [
            [numpy.cos(z_radius), -numpy.sin(z_radius), 0],
            [numpy.sin(z_radius), numpy.cos(z_radius), 0],
            [0, 0, 1],
        ]
    )

    # conversion_matrix = numpy.matrix(z_matrix) * numpy.matrix(y_matrix) * numpy.matrix(x_matrix)
    # n = numpy.matrix(conversion_matrix.T) * numpy.matrix(numpy.array(b))

    n = (
        numpy.matrix(numpy.array(b))
        * numpy.matrix(x_matrix)
        * numpy.matrix(y_matrix)
        * numpy.matrix(z_matrix)
    )
    return n
