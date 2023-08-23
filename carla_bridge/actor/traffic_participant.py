#!/usr/bin/env python

#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla traffic participants
"""

import math

from modules.common_msgs.perception_msgs.perception_obstacle_pb2 import (
    PerceptionObstacle,
)

from carla_bridge.actor.actor import Actor
import carla_bridge.utils.transforms as trans


class TrafficParticipant(Actor):
    """
    actor implementation details for traffic participant
    """

    def __init__(self, uid, name, parent, node, carla_actor):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: bridge.Parent
        :param node: node-handle
        :type node: CyberNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        """
        self.classification_age = 0
        super().__init__(
            uid=uid, name=name, parent=parent, node=node, carla_actor=carla_actor
        )

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.

        On update vehicles send:
        - tf global frame
        - object message
        - marker message

        :return:
        """
        self.classification_age += 1
        super().update(frame, timestamp)

    def get_object_info(self):
        """
        Function to send object messages of this traffic participant.

        A derived_object_msgs.msg.Object is prepared to be writed via '/carla/objects'

        :return:
        """
        obj = PerceptionObstacle()
        # ID
        obj.id = self.get_id()
        # Position
        carla_transform = self.carla_actor.get_transform()
        obj.position.x = carla_transform.location.x
        obj.position.y = -carla_transform.location.y
        obj.position.z = carla_transform.location.z
        # Theta
        obj.theta = -math.radians(carla_transform.rotation.yaw)
        # Velocity
        carla_velocity = self.carla_actor.get_velocity()
        obj.velocity.x = carla_velocity.x
        obj.velocity.y = -carla_velocity.y
        obj.velocity.z = carla_velocity.z
        # Acceleration
        carla_accel = self.carla_actor.get_acceleration()
        obj.acceleration.x = carla_accel.x
        obj.acceleration.y = -carla_accel.y
        obj.acceleration.z = carla_accel.z
        # Shape
        obj.length = self.carla_actor.bounding_box.extent.x * 2.0
        obj.width = self.carla_actor.bounding_box.extent.y * 2.0
        obj.height = self.carla_actor.bounding_box.extent.z * 2.0

        # Classification if available in attributes
        if self.get_classification() != PerceptionObstacle.Type.UNKNOWN:
            obj.type = self.get_classification()

        return obj

    def get_classification(self):  # pylint: disable=no-self-use
        """
        Function to get object classification (overridden in subclasses)
        """
        return PerceptionObstacle.UNKNOWN

    def get_marker_pose(self):
        """
        Function to return the pose for traffic participants.

        :return: the pose of the traffic participant.
        :rtype: geometry_msgs.msg.Pose
        """
        return trans.carla_transform_to_cyber_pose(self.carla_actor.get_transform())
