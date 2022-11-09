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
import carla_common.transforms as trans

from carla_cyber_bridge.actor import Actor

from cyber.carla_bridge.carla_proto.proto.carla_geometry_pb2 import SolidPrimitive
from cyber.carla_bridge.carla_proto.proto.carla_marker_pb2 import ColorRGBA, Marker, MarkerList
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacle


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
        :type parent: carla_cyber_bridge.Parent
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        """
        self.classification_age = 0
        super(TrafficParticipant, self).__init__(uid=uid,
                                                 name=name,
                                                 parent=parent,
                                                 node=node,
                                                 carla_actor=carla_actor)

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
        super(TrafficParticipant, self).update(frame, timestamp)

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

    def get_marker_color(self):  # pylint: disable=no-self-use
        """
        Function (override) to return the color for marker messages.

        :return: default color used by traffic participants
        :rtpye : std_msgs.msg.ColorRGBA
        """
        color = ColorRGBA()
        color.r = 0.
        color.g = 0.
        color.b = 255.
        return color

    def get_marker_pose(self):
        """
        Function to return the pose for traffic participants.

        :return: the pose of the traffic participant.
        :rtype: geometry_msgs.msg.Pose
        """
        return trans.carla_transform_to_cyber_pose(self.carla_actor.get_transform())

    def get_marker(self, timestamp=None):
        """
        Helper function to create a ROS visualization_msgs.msg.Marker for the actor

        :return:
        visualization_msgs.msg.Marker
        """
        marker = Marker(header=self.get_msg_header(frame_id="map", timestamp=timestamp))
        marker.color.CopyFrom(self.get_marker_color())
        marker.color.a = 0.3
        marker.id = self.get_id()
        marker.type = Marker.CUBE

        marker.pose.CopyFrom(self.get_marker_pose())
        marker.scale.x = self.carla_actor.bounding_box.extent.x * 2.0
        marker.scale.y = self.carla_actor.bounding_box.extent.y * 2.0
        marker.scale.z = self.carla_actor.bounding_box.extent.z * 2.0
        return marker
