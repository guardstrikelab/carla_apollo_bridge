#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla vehicles
"""

from modules.common_msgs.perception_msgs.perception_obstacle_pb2 import (
    PerceptionObstacle,
)

from carla_bridge.actor.traffic_participant import TrafficParticipant


class Vehicle(TrafficParticipant):
    """
    Actor implementation details for vehicles
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
        :type node: bridge.CarlaCyberBridge
        :param carla_actor: carla vehicle actor object
        :type carla_actor: carla.Vehicle
        """
        self.classification = PerceptionObstacle.VEHICLE
        if "object_type" in carla_actor.attributes:
            if carla_actor.attributes["object_type"] == "car":
                self.classification = PerceptionObstacle.VEHICLE
            elif carla_actor.attributes["object_type"] == "bike":
                self.classification = PerceptionObstacle.BICYCLE
            elif carla_actor.attributes["object_type"] == "motorcycle":
                self.classification = PerceptionObstacle.BICYCLE
            elif carla_actor.attributes["object_type"] == "truck":
                self.classification = PerceptionObstacle.VEHICLE
            elif carla_actor.attributes["object_type"] == "other":
                self.classification = PerceptionObstacle.UNKNOWN_MOVABLE

        super().__init__(
            uid=uid, name=name, parent=parent, node=node, carla_actor=carla_actor
        )

    def get_classification(self):
        """
        Function (override) to get classification
        :return:
        """
        return self.classification
