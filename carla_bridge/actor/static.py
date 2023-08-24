#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla static obstacles
"""

from modules.common_msgs.perception_msgs.perception_obstacle_pb2 import (
    PerceptionObstacle,
)

from carla_bridge.actor.traffic_participant import TrafficParticipant


class Static(TrafficParticipant):
    """
    Actor implementation details for static obstacles
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
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        """
        self.classification = PerceptionObstacle.UNKNOWN_UNMOVABLE
        super().__init__(
            uid=uid, name=name, parent=parent, node=node, carla_actor=carla_actor
        )

    def get_classification(self):
        """
        Function (override) to get classification
        :return:
        """
        return self.classification
