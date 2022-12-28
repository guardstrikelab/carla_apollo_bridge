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

from carla_cyber_bridge.traffic_participant import TrafficParticipant

from cyber.carla_bridge.carla_proto.proto.carla_marker_pb2 import ColorRGBA
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacle


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
        :type parent: carla_cyber_bridge.Parent
        :param node: node-handle
        :type node: carla_cyber_bridge.CarlaCyberBridge
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        """
        self.classification = PerceptionObstacle.UNKNOWN_UNMOVABLE
        super(Static, self).__init__(uid=uid,
                                      name=name,
                                      parent=parent,
                                      node=node,
                                      carla_actor=carla_actor)

    def get_marker_color(self):  # pylint: disable=no-self-use
        """
        Function (override) to return the color for marker messages.

        :return: the color used by a static obstacle marker
        :rtpye : std_msgs.msg.ColorRGBA
        """
        color = ColorRGBA()
        color.r = 255.0
        color.g = 255.0
        color.b = 0.0
        return color

    def get_classification(self):
        """
        Function (override) to get classification
        :return:
        """
        return self.classification
