#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla pedestrians
"""

from modules.common_msgs.perception_msgs.perception_obstacle_pb2 import (
    PerceptionObstacle,
)

from carla_bridge.actor.traffic_participant import TrafficParticipant


class Walker(TrafficParticipant):
    """
    Actor implementation details for pedestrians
    """

    def __init__(self, uid, name, parent, node, carla_actor):
        """
        Constructor

        :param uid: unique identifier for this object
        :param name: name identifying this object
        :param parent: the parent of this
        :param node: node-handle
        :param carla_actor: carla walker actor object
        """
        super().__init__(
            uid=uid, name=name, parent=parent, node=node, carla_actor=carla_actor
        )

    # def destroy(self):
    #     """
    #     Function (override) to destroy this object.

    #     Terminate CyberRT readers
    #     Finally forward call to super class.

    #     :return:
    #     """
    #     super(Walker, self).destroy()

    def get_classification(self):
        """
        Function (override) to get classification
        :return:
        """
        return PerceptionObstacle.PEDESTRIAN
