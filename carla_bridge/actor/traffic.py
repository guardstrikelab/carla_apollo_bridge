#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla traffic objects
"""

from carla import TrafficLightState
from modules.common_msgs.perception_msgs.traffic_light_detection_pb2 import (
    TrafficLight as ApolloTrafficLight,
)

from actor.actor import Actor


class Traffic(Actor):
    """
    Actor implementation details for traffic objects
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
        super().__init__(
            uid=uid, name=name, parent=parent, node=node, carla_actor=carla_actor
        )


class TrafficLight(Actor):
    """
    Traffic implementation details for traffic lights
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
        :type carla_actor: carla.TrafficLight
        """
        super().__init__(
            uid=uid, name=name, parent=parent, node=node, carla_actor=carla_actor
        )

    def get_status(self):
        """
        Get the current state of the traffic light
        """
        carla_state = self.carla_actor.get_state()
        if carla_state == TrafficLightState.Red:
            return ApolloTrafficLight.RED
        elif carla_state == TrafficLightState.Yellow:
            return ApolloTrafficLight.YELLOW
        elif carla_state == TrafficLightState.Green:
            return ApolloTrafficLight.GREEN
        elif carla_state == TrafficLightState.Off:
            return ApolloTrafficLight.BLACK
        else:
            return ApolloTrafficLight.UNKNOWN

    def get_id(self):
        opendrive_id = self.carla_actor.get_opendrive_id()
        carla_map = self.carla_actor.get_world().get_map()
        light_waypoint = carla_map.get_waypoint(self.carla_actor.get_location())
        return f"signal_{light_waypoint.road_id}_{opendrive_id}"
