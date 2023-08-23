#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
a sensor that reports the state of all traffic lights
"""

from modules.common_msgs.perception_msgs.traffic_light_detection_pb2 import (
    TrafficLightDetection,
)

from carla_bridge.actor.pseudo_actor import PseudoActor
from carla_bridge.actor.traffic import TrafficLight


class TrafficLightsSensor(PseudoActor):
    """
    a sensor that reports the state of all traffic lights
    """

    def __init__(self, uid, name, parent, node, actor_list, log):
        """
        Constructor
        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying the sensor
        :type name: string
        :param parent: the parent of this
        :type parent: bridge.Parent
        :param node: node-handle
        :type node: CyberNode
        :param actor_list: current list of actors
        :type actor_list: map(carla-actor-id -> python-actor-object)
        """

        super().__init__(uid=uid, name=name, parent=parent, node=node)
        
        self.log = log
        self.actor_list = actor_list
        self.traffic_light_writer = node.new_writer(
            self.get_topic_prefix(), TrafficLightDetection
        )

    def destroy(self):
        """
        Function to destroy this object.
        :return:
        """
        super().destroy()
        self.actor_list = None

    def get_topic_prefix(self):
        """
        get the topic name of the current entity.

        :return: the final topic name of this object
        :rtype: string
        """
        return "/apollo/perception/traffic_light"

    @staticmethod
    def get_blueprint_name():
        """
        Get the blueprint identifier for the pseudo sensor
        :return: name
        """
        return "sensor.pseudo.traffic_lights"

    def update(self, frame, timestamp):  # pylint: disable=W0613
        """
        Get the state of all known traffic lights
        """
        traffic_light_detection = TrafficLightDetection()
        for actor_id in self.actor_list:
            actor = self.actor_list[actor_id]
            if isinstance(actor, TrafficLight):
                traffic_light = traffic_light_detection.traffic_light.add()
                traffic_light.color = actor.get_status()
                # find id of the traffic light in the map
                traffic_light.id = actor.get_id()

        self.traffic_light_writer.write(traffic_light_detection)
