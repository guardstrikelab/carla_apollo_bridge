#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
handle a object sensor
"""

from carla_cyber_bridge.pseudo_actor import PseudoActor
from carla_cyber_bridge.vehicle import Vehicle
from carla_cyber_bridge.walker import Walker

from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacle, PerceptionObstacles


class ObjectSensor(PseudoActor):

    """
    Pseudo object sensor
    """

    def __init__(self, uid, name, parent, node, actor_list):
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
        :param actor_list: current list of actors
        :type actor_list: map(carla-actor-id -> python-actor-object)
        """

        super(ObjectSensor, self).__init__(uid=uid,
                                           name=name,
                                           parent=parent,
                                           node=node)
        self.actor_list = actor_list
        self.object_writer = node.new_writer(self.get_topic_prefix(),
                                             PerceptionObstacles,
                                             qos_depth=10)

    def destroy(self):
        """
        Function to destroy this object.
        :return:
        """
        super(ObjectSensor, self).destroy()
        self.actor_list = None

    def get_topic_prefix(self):
        """
        get the topic name of the current entity.

        :return: the final topic name of this object
        :rtype: string
        """
        return "/apollo/perception/" + self.name

    @staticmethod
    def get_blueprint_name():
        """
        Get the blueprint identifier for the pseudo sensor
        :return: name
        """
        return "sensor.pseudo.objects"

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.
        On update map sends:
        - tf global frame
        :return:
        """
        cyber_objects = PerceptionObstacles()
        cyber_objects.header.CopyFrom(self.get_msg_header(frame_id="map", timestamp=timestamp))
        for actor_id in self.actor_list.keys():
            # currently only Vehicles and Walkers are added to the object array
            if self.parent is None or self.parent.uid != actor_id:
                actor = self.actor_list[actor_id]
                if isinstance(actor, Vehicle):
                    cyber_objects.perception_obstacle.append(actor.get_object_info())
                elif isinstance(actor, Walker):
                    cyber_objects.perception_obstacle.append(actor.get_object_info())
        self.object_writer.write(cyber_objects)
