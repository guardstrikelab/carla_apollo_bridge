#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
handle a tf sensor
"""

import os

import cyber_compatibility as cybercomp

from carla_cyber_bridge.pseudo_actor import PseudoActor

from modules.transform.proto.transform_pb2 import Transform, TransformStamped


class TFSensor(PseudoActor):

    """
    Pseudo tf sensor
    """

    def __init__(self, uid, name, parent, node):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying the sensor
        :type name: string
        :param parent: the parent of this
        :type parent: carla_cyber_bridge.Parent
        :param node: node-handle
        :type node: carla_cyber_bridge.CarlaCyberBridge
        """

        super(TFSensor, self).__init__(uid=uid,
                                       name=name,
                                       parent=parent,
                                       node=node)

        # self._tf_broadcaster = tf2_cyber.TransformBroadcaster()

    @staticmethod
    def get_blueprint_name():
        """
        Get the blueprint identifier for the pseudo sensor
        :return: name
        """
        return "sensor.pseudo.tf"

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.
        """
        self.parent.get_prefix()

        transform = Transform()
        try:
            transform.CopyFrom(self.parent.get_current_cyber_transform())
        except AttributeError:
            # parent actor disappeared, do not send tf
            self.node.logwarn(
                "TFSensor could not write transform. Actor {} not found".format(self.parent.uid))
            return

        # self._tf_broadcaster.sendTransform(TransformStamped(
        #     header=self.get_msg_header("map", timestamp=timestamp),
        #     child_frame_id=self.parent.get_prefix(),
        #     transform=transform))
