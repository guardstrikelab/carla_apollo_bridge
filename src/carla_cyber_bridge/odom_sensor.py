#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
handle a odom sensor
"""

from carla_cyber_bridge.pseudo_actor import PseudoActor

from cyber.carla_bridge.carla_proto.proto.carla_odometry_pb2 import Odometry


class OdometrySensor(PseudoActor):

    """
    Pseudo odometry sensor
    """

    def __init__(self, uid, name, parent, node):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param carla_world: carla world object
        :type carla_world: carla.World
        :param parent: the parent of this
        :type parent: carla_cyber_bridge.Parent
        :param node: node-handle
        :type node: carla_cyber_bridge.CarlaCyberBridge
        """

        super(OdometrySensor, self).__init__(uid=uid,
                                             name=name,
                                             parent=parent,
                                             node=node)

        self.odometry_writer = node.new_writer("/apollo/sensor/odometry",  # carla odometry sensor
                                               Odometry,
                                               qos_depth=10)

    def destroy(self):
        super(OdometrySensor, self).destroy()

    @staticmethod
    def get_blueprint_name():
        """
        Get the blueprint identifier for the pseudo sensor
        :return: name
        """
        return "sensor.pseudo.odom"

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.
        """
        odometry = Odometry(header=self.parent.get_msg_header("map", timestamp=timestamp))
        odometry.child_frame_id = self.parent.get_prefix()
        try:
            odometry.pose.CopyFrom(self.parent.get_current_cyber_pose())
            odometry.twist.CopyFrom(self.parent.get_current_cyber_twist_rotated())
        except AttributeError:
            # parent actor disappeared, do not send tf
            self.node.logwarn(
                "OdometrySensor could not write. parent actor {} not found".format(self.parent.uid))
            return
        self.odometry_writer.write(odometry)
