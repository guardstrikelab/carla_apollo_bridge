#!/usr/bin/env python

#
# Copyright (c) 2021 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

from cyber_py import cyber, cyber_time, cyber_timer, parameter

from cyber_compatibility.loggings import logdebug, loginfo, logwarn, logerr, logfatal
from cyber_compatibility.exceptions import *


class CompatibleNode(object):

    def __init__(self, name, **kwargs):
        cyber.init()
        self.node = cyber.Node(name)

    def get_param(self, name, alternative_value=None):
        return parameter.Parameter(name, alternative_value)

    def get_param(self, name, alternative_value=None):
        return parameter.Parameter(name, alternative_value)

    def get_time(self):
        return cyber_time.Time.now().to_sec()

    def logdebug(self, msg):
        logdebug(msg)

    def loginfo(self, msg):
        loginfo(msg)

    def logwarn(self, msg):
        logwarn(msg)

    def logerr(self, msg):
        logerr(msg)

    def logfatal(self, msg):
        logfatal(msg)

    def new_writer(self, name, data_type, qos_depth=1):
        return self.node.create_writer(name, data_type, qos_depth)

    def new_reader(self, name, data_type, callback, args=None):
        return self.node.create_reader(name, data_type, callback, args)

    def new_rate(self, frequency):
        return cyber_time.Rate(frequency)

    def new_timer(self, timer_period_sec, callback, oneshot=0):
        return cyber_timer.Timer(timer_period_sec * 1000, callback, oneshot)  # ms

    # def wait_for_message(self, topic, msg_type, timeout=None, qos_depth=None):
    #     try:
    #         return cyber.wait_for_message(topic, msg_type, timeout)
    #     except cyber.ROSException as e:
    #         raise ROSException(e)

    def new_service(self, name, req_type, res_type, callback, args=None):
        return self.node.create_service(name, req_type, res_type, callback, args)

    def new_client(self, name, req_type, res_type):
        return self.node.create_client(name, req_type, res_type)

    def call_service(self, client, req):
        try:
            response = client.send_request(req)
            return response
        except ServiceException as e:
            raise ServiceException(e)

    def spin(self):
        self.node.spin()

    def destroy(self):
        del self.node
