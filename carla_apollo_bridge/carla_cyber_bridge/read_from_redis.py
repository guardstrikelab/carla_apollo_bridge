#!/usr/bin/env python3

# ****************************************************************************
# Copyright 2019 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ****************************************************************************
# -*- coding: utf-8 -*-
"""Module for example of talker."""

import os
import time
import redis
import datetime

# from modules.control.proto.control_cmd_pb2 import ControlCommand
# from modules.localization.proto.localization_pb2 import LocalizationEstimate
# from cyber_py import cyber, cyber_time, cyber_timer, parameter

r = redis.Redis(host="172.16.19.58", port=6379, db=1, password="123456", decode_responses=True)

def test_talker_class():
    """
    Test talker.
    """
    # localization = LocalizationEstimate()

    # test_node = cyber.Node("read_from_redis")
    r.ltrim("apollo_pose", 1, 0)
    while True:
        if r.llen("apollo_pose") != 0:
            localization = eval(r.rpop("apollo_pose"))
            print("localization is {}".format(localization))
        time.sleep(0.01)

    # content = r.rpop("apollo_pose")
    # print("content is {}".format(content))
    # localization = eval(content)
    # print("localization is {}, type is {}".format(localization, type(localization)))


    # r.ltrim("apollo_control", 1, 0)
    # while True:
    #     if r.llen("apollo_control") != 0:
    #         content = r.rpop("apollo_control").replace("'", "")
    #         cmd = '''rostopic pub /carla/ego_vehicle/vehicle_control_cmd carla_msgs/CarlaEgoVehicleControl "{}" -1 '''.format(
    #             content)
    #         # os.system(cmd)
    #         print("cmd is {}".format(cmd))
    #     time.sleep(0.01)


    # while r.llen("apollo_control") != 0:
    #     # content = r.rpop("apollo_control")
    #     # print("content is {}".format(content))
    #     # control = eval(content)
    #     # print("control is {}, type is {}".format(control, type(control)))
    #     content = r.rpop("apollo_control").replace("'", "")
    #     cmd = '''rostopic pub /carla/ego_vehicle/vehicle_control_cmd carla_msgs/CarlaEgoVehicleControl "{}" -1 '''.format(content)
    #     os.system(cmd)
    #     time.sleep(0.01)

    # content = r.rpop("apollo_control")
    # print("content is {}".format(content))

def read_control():
    while r.llen("apollo_control") != 0:
        content = r.rpop("apollo_control")
        print("content is {}".format(content))
        control = eval(content)
        time.sleep(0.01)


if __name__ == '__main__':
    read_control()
