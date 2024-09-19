#!/usr/bin/env python
"""
/*******************************************************************************
 * SYNKROTRON Confidential
 * Copyright (C) 2023 SYNKROTRON Inc. All rights reserved.
 * The source code for this program is not published
 * and protected by copyright controlled
 *******************************************************************************/
"""

import os
import time
import subprocess

from cyber_py3 import cyber, cyber_time, cyber_timer, parameter


class CyberNode(object):

    def __init__(self, name, **kwargs):
        self.kwargs = kwargs
        self.cyber = cyber
        self.cyber.init(name)
        self.node = self.cyber.Node(name)

        self.cyber_time = cyber_time
        self.cyber_timer = cyber_timer
        self.cyber_parameter = parameter

    def get_param(self, name, alternative_value=None):
        return self.cyber_parameter.Parameter(name, alternative_value)

    def get_time(self):
        return self.cyber_time.Time.now().to_sec()

    def create_writer(self, name, data_type, qos_depth=1):
        return self.node.create_writer(name, data_type, qos_depth)

    def create_reader(self, name, data_type, callback, args=None):
        return self.node.create_reader(name, data_type, callback, args)

    def rate(self, frequency):
        return self.cyber_time.Rate(frequency)

    def timer(self, timer_period_sec, callback, oneshot=0):
        return self.cyber_timer.Timer(timer_period_sec * 1000, callback, oneshot)  # ms

    def create_service(self, name, req_type, res_type, callback, args):
        return self.node.create_service(name, req_type, res_type, callback, args)

    def create_client(self, name, req_type, res_type):
        return self.node.create_client(name, req_type, res_type)

    @staticmethod
    def call_service(client, req):
        response = client.send_request(req)
        return response

    def spin(self):
        self.node.spin()

    def destroy(self):
        self.shutdown()

    def ok(self):
        return not self.cyber.is_shutdown()

    def shutdown(self):
        self.cyber.shutdown()

    def wait_for_shutdown(self):
        self.cyber.waitforshutdown()

    def get_timestamp(self, sec=0, nsec=0, from_sec=False):
        total = 0
        if from_sec:
            total = self.cyber_time.Time(float(sec)).to_nsec()
        else:
            total = self.cyber_time.Time(sec * 1000000000 + nsec)
        secs = total / 1000000000
        nsecs = total - secs * 1000000000
        return {"secs": secs, "nsecs": nsecs}

    @staticmethod
    def stop_apollo_control():
        launch_cmd = f"cyber_launch stop /apollo/modules/control/launch/control.launch"
        os.system(launch_cmd)
        time.sleep(1)
        # 上面命令有时候无法让 control.dag 进程退出，所以用下面命令再强制 kill
        get_control_pid_cmd = "ps aux | grep control.dag | grep -v grep | cut -c 10-16"
        pid = subprocess.getoutput(get_control_pid_cmd).strip()
        if pid != "":
            # log.warning(f"apollo_control_pid is {pid}")
            os.system(f"kill -9 {pid}")
        time.sleep(1)

    @staticmethod
    def start_apollo_control():
        launch_cmd = f"nohup cyber_launch start /apollo/modules/control/launch/control.launch &"
        # log.info(f"start control cmd is {launch_cmd}")
        os.system(launch_cmd)
        time.sleep(1)

    def restart_apollo_control(self):
        self.stop_apollo_control()
        self.start_apollo_control()
