#!/usr/bin/env python
#
# SYNKROTRON Confidential
# Copyright (C) 2023 SYNKROTRON Inc. All rights reserved.
# The source code for this program is not published
# and protected by copyright controlled
#


import os
import json
import time
import signal
import pathlib
import multiprocessing

from utils import log
from customer_algorithm import CustomerAlgo
from oasis.agent.event_listening import event_run


class Adapter:
    def __init__(self):
        self.task_id = ""
        self.customer_pid = {}

    def run(self):
        while True:
            if process_queue.empty():
                # log.debug("queue is empty")
                time.sleep(1)
                continue

            data = process_queue.get(timeout=1)
            data_dict = json.loads(data)
            cmd_type = data_dict.get("command")
            if cmd_type not in ["LoadScenario", "SimulationResult", "Exception"]:
                time.sleep(1)
                continue

            log.info(f"get queue data: {data_dict}")
            self.task_id = data_dict.get("unique_id")
            if cmd_type == "LoadScenario":
                log.info(f"task_id {self.task_id} is running, received: LoadScenario")
                task_status_dict[self.task_id] = "init"

                customer_algo = multiprocessing.Process(
                    target=CustomerAlgo,
                    args=(self.task_id, root_dir, task_status_dict)
                )
                customer_algo.start()

                self.customer_pid[self.task_id] = customer_algo.pid
                log.info(f"task_id {self.task_id} start demo algo {customer_algo.pid}")

            elif cmd_type in ["SimulationResult", "Exception"]:
                log.info(f"task_id {self.task_id} is stopping, received: SimulationResult")
                task_status_dict[self.task_id] = "stop"
                self.stop_process()
                self.customer_pid.clear()

    def stop_process(self):
        pid = self.customer_pid.get(self.task_id)
        if pid is None:
            return

        os.kill(pid, signal.SIGTERM)


if __name__ == "__main__":
    tcs_host_ip = "172.16.19.100"
    tcs_internal_port = 8881

    process_manager = multiprocessing.Manager()
    process_queue = process_manager.Queue()
    task_status_dict = process_manager.dict()

    root_dir = pathlib.Path(__file__).resolve().parent
    # 启动 tcs 事件监听任务的启停状态
    tc = multiprocessing.Process(target=event_run, args=(tcs_host_ip, tcs_internal_port, process_queue, root_dir))
    tc.start()

    Adapter().run()
