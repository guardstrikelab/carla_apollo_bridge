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
import json
import carla
import yaml
import pathlib

from sensor.imu import IMU
from sensor.gnss import GNSS
from sensor.lidar import Lidar
from sensor.camera import Camera
from sensor.obstacle import Obstacle

from utils.map_util import MapUtil
from utils.cyber_node import CyberNode
from utils import log

from core.routing import Routing
from actor.ego_vehicle import EgoVehicle

from cyber.proto.clock_pb2 import Clock
from modules.common_msgs.routing_msgs import routing_pb2

from oasis import OasisSim, PNCInterfaces, SensorInterfaces

root_dir = pathlib.Path(__file__).resolve().parent


class CustomerAlgo(CyberNode):
    def __init__(self, task_id, root_path, task_status_dict):
        super().__init__("oasis_bridge")

        self.args = self._load_config()
        self.task_id = task_id
        self.task_status_dict = task_status_dict
        self.root_path = root_path

        # SDK 中的对象
        self.sim = OasisSim(host=self.args.get("host"), port=self.args.get("carla_port"))
        self.pnc = PNCInterfaces(self.sim)
        self.sensor = SensorInterfaces(self.sim)
        self.ego_vehicle = self.sim.ego_vehicle

        # True Indicates that all signal lights are running properly. Otherwise, all signal lights remain Off
        self.signal_lights_on = False

        self.clock_writer = self.create_writer("/clock", Clock, 10)
        self.routing_req = self.create_writer("/apollo/routing_request", routing_pb2.RoutingRequest)
        self.routing_res = self.create_writer("/apollo/routing_response", routing_pb2.RoutingResponse)

        # carla world
        self.world = self.sim.world
        self.scenario_info = self._get_scenario_info()
        self.map_name = self._get_map_name()
        self.run()

    @staticmethod
    def _load_config():
        config_file = os.path.join(root_dir, "config/settings.yaml")
        with open(config_file, encoding="utf-8") as f:
            args = yaml.safe_load(f)["oasis"]
        return args

    def _get_scenario_info(self):
        scenario_info = {}
        scenario_info_path = os.path.join(self.root_path, "scenario_info.json")
        if os.path.exists(scenario_info_path):
            with open(scenario_info_path, "r") as file:
                scenario_info = json.load(file)
            log.info(f"scenario_info is {scenario_info}")
        return scenario_info

    def run(self):
        log.info("run apollo")

        # 每次运行任务时开启 control 模块，运行完毕后关闭 control 模块
        self.start_apollo_control()

        self._set_map()

        # 发送 routing response 或者 routing request
        self._set_destination()

        # 暂时不需要关注信号灯
        # self._set_signal_lights()

        self._update_actor_factor()

    def _get_map_name(self):
        carla_map_name = self.sim.get_carla_map().name
        # carla_map_name = Carla/Maps/Town01 -> carla_town01
        carla_map_name = carla_map_name.split("/")[-1]
        log.info(f"carla_map_name is {carla_map_name}")
        map_name = f"carla_{carla_map_name.lower()}"
        return map_name

    def _set_map(self):
        # TODO: 每次运行都会通过配置文件设置地图路径，待优化如果不存在再设置
        file_name = "/apollo/modules/common/data/global_flagfile.txt"
        new_line = f"--map_dir=/apollo/modules/map/data/{self.map_name}"
        with open(file_name, "r", encoding="utf-8") as file:
            lines = file.readlines()

        lines[-1] = new_line + "\n"

        with open(file_name, "w", encoding="utf-8") as file:
            file.writelines(lines)

    def _set_destination(self, use_carla_routing=False):
        """
        Set destination and send routing request or response.
        If you use_carla_routing is True, GlobalRoutePlanner will be used to plan the global route
        and send RoutingResponse to Apollo.
        Else RoutingRequest will be sent to Apollo.
        Args:
            use_carla_routing: Whether to use carla routing function. (GlobalRoutePlanner)
        """

        dst_position = self.scenario_info.get("end_position")
        if dst_position is None:
            log.error(f"dst_position is None")

        carla_map = self.sim.get_carla_map()
        # carla_map_name = carla_map.name
        # map_name = f"carla_{carla_map_name.lower()}"

        map_util = MapUtil(self.map_name)
        routing = Routing(carla_map, map_util)
        start_location = self.ego_vehicle.get_location()
        target_location = carla.Location(
            float(dst_position["x"]),
            float(dst_position["y"]) * -1,
            float(dst_position["z"]),
        )
        log.info(f"start_location is {start_location}, dest_location is {target_location}")

        if use_carla_routing:
            routing_response = routing.generate_routing_response(start_location, target_location)
            self.routing_res.write(routing_response)
        else:
            routing_request = routing.generate_routing_request(start_location, target_location)
            self.routing_req.write(routing_request)

    def _find_front_6mm_camera(self):
        while True:
            actors = self.world.get_actors().filter("sensor.camera.rgb")
            gnss_role_name = self.args.get("ego_sensors").get("front_6mm_camera").get("role_name")
            for actor in actors:
                if gnss_role_name == actor.attributes.get('role_name'):
                    log.info("front_6mm_camera sensor found")
                    return actor
                log.warning("not found front_6mm_camera sensor")
            time.sleep(0.1)

    def _set_signal_lights(self):
        if not self.signal_lights_on:
            # turn off all signals
            traffic_lights = self.world.get_actors().filter("traffic.traffic_light")
            for traffic_light in traffic_lights:
                traffic_light.set_state(carla.TrafficLightState.Off)
                traffic_light.freeze(True)

    def _update_actor_factor(self):
        # 先找 Carla 对象
        gnss_role_name = self.args.get("ego_sensors").get("gnss").get("role_name")
        gnss_actor_sensor = self.sensor.get_gnss_sensor(gnss_role_name)
        if gnss_actor_sensor is not None:
            log.info("gnss sensor found")

        imu_role_name = self.args.get("ego_sensors").get("imu").get("role_name")
        imu_actor_sensor = self.sensor.get_imu_sensor(imu_role_name)
        if imu_actor_sensor is not None:
            log.info("imu sensor found")

        lidar_role_name = self.args.get("ego_sensors").get("lidar").get("role_name")
        lidar_actor_sensor = self.sensor.get_ray_cast_lidar_sensor(lidar_role_name)
        if lidar_actor_sensor is not None:
            log.info("lidar sensor found")

        front_6mm_role_name = self.args.get("ego_sensors").get("front_6mm_camera").get("role_name")
        front_camera_actor_sensor = self.sensor.get_rgb_camera_sensor(front_6mm_role_name)
        if front_camera_actor_sensor is not None:
            log.info("front_6mm_camera sensor found")

        gnss_obj = GNSS(gnss_actor_sensor, self.ego_vehicle, self)
        imu_obj = IMU(imu_actor_sensor, self.ego_vehicle, self)
        lidar_obj = Lidar(lidar_actor_sensor, self.ego_vehicle, self)
        camera_actor = self._find_front_6mm_camera()
        camera_obj = Camera(front_camera_actor_sensor, camera_actor, self)
        ego_vehicle_obj = EgoVehicle(self.ego_vehicle, self)

        # 启动雷达感知时，首次需要给 /apollo/perception/obstacles channel 写入空数据，否则 planning 模块报错
        obstacle_obj = Obstacle("obstacle", self.world, self)
        obstacle_obj.update_null_obstacle()

        perception_switch = self.args.get("perception_switch")
        log.warning(f"perception_switch is {perception_switch}, bool type is {bool(perception_switch)}")

        while True:
            self.clock_writer.write(Clock(clock=int(self.get_time())))

            # 更新传感器数据
            gnss_obj.update()
            imu_obj.update()
            lidar_obj.update()
            camera_obj.update()
            # 更新车辆数据
            ego_vehicle_obj.update()

            # 如果感知开关关闭，即仅测试规控，则需要将真实的障碍物更新到 /apollo/perception/obstacles 中
            if perception_switch is False:
                obstacle_obj.update_truth_obstacle()

            if self.task_status_dict.get(self.task_id) == "stop":
                self.task_status_dict.clear()
                log.info(f"task_status_dict is {self.task_status_dict}")
                break

        # 每次运行任务时开启 control 模块，运行完毕后关闭 control 模块
        self.stop_apollo_control()
        log.info(f"task process exit")



