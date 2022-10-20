#!/usr/bin/env python
#
# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Rosbridge class:

Class that handle communication between CARLA and Cyber
"""

import sys
import time

sys.path.append("../")

import os
import pkg_resources
try:
    import queue
except ImportError:
    import Queue as queue
import sys
from distutils.version import LooseVersion
from threading import Thread, Lock, Event
import yaml

import carla

import cyber_compatibility as cybercomp
from cyber_compatibility.node import CompatibleNode

from carla_cyber_bridge.actor import Actor
from carla_cyber_bridge.actor_factory import ActorFactory
from carla_cyber_bridge.carla_status_writer import CarlaStatusWriter
# from carla_cyber_bridge.debug_helper import DebugHelper
from carla_cyber_bridge.ego_vehicle import EgoVehicle
from carla_cyber_bridge.world_info import WorldInfo

from cyber_py import cyber
from cyber.carla_bridge.carla_proto.proto.carla_clock_pb2 import Time, Clock
from cyber.carla_bridge.carla_proto.proto.carla_control_pb2 import CarlaControl
from cyber.carla_bridge.carla_proto.proto.carla_spawn_object_pb2 import (
    SpawnObjectRequest,
    SpawnObjectResponse
)
from cyber.carla_bridge.carla_proto.proto.carla_destroy_object_pb2 import (
    DestroyObjectRequest,
    DestroyObjectResponse
)
from cyber.carla_bridge.carla_proto.proto.carla_get_blueprints_pb2 import (
    GetBlueprintsRequest,
    GetBlueprintsResponse
)
from cyber.carla_bridge.carla_proto.proto.carla_weather_parameters_pb2 import CarlaWeatherParameters
from modules.localization.proto.localization_pb2 import LocalizationEstimate, LocalizationStatus
from modules.transform.proto.transform_pb2 import TransformStamped, TransformStampeds
import math
import carla_common.transforms as trans
from cyber.carla_bridge.carla_proto.proto.carla_geometry_pb2 import Twist, Accel
from modules.common.proto.geometry_pb2 import Point3D
import json
import numpy as np

class CarlaCyberBridge(CompatibleNode):

    """
    Carla Cyber bridge
    """

    with open(os.path.join(os.path.dirname(__file__), "CARLA_VERSION")) as f:
        CARLA_VERSION = f.read()[:-1]

    # in synchronous mode, if synchronous_mode_wait_for_vehicle_control_command is True,
    # wait for this time until a next tick is triggered.
    VEHICLE_CONTROL_TIMEOUT = 1.

    def __init__(self):
        """
        Constructor

        :param carla_world: carla world object
        :type carla_world: carla.World
        :param params: dict of parameters, see settings.yaml
        :type params: dict
        """
        super(CarlaCyberBridge, self).__init__("cyber_bridge_node")

    def initialize_bridge(self, carla_world, params):
        """
        Initialize the bridge
        """
        self.parameters = params
        self.carla_world = carla_world

        self.synchronous_mode_update_thread = None
        self.shutdown = Event()
        self.carla_actor = None
        self.carla_tick_count = 0
        self.find_car = False

        self.carla_settings = carla_world.get_settings()
        if not self.parameters["passive"]:
            # workaround: settings can only applied within non-sync mode
            if self.carla_settings.synchronous_mode:
                self.carla_settings.synchronous_mode = False
                carla_world.apply_settings(self.carla_settings)

            self.loginfo("synchronous_mode: {}".format(
                self.parameters["synchronous_mode"]))
            self.carla_settings.synchronous_mode = self.parameters["synchronous_mode"]
            self.loginfo("fixed_delta_seconds: {}".format(
                self.parameters["fixed_delta_seconds"]))
            self.carla_settings.fixed_delta_seconds = self.parameters["fixed_delta_seconds"]
            carla_world.apply_settings(self.carla_settings)

        self.loginfo("Parameters:")
        for key in self.parameters:
            self.loginfo("  {}: {}".format(key, self.parameters[key]))

        # active sync mode in the cyber bridge only if CARLA world is configured in sync mode and
        # passive mode is not enabled.
        self.sync_mode = self.carla_settings.synchronous_mode and not self.parameters["passive"]
        if self.carla_settings.synchronous_mode and self.parameters["passive"]:
            self.loginfo(
                "Passive mode is enabled and CARLA world is configured in synchronous mode. This configuration requires another client ticking the CARLA world.")

        self.carla_control_queue = queue.Queue()

        # actor factory
        self.actor_factory = ActorFactory(self, carla_world, self.sync_mode)

        # add world info
        self.world_info = WorldInfo(carla_world=self.carla_world, node=self)
        # add debug helper
        # self.debug_helper = DebugHelper(carla_world.debug, self)

        # Communication topics
        self.clock_writer = self.new_writer('/clock', Clock, 10)

        self.status_writer = CarlaStatusWriter(
            self.carla_settings.synchronous_mode,
            self.carla_settings.fixed_delta_seconds,
            self)

        # for waiting for ego vehicle control commands in synchronous mode,
        # their ids are maintained in a list.
        # Before tick(), the list is filled and the loop waits until the list is empty.
        self._all_vehicle_control_commands_received = Event()
        self._expected_ego_vehicle_control_command_ids = []
        self._expected_ego_vehicle_control_command_ids_lock = Lock()

        if self.sync_mode:
            self.logdebug("In sync mode statement.")
            self.carla_run_state = CarlaControl.Command.PLAY
            # self.carla_control_reader = \
            #     self.new_reader("/carla/control", CarlaControl,
            #                           self.carla_control_queue.put(self.carla_run_state))
            self.carla_control_reader = \
                self.new_reader("/carla/control", CarlaControl,
                                self.carla_control_queue.put(self.carla_run_state))
            self.synchronous_mode_update_thread = Thread(
                target=self._synchronous_mode_update)
            self.synchronous_mode_update_thread.daemo = True
            self.synchronous_mode_update_thread.start()
            self.loginfo("synchronous_mode_update_thread start")
        else:
            self.logdebug("In non-sync mode statement.")

            self.timestamp_last_run = 0.0

            self.actor_factory.start()

            # register callback to update actors
            self.on_tick_id = self.carla_world.on_tick(self._carla_time_tick)

        self.logdebug("Before services configuration.")

        # services configuration.
        self._registered_actors = []
        self.spawn_object_service = self.new_service("/carla/spawn_object",
                                                     SpawnObjectRequest,
                                                     SpawnObjectResponse,
                                                     self.spawn_object)
        self.destroy_object_service = self.new_service("/carla/destroy_object",
                                                       DestroyObjectRequest,
                                                       DestroyObjectResponse,
                                                       self.destroy_object)

        self.get_blueprints_service = self.new_service("/carla/get_blueprints",
                                                       GetBlueprintsRequest,
                                                       GetBlueprintsResponse,
                                                       self.get_blueprints)

        self.carla_weather_reader = self.new_reader("/carla/weather_control",
                                                    CarlaWeatherParameters,
                                                    self.on_weather_changed)

        self.vehicle_pose_writer = self.new_writer(
            "/apollo/localization/pose",
            LocalizationEstimate,
            qos_depth=10)
        self.localization_status_writer = self.new_writer(
            "/apollo/localization/msf_status",
            LocalizationStatus,
            qos_depth=10)

        self.tf_writer = self.new_writer("/tf", TransformStampeds)


        self.logdebug("carla_weather_reader.")

    def spawn_object(self, req):
        response = SpawnObjectResponse()
        if not self.shutdown.is_set():
            try:
                id_ = self.actor_factory.spawn_actor(req)
                self._registered_actors.append(id_)
                response.id = id_
            except Exception as e:
                self.logwarn("Error spawning object '{}': {}".format(req.type, e))
                response.id = -1
                response.error_string = str(e)
        else:
            response.id = -1
            response.error_string = 'Bridge is shutting down, object will not be spawned.'
        return response

    def destroy_object(self, req):
        response = DestroyObjectResponse()
        destroyed_actors = self.actor_factory.destroy_actor(req.id)
        response.success = bool(destroyed_actors)
        for actor in destroyed_actors:
            if actor in self._registered_actors:
                self._registered_actors.remove(actor)
        return response

    def get_blueprints(self, req):
        response = GetBlueprintsResponse()
        if req.filter:
            bp_filter = req.filter
        else:
            bp_filter = "*"

        response.blueprints = [
            bp.id for bp in self.carla_world.get_blueprint_library().filter(bp_filter)]
        response.blueprints.extend(self.actor_factory.get_pseudo_sensor_types())
        response.blueprints.sort()
        return response

    def on_weather_changed(self, weather_parameters):
        """
        Callback on new weather parameters
        :return:
        """
        if not self.carla_world:
            return
        self.loginfo("Applying weather parameters...")
        weather = carla.WeatherParameters()
        weather.cloudiness = weather_parameters.cloudiness
        weather.precipitation = weather_parameters.precipitation
        weather.precipitation_deposits = weather_parameters.precipitation_deposits
        weather.wind_intensity = weather_parameters.wind_intensity
        weather.fog_density = weather_parameters.fog_density
        weather.fog_distance = weather_parameters.fog_distance
        weather.wetness = weather_parameters.wetness
        weather.sun_azimuth_angle = weather_parameters.sun_azimuth_angle
        weather.sun_altitude_angle = weather_parameters.sun_altitude_angle
        self.carla_world.set_weather(weather)

    def process_run_state(self):
        """
        process state changes
        """
        command = None

        # get last command
        while not self.carla_control_queue.empty():
            command = self.carla_control_queue.get()

        while command is not None and cybercomp.ok():
            self.carla_run_state = command

            if self.carla_run_state == CarlaControl.Command.PAUSE:
                # wait for next command
                self.loginfo("State set to PAUSED")
                self.status_writer.set_synchronous_mode_running(False)
                command = self.carla_control_queue.get()
            elif self.carla_run_state == CarlaControl.Command.PLAY:
                self.loginfo("State set to PLAY")
                self.status_writer.set_synchronous_mode_running(True)
                return
            elif self.carla_run_state == CarlaControl.Command.STEP_ONCE:
                self.loginfo("Execute single step.")
                self.status_writer.set_synchronous_mode_running(True)
                self.carla_control_queue.put(CarlaControl.Command.PAUSE)
                return

    def _synchronous_mode_update(self):
        """
        execution loop for synchronous mode
        """
        last_car_pos = 0
        while not self.shutdown.is_set():
            self.process_run_state()
            if self.parameters['synchronous_mode_wait_for_vehicle_control_command']:
                # fill list of available ego vehicles
                self._expected_ego_vehicle_control_command_ids = []
                with self._expected_ego_vehicle_control_command_ids_lock:
                    for actor_id, actor in self.actor_factory.actors.items():
                        if isinstance(actor, EgoVehicle):
                            self._expected_ego_vehicle_control_command_ids.append(
                                actor_id)
                            self.loginfo("actor_id is {}".format(actor_id))
            frame = self.carla_world.tick()
            world_snapshot = self.carla_world.get_snapshot()

            self.status_writer.set_frame(frame)
            self.update_clock(world_snapshot.timestamp)
            # self.logdebug("Tick for frame {} returned. Waiting for sensor data...".format(
            #     frame))
            self._update(frame, world_snapshot.timestamp.elapsed_seconds)
            # self.logdebug("Waiting for sensor data finished.")
            self.actor_factory.update_available_objects()

            if self.parameters['synchronous_mode_wait_for_vehicle_control_command']:
                # wait for all ego vehicles to send a vehicle control command
                if self._expected_ego_vehicle_control_command_ids:
                    if not self._all_vehicle_control_commands_received.wait(CarlaCyberBridge.VEHICLE_CONTROL_TIMEOUT):
                        self.logwarn("Timeout ({}s) while waiting for vehicle control commands. "
                                     "Missing command from actor ids {}".format(CarlaCyberBridge.VEHICLE_CONTROL_TIMEOUT,
                                                                                self._expected_ego_vehicle_control_command_ids))
                    self._all_vehicle_control_commands_received.clear()





    def _carla_time_tick(self, carla_snapshot):
        """
        Private callback registered at carla.World.on_tick()
        to trigger cyclic updates.

        After successful locking the update mutex
        (only perform trylock to respect bridge processing time)
        the clock and the children are updated.
        Finally the Cyber messages collected to be writed are sent out.

        :param carla_timestamp: the current carla time
        :type carla_timestamp: carla.Timestamp
        :return:
        """
        if cybercomp.ok():
            if self.timestamp_last_run < carla_snapshot.timestamp.elapsed_seconds:
                self.timestamp_last_run = carla_snapshot.timestamp.elapsed_seconds
                self.update_clock(carla_snapshot.timestamp)
                self.status_writer.set_frame(carla_snapshot.frame)
                self._update(carla_snapshot.frame,
                             carla_snapshot.timestamp.elapsed_seconds)

    def _update(self, frame_id, timestamp):
        """
        update all actors
        :return:
        """
        self.world_info.update(frame_id, timestamp)
        self.actor_factory.update_actor_states(frame_id, timestamp)

    def _ego_vehicle_control_applied_callback(self, ego_vehicle_id):
        if not self.sync_mode or \
                not self.parameters['synchronous_mode_wait_for_vehicle_control_command']:
            return
        with self._expected_ego_vehicle_control_command_ids_lock:
            if ego_vehicle_id in self._expected_ego_vehicle_control_command_ids:
                self._expected_ego_vehicle_control_command_ids.remove(
                    ego_vehicle_id)
            else:
                self.logwarn(
                    "Unexpected vehicle control command received from {}".format(ego_vehicle_id))
            if not self._expected_ego_vehicle_control_command_ids:
                self._all_vehicle_control_commands_received.set()

    def update_clock(self, carla_timestamp):
        """
        perform the update of the clock

        :param carla_timestamp: the current carla time
        :type carla_timestamp: carla.Timestamp
        :return:
        """
        if cybercomp.ok():
            self.timestamp = cybercomp.get_timestamp(carla_timestamp.elapsed_seconds, from_sec=True)
            self.clock_writer.write(Clock(clock=Time(secs=self.timestamp['secs'],
                                                     nsecs=self.timestamp['nsecs'])))

    def destroy(self):
        """
        Function to destroy this object.

        :return:
        """
        self.loginfo("Shutting down...")
        if not self.sync_mode:
            if self.on_tick_id:
                self.carla_world.remove_on_tick(self.on_tick_id)
            # self.actor_factory.thread.join()
        else:
            self.synchronous_mode_update_thread.join()
        self.loginfo("Object update finished.")
        # self.debug_helper.destroy()
        self.status_writer.destroy()
        self.carla_control_queue.put(CarlaControl.Command.STEP_ONCE)

        for uid in self._registered_actors:
            self.actor_factory.destroy_actor(uid)
        self.actor_factory.update_available_objects()
        self.actor_factory.clear()
        super(CarlaCyberBridge, self).destroy()


def main(args=None):
    """
    main function for carla simulator Cyber bridge
    maintaining the communication client and the CarlaBridge object
    """
    carla_bridge = None
    carla_world = None
    carla_client = None
    executor = None
    parameters = {}

    executor = cybercomp.executors.MultiThreadedExecutor()
    carla_bridge = CarlaCyberBridge()
    executor.add_node(carla_bridge)

    config_file = os.path.dirname(__file__) + "/config/settings.yaml"
    carla_bridge.loginfo("The config file path is {}.".format(config_file))

    parameters = yaml.safe_load(open(config_file))['carla']

    # self.loginfo(parameters)



    carla_bridge.loginfo("Trying to connect to {host}:{port}".format(
        host=parameters['host'], port=parameters['port']))

    try:
        carla_client = carla.Client(
            host=parameters['host'],
            port=parameters['port'])
        carla_client.set_timeout(parameters['timeout'])

        # check carla version
        dist = pkg_resources.get_distribution("carla")
        if LooseVersion(dist.version) != LooseVersion(CarlaCyberBridge.CARLA_VERSION):
            carla_bridge.logfatal("CARLA python module version {} required. Found: {}".format(
                CarlaCyberBridge.CARLA_VERSION, dist.version))
            sys.exit(1)

        if LooseVersion(carla_client.get_server_version()) != \
           LooseVersion(carla_client.get_client_version()):
            carla_bridge.logwarn(
                "Version mismatch detected: You are trying to connect to a simulator that might be incompatible with this API. Client API version: {}. Simulator API version: {}"
                .format(carla_client.get_client_version(),
                        carla_client.get_server_version()))

        carla_world = carla_client.get_world()

        if "town" in parameters and not parameters['passive']:
            if parameters["town"].endswith(".xodr"):
                carla_bridge.loginfo(
                    "Loading opendrive world from file '{}'".format(parameters["town"]))
                with open(parameters["town"]) as od_file:
                    data = od_file.read()
                carla_world = carla_client.generate_opendrive_world(str(data))
            else:
                if carla_world.get_map().name != parameters["town"]:
                    carla_bridge.loginfo("Loading town '{}' (previous: '{}').".format(
                        parameters["town"], carla_world.get_map().name))
                    carla_world = carla_client.load_world(parameters["town"])
            # carla_world.tick()

        carla_bridge.initialize_bridge(carla_world, parameters)
        # carla_bridge.initialize_bridge(carla_client.get_world(), parameters)


        spectator = carla_world.get_spectator()
        ego_vehicle = carla_world.get_actor(258)
        loc = ego_vehicle.get_transform().location
        spectator.set_transform(carla.Transform(carla.Location(x=loc.x,y=loc.y,z=35),carla.Rotation(yaw=0,pitch=-90,roll=0)))

        carla_bridge.loginfo("spin ...")
        carla_bridge.spin()

    except (IOError, RuntimeError) as e:
        carla_bridge.logerr("Error: {}".format(e))
    except KeyboardInterrupt as e:
        carla_bridge.logerr("Error: {}".format(e))
    finally:
        carla_world = carla_client.get_world()
        settings = carla_world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        carla_world.apply_settings(settings)
        carla_bridge.logerr("Shutting down.")
        carla_bridge.destroy()
        cybercomp.shutdown()
        del carla_world
        del carla_client
        os._exit(0)


if __name__ == "__main__":
    main()
