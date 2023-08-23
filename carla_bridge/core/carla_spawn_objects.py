#!/usr/bin/env python
#
# Copyright (c) 2019-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
base class for spawning objects (carla actors and pseudo_actors) in Cyber

Gets config file from ros parameter ~objects_definition_file and spawns corresponding objects
through Cyber service /carla/spawn_object.

Looks for an initial spawn point first in the launchfile, then in the config file, and
finally ask for a random one to the spawn service.

"""

import sys

from carla_bridge.actor.ego_vehicle import EgoVehicle
from carla_bridge.core.spawn_object_param import SpawnObjectParam, KeyValue

sys.path.append("../")

import json
import math
import os

from transforms3d.euler import euler2quat

from modules.common_msgs.localization_msgs.pose_pb2 import Pose


class CarlaSpawnObjects:

    """
    Handles the spawning of the ego vehicle and its sensors

    Derive from this class and implement method sensors()
    """

    def __init__(self, bridge_node):
        self.bridge_node = bridge_node
        self.log = bridge_node.log
        self.actor_factory = bridge_node.actor_factory

        self.objects_definition_file = (
                os.path.dirname(os.path.dirname(__file__)) + "/config/objects.json"
        )

        self.players = []
        self.vehicles_sensors = []
        self.global_sensors = []

    def spawn_objects(self):
        """
        Spawns the objects

        Either at a given spawnpoint or at a random Carla spawnpoint

        :return:
        """
        # Read sensors from file
        if not self.objects_definition_file or not os.path.exists(self.objects_definition_file):
            raise RuntimeError(
                f"Could not read object definitions from {self.objects_definition_file}")
        with open(self.objects_definition_file) as handle:
            json_actors = json.loads(handle.read())

        global_sensors = []
        vehicles = []

        for actor in json_actors["objects"]:
            actor_type = actor["type"].split('.')[0]
            if actor["type"] == "sensor.pseudo.actor_list":
                global_sensors.append(actor)
            elif actor_type == "sensor":
                global_sensors.append(actor)
            elif actor_type == "vehicle" or actor_type == "walker":
                vehicles.append(actor)
            else:
                self.log.warn(
                    "Object with type {} is not a vehicle, a walker or a sensor, ignoring".format(actor["type"]))

        self.setup_sensors(global_sensors)
        self.setup_vehicles(vehicles)
        
        self.log.info("All objects spawned.")

    def setup_vehicles(self, vehicles):
        for vehicle in vehicles:
            spawn_object_param = SpawnObjectParam()
            spawn_object_param.type = vehicle["type"]
            spawn_object_param.id = vehicle["id"]
            spawn_object_param.attach_to = 0
            spawn_object_param.random_pose = False

            spawn_point = None

            if "spawn_point" in vehicle:
                # get spawn point from config file
                try:
                    spawn_point = self.create_spawn_point(
                        vehicle["spawn_point"]["x"],
                        vehicle["spawn_point"]["y"],
                        vehicle["spawn_point"]["z"],
                        vehicle["spawn_point"]["roll"],
                        vehicle["spawn_point"]["pitch"],
                        vehicle["spawn_point"]["yaw"]
                    )
                    self.log.info("Spawn point from configuration file")
                except KeyError as e:
                    self.log.error("{}: Could not use the spawn point from config file, ".format(vehicle["id"]) +
                                "the mandatory attribute {} is missing, a random spawn point will be used".format(e))

            if spawn_point is None:
                # pose not specified, ask for a random one in the service call
                self.log.info("Spawn point selected at random")
                spawn_point = Pose()  # empty pose
                spawn_object_param.random_pose = True

            spawn_object_param.transform.CopyFrom(spawn_point)
            self.bridge_node.spawn_object(spawn_object_param)

            ego_vehicle_spawned = False
            while not ego_vehicle_spawned:
                actors = self.actor_factory.actors.copy()
                for uid, act in actors.items():
                    if isinstance(act, EgoVehicle):
                        ego_vehicle_spawned = True
                        self.setup_sensors(vehicle["sensors"], uid)
                        break
                    
    def setup_sensors(self, sensors, attached_vehicle_id=None):
        """
        Create the sensors defined by the user and attach them to the vehicle
        (or not if global sensor)
        :param sensors: list of sensors
        :param attached_vehicle_id: id of vehicle to attach the sensors to
        :return actors: list of ids of objects created
        """
        sensor_names = []
        for sensor_spec in sensors:
            self.log.debug(f"sensor: {sensor_spec}, attach: {attached_vehicle_id}")
            if not self.bridge_node.ok():
                break
            try:
                sensor_type = str(sensor_spec.pop("type"))
                sensor_id = str(sensor_spec.pop("id"))

                sensor_name = sensor_type + "/" + sensor_id
                if sensor_name in sensor_names:
                    raise NameError
                sensor_names.append(sensor_name)

                if attached_vehicle_id is None and "pseudo" not in sensor_type:
                    spawn_point = sensor_spec.pop("spawn_point")
                    sensor_transform = self.create_spawn_point(
                        spawn_point.pop("x"),
                        spawn_point.pop("y"),
                        spawn_point.pop("z"),
                        spawn_point.pop("roll", 0.0),
                        spawn_point.pop("pitch", 0.0),
                        spawn_point.pop("yaw", 0.0))
                else:
                    # if sensor attached to a vehicle, or is a 'pseudo_actor', allow default pose
                    spawn_point = sensor_spec.pop("spawn_point", 0)
                    if spawn_point == 0:
                        sensor_transform = self.create_spawn_point(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                    else:
                        sensor_transform = self.create_spawn_point(
                            spawn_point.pop("x", 0.0),
                            spawn_point.pop("y", 0.0),
                            spawn_point.pop("z", 0.0),
                            spawn_point.pop("roll", 0.0),
                            spawn_point.pop("pitch", 0.0),
                            spawn_point.pop("yaw", 0.0))

                spawn_object_param = SpawnObjectParam()
                spawn_object_param.type = sensor_type
                spawn_object_param.id = sensor_id
                spawn_object_param.attach_to = attached_vehicle_id if attached_vehicle_id is not None else 0
                spawn_object_param.transform.CopyFrom(sensor_transform)
                spawn_object_param.random_pose = False  # never set a random pose for a sensor

                attached_objects = []
                for attribute, value in sensor_spec.items():
                    if attribute == "attached_objects":
                        for attached_object in sensor_spec["attached_objects"]:
                            attached_objects.append(attached_object)
                        continue
                    spawn_object_param.attributes.append(
                        KeyValue(key=str(attribute), value=str(value)))

                response_id = self.bridge_node.spawn_object(spawn_object_param)

                if attached_objects:
                    # spawn the attached objects
                    self.setup_sensors(attached_objects, response_id)

                if attached_vehicle_id is None:
                    self.global_sensors.append(response_id)
                else:
                    self.vehicles_sensors.append(response_id)

            except KeyError as e:
                self.log.error(
                    "Sensor {} will not be spawned, the mandatory attribute {} is missing".format(sensor_name, e))
                continue

            except RuntimeError as e:
                self.log.error(
                    "Sensor {} will not be spawned: {}".format(sensor_name, e))
                continue

            except NameError:
                self.log.error("Sensor rolename '{}' is only allowed to be used once. The second one will be ignored.".format(
                    sensor_id))
                continue

    def create_spawn_point(self, x, y, z, roll, pitch, yaw):
        spawn_point = Pose()
        spawn_point.position.x = x
        spawn_point.position.y = y
        spawn_point.position.z = z
        quat = euler2quat(math.radians(roll), math.radians(pitch), math.radians(yaw))

        spawn_point.orientation.qx = quat[1]
        spawn_point.orientation.qy = quat[2]
        spawn_point.orientation.qz = quat[3]
        spawn_point.orientation.qw = quat[0]
        return spawn_point

    def destroy(self):
        """
        destroy all the players and sensors
        """
        self.log.info("Destroying spawned objects...")
        # destroy vehicles sensors
        for actor_id in self.vehicles_sensors:
            if self.bridge_node.destroy_object(actor_id):
                self.log.info("Object {} successfully destroyed.".format(actor_id))
        self.vehicles_sensors = []

        # destroy global sensors
        for actor_id in self.global_sensors:
            if self.bridge_node.destroy_object(actor_id):
                self.log.info("Object {} successfully destroyed.".format(actor_id))
        self.global_sensors = []

        # destroy player
        for player_id in self.players:
            if self.bridge_node.destroy_object(player_id):
                self.log.info("Object {} successfully destroyed.".format(player_id))
        self.players = []
