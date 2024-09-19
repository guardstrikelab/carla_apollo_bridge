#!/usr/bin/env python

import math
from utils import log
from utils.transforms import carla_transform_to_cyber_pose


from modules.common_msgs.basic_msgs.header_pb2 import Header
from modules.common_msgs.perception_msgs.perception_obstacle_pb2 import PerceptionObstacle, PerceptionObstacles


class Obstacle:
    def __init__(self, name, world, node):
        self.node = node
        self.name = name
        self.world = world

        self.obstacle_writer = self.node.create_writer(
            "/apollo/perception/obstacles",
            PerceptionObstacles,
            qos_depth=10
        )

    def get_msg_header(self, frame_id=None, timestamp=None):
        header = Header()
        if frame_id:
            header.frame_id = frame_id
        else:
            header.frame_id = "obstacle"

        if not timestamp:
            timestamp = self.node.get_time()
        header.timestamp_sec = timestamp
        return header

    def update_null_obstacle(self):
        cyber_objects = PerceptionObstacles()
        cyber_objects.header.CopyFrom(self.get_msg_header(frame_id="obstacle"))
        self.obstacle_writer.write(cyber_objects)

    def update_truth_obstacle(self):
        classification = PerceptionObstacle.Type.UNKNOWN

        cyber_objects = PerceptionObstacles()
        cyber_objects.header.CopyFrom(self.get_msg_header(frame_id="obstacle"))

        # 暂时只统计车辆障碍物
        actors = [actor for actor in self.world.get_actors() if actor.type_id.startswith("vehicle")]
        for actor in actors:
            if actor.attributes.get("role_name") == "ego_vehicle":
                # log.debug(f"ego vehicle is: {actor}")
                continue
            # log.info(f"actor is {actor}, id: {actor.id}, attributes: {actor.attributes}, type_id: {actor.type_id}")

            # vehicle and bicycle
            base_type = actor.attributes.get("base_type")
            if base_type in ["car", "truck", "van"]:
                classification = PerceptionObstacle.VEHICLE
            elif base_type in ["bicycle", "motorcycle"]:
                classification = PerceptionObstacle.BICYCLE
            elif base_type in ["other"]:
                classification = PerceptionObstacle.UNKNOWN_MOVABLE
            special_type = actor.attributes.get("special_type")
            if special_type in ["motorcycle"]:
                classification = PerceptionObstacle.BICYCLE

            # walker
            if actor.type_id.startswith("walker"):
                classification = PerceptionObstacle.PEDESTRIAN

            # static
            if actor.type_id.startswith("static.prop"):
                classification = PerceptionObstacle.UNKNOWN_UNMOVABLE

            cyber_objects.perception_obstacle.append(self._get_object_info(actor, classification))

        self.obstacle_writer.write(cyber_objects)

    @staticmethod
    def _get_object_info(actor, classification):
        obj = PerceptionObstacle()

        obj.id = actor.id

        cyber_pose = carla_transform_to_cyber_pose(actor.get_transform())
        obj.position.x = cyber_pose.position.x
        obj.position.y = cyber_pose.position.y
        obj.position.z = cyber_pose.position.z

        obj.theta = -math.radians(actor.get_transform().rotation.yaw)
        # Velocity
        carla_velocity = actor.get_velocity()
        obj.velocity.x = carla_velocity.x
        obj.velocity.y = -carla_velocity.y
        obj.velocity.z = carla_velocity.z

        # Acceleration
        carla_accel = actor.get_acceleration()
        obj.acceleration.x = carla_accel.x
        obj.acceleration.y = -carla_accel.y
        obj.acceleration.z = carla_accel.z

        # Shape
        obj.length = actor.bounding_box.extent.x * 2.0
        obj.width = actor.bounding_box.extent.y * 2.0
        obj.height = actor.bounding_box.extent.z * 2.0

        # Oasis / Carla 的自行车/摩托车的三维为 0 问题的临时解决方案
        if obj.length == 0 and obj.width == 0 and obj.height == 0 and classification == 4:
            obj.length += 1.8
            obj.width += 0.6
            obj.height += 1.6

        # Classification if available in attributes
        obj.type = classification

        return obj

