#!/usr/bin/env python
#
# SYNKROTRON Confidential
# Copyright (C) 2023 SYNKROTRON Inc. All rights reserved.
# The source code for this program is not published
# and protected by copyright controlled
#
"""
Classes to map util
"""
import math

from carla import Location
from modules.common_msgs.basic_msgs.geometry_pb2 import PointENU
from modules.common_msgs.map_msgs import map_pb2
from modules.tools.common import proto_utils


def _get_distance_from_lane(lane, my_point: PointENU):
    """
    Obtain the minimum distance between the given point and the lane.
    Also returns the nearest lane central point from the given point.
    Args:
        lane: lane, see map_lane.proto
        my_point: the point to compute distance from the lane.
    """
    min_distance = float("inf")
    central_curve = lane.central_curve

    for i in range(len(central_curve.segment)):
        for curve_point in central_curve.segment[i].line_segment.point:
            distance = _compute_distance_of_points(curve_point, my_point)
            if distance < min_distance:
                min_distance = distance
                print(f"lane id: {lane.id.id}, min_distance: {min_distance}")

    return min_distance


def _compute_s_from_start(lane, point: PointENU):
    """
    Given a lane and a point within the lane,
    calculate the distance between the point and the starting point of the lane (s)
    """
    result = 0.0
    start_position = lane.central_curve.segment[0].start_position
    for curve_point in lane.central_curve.segment[0].line_segment.point:
        if _point_equals(start_position, curve_point):
            continue
        result += _compute_distance_of_points(start_position, curve_point)
        start_position = curve_point
        if _point_equals(point, curve_point):
            break
    return result


def _compute_distance_of_points(point_a, point_b):
    return math.sqrt((point_a.x - point_b.x) ** 2 + (point_a.y - point_b.y) ** 2)


def _point_equals(point_a, point_b):
    return point_a.x == point_b.x and point_a.y == point_b.y


def _calculate_projection(point_a, point_b, point_c):
    """
    Given the coordinates of points a, b, and c, find the coordinates of the projection point of point c on line ab.
    """
    dot_product = (point_c.x - point_a.x) * (point_b.x - point_a.x) + (
        point_c.y - point_a.y
    ) * (point_b.y - point_a.y)
    length_sqrd = (point_b.x - point_a.x) ** 2 + (point_b.y - point_a.y) ** 2

    point_d = PointENU()
    point_d.x = round(
        point_a.x + (dot_product / length_sqrd) * (point_b.x - point_a.x), 2
    )
    point_d.y = round(
        point_a.y + (dot_product / length_sqrd) * (point_b.y - point_a.y), 2
    )
    point_d.z = 0

    return point_d


def _is_point_in_segment(point_a, point_b, point_d):
    """
    Given the coordinates of points a, b, and d, determine whether point d is located on line segment ab.
    """
    length_ab = round(
        math.sqrt((point_b.x - point_a.x) ** 2 + (point_b.y - point_a.y) ** 2), 2
    )
    length_ad = round(
        (point_d.x - point_a.x) * (point_b.x - point_a.x)
        + (point_d.y - point_a.y) * (point_b.y - point_a.y),
        2,
    )
    length_bd = round(
        (point_d.x - point_b.x) * (point_a.x - point_b.x)
        + (point_d.y - point_b.y) * (point_a.y - point_b.y),
        2,
    )

    return length_ad <= length_ab and length_bd <= length_ab


def _is_between(number, bound1, bound2):
    lower_bound = min(bound1, bound2)
    upper_bound = max(bound1, bound2)
    return lower_bound <= number <= upper_bound


class MapUtil:
    """map util"""

    def __init__(self, map_name):
        self.map_pb = map_pb2.Map()
        self.set_map(map_name)

    def _load(self, map_name):
        res = proto_utils.get_pb_from_file(map_name, self.map_pb)
        return res is not None

    def set_map(self, map_name):
        map_name = f"/apollo/modules/map/data/{map_name}/base_map.bin"
        self._load(map_name)

    def get_lane_of_point(self, point: PointENU):
        """
        If the point is within the lane, the distance should be the smallest.
        Args:
            point: the point to compute distance from the lane.
        Returns:
            lane_id and s (Can form a waypoint)
        """
        min_distance = float("inf")
        nearest_lane = None
        candidate_nearest_lanes = []

        for lane in self.map_pb.lane:
            distance = _get_distance_from_lane(lane, point)
            if distance < min_distance:
                min_distance = distance
                candidate_nearest_lanes.clear()
                candidate_nearest_lanes.append(lane)
            elif distance == min_distance:
                candidate_nearest_lanes.append(lane)

        if len(candidate_nearest_lanes) > 1:
            for candidate in candidate_nearest_lanes:
                start_point = candidate.central_curve.segment[0].line_segment.point[0]
                end_point = candidate.central_curve.segment[0].line_segment.point[-1]
                projection_point = _calculate_projection(start_point, end_point, point)

                if not ((point.x > start_point.x) ^ (point.x < end_point.x)) or not (
                    (point.y > start_point.y) ^ (point.y < end_point.y)
                ):
                    nearest_lane = candidate

                if _is_between(
                    projection_point.x, start_point.x, end_point.x
                ) and _is_between(projection_point.y, start_point.y, end_point.y):
                    nearest_lane = candidate

        else:
            nearest_lane = candidate_nearest_lanes[0]

        return nearest_lane

    def get_apollo_s_coordinate(self, lane, carla_map, carla_s_coordinate):
        """
        Due to the possible inconsistency between the starting point of the lane on the Carla map and the starting point
        of the lane on the Apollo map, it may result in different s-coordinates of the same point on the same lane.
        This method can convert the s-coordinate of the lane in the Carla map to the correct s-coordinate of the lane in
        the Apollo map.
        Args:
            lane: the lane where the coordinate point is located
            carla_map: carla.Map
            carla_s_coordinate: the s coordinate computed by carla.Map.get_waypoint()

        """
        apollo_s_coordinate = carla_s_coordinate

        s_coordinate_reversed = False

        previous_s = previous_road_id = 0
        for i in range(len(lane.central_curve.segment)):
            for point in lane.central_curve.segment[i].line_segment.point:
                carla_waypoint = carla_map.get_waypoint(
                    Location(point.x, -point.y, point.z)
                )
                carla_s = carla_waypoint.s
                carla_road_id = carla_waypoint.road_id
                if previous_s == 0 or previous_road_id != carla_road_id:
                    previous_s = carla_s
                    previous_road_id = carla_road_id
                    continue
                if carla_s < previous_s:
                    s_coordinate_reversed = True
                break

        if s_coordinate_reversed:
            apollo_s_coordinate = lane.length - carla_s_coordinate
        return apollo_s_coordinate

    def get_lane_by_id(self, lane_id):
        for lane in self.map_pb.lane:
            if str(lane_id) == str(lane.id.id):
                return lane
        return None
