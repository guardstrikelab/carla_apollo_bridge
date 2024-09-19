#!/usr/bin/env python
#
# SYNKROTRON Confidential
# Copyright (C) 2023 SYNKROTRON Inc. All rights reserved.
# The source code for this program is not published
# and protected by copyright controlled
#

from carla import Location
from cyber.python.cyber_py3 import cyber_time
from modules.common_msgs.routing_msgs import routing_pb2

from carla_api.global_route_planner import GlobalRoutePlanner


class Routing(object):
    """
    Plan the global path, generate a RoutingResponse, and send it to Apollo's Planning module.
    First, use GlobalRoutePlanner in carla to generate a path composed of waypoints,
    then call libhdmap.so library to find the nearest lane of each waypoint,
    and finally use these lanes to generate RoutingResponse.
    """

    def __init__(self, carla_map, map_util):
        # Setup global planner.
        self.map = carla_map

        self._grp = GlobalRoutePlanner(carla_map, 1.0)  # Distance between waypoints

        self.map_util = map_util

    def generate_routing_request(
        self, start_location: Location, target_location: Location, routing_request=None
    ):
        """
        Generate RoutingRequest based on starting and target location points.

        Due to the road before and after map conversion, both the id and s coordinates will change, causing them to not
        correspond and requiring conversion based on the x and y coordinates.
        The following is an explanation of the changes in the s coordinate.
        In a lane, the s coordinate of the starting position is 0, and the s coordinate of the ending position is the
        length of the lane.

        Oasis (carla) map: In Oasis, the starting point of the lane is independent of the direction of the lane.
        All lanes on the same road start on the same side, and equals to the road start position.

        Apollo map (converted by road_runner): In Apollo, the starting point of a lane is related to the direction of
        the lane, and the starting point of the lane is located at the starting point of the lane's direction of travel,
        rather than the starting point of the road.
        """
        now_cyber_time = cyber_time.Time.now().to_sec()

        if not routing_request:
            routing_request = routing_pb2.RoutingRequest()
        routing_request.header.timestamp_sec = now_cyber_time
        routing_request.header.module_name = "oasis_bridge"

        self._set_imap_waypoint(start_location, target_location, routing_request)
        # self._set_old_map_waypoint(start_location, target_location, routing_request)
        # self._set_road_runner_waypoint(start_location, target_location, routing_request)

        return routing_request

    def generate_routing_response(
        self, start_location: Location, target_location: Location
    ):
        """
        Computes RoadSegments between two locations.
          Assumes that the ego vehicle has the same orientation as the lane on
          which it is on.

          Returns:
            modules.common_msgs.routing_msgs.routing_pb2.RoutingResponse
        """
        routing_response = routing_pb2.RoutingResponse()
        self.generate_routing_request(
            start_location, target_location, routing_response.routing_request
        )
        route = self._grp.trace_route(start_location, target_location)
        for route_element in route:
            waypoint = route_element[0]
            road_id = waypoint.road_id
            section_id = waypoint.section_id
            lane_id = f"{road_id}_{section_id + 1}_{waypoint.lane_id}"
            # lane_change = waypoint.lane_change
            # is_junction = waypoint.is_junction
            if (
                len(routing_response.road) > 0
                and str(road_id) == routing_response.road[-1].id
            ):
                last_lane = routing_response.road[-1].passage[-1].segment[-1]
                if last_lane.id == lane_id:
                    last_lane.end_s = float(waypoint.s)
            # TODO Determine whether to change lanes
            else:
                road = routing_response.road.add()
                passage = road.passage.add()
                passage.can_exit = True
                lane = passage.segment.add()
                road.id = str(road_id)
                lane.id = lane_id
                lane.start_s = float(waypoint.s)
                lane.end_s = float(waypoint.s)

        now_cyber_time = cyber_time.Time.now().to_sec()

        routing_response.header.timestamp_sec = now_cyber_time
        routing_response.header.module_name = "oasis_bridge"
        return routing_response

    def _set_road_runner_waypoint(
        self, start_location, target_location, routing_request
    ):
        (
            start_carla_waypoint,
            target_carla_waypoint,
            start_apollo_waypoint,
            target_apollo_waypoint,
        ) = self._generate_waypoint(start_location, target_location, routing_request)

        start_lane = self.map_util.get_lane_of_point(start_location)
        target_lane = self.map_util.get_lane_of_point(target_location)

        start_apollo_waypoint.id = start_lane.id.id
        target_apollo_waypoint.id = target_lane.id.id

        start_apollo_waypoint.s = self.map_util.get_apollo_s_coordinate(
            start_lane, self.map, start_carla_waypoint.s
        )
        target_apollo_waypoint.s = self.map_util.get_apollo_s_coordinate(
            target_lane, self.map, target_carla_waypoint.s
        )

    def _set_old_map_waypoint(self, start_location, target_location, routing_request):
        (
            start_carla_waypoint,
            target_carla_waypoint,
            start_apollo_waypoint,
            target_apollo_waypoint,
        ) = self._generate_waypoint(start_location, target_location, routing_request)

        start_apollo_waypoint.id = (
            f"{start_carla_waypoint.road_id}_"
            f"{start_carla_waypoint.section_id + 1}_"
            f"{start_carla_waypoint.lane_id}"
        )
        start_apollo_waypoint.s = self.map_util.get_apollo_s_coordinate(
            self.map_util.get_lane_by_id(start_apollo_waypoint.id),
            self.map,
            start_carla_waypoint.s,
        )

        target_apollo_waypoint.id = (
            f"{target_carla_waypoint.road_id}_"
            f"{target_carla_waypoint.section_id + 1}_"
            f"{target_carla_waypoint.lane_id}"
        )
        target_apollo_waypoint.s = self.map_util.get_apollo_s_coordinate(
            self.map_util.get_lane_by_id(target_apollo_waypoint.id),
            self.map,
            target_carla_waypoint.s,
        )

    def _set_imap_waypoint(self, start_location, target_location, routing_request):
        (
            start_carla_waypoint,
            target_carla_waypoint,
            start_apollo_waypoint,
            target_apollo_waypoint,
        ) = self._generate_waypoint(start_location, target_location, routing_request)

        start_apollo_waypoint.id = (
            f"road_{start_carla_waypoint.road_id}_"
            f"lane_{start_carla_waypoint.section_id}_"
            f"{start_carla_waypoint.lane_id}"
        )
        start_apollo_waypoint.s = self.map_util.get_apollo_s_coordinate(
            self.map_util.get_lane_by_id(start_apollo_waypoint.id),
            self.map,
            start_carla_waypoint.s,
        )

        target_apollo_waypoint.id = (
            f"road_{target_carla_waypoint.road_id}_"
            f"lane_{target_carla_waypoint.section_id}_"
            f"{target_carla_waypoint.lane_id}"
        )
        target_apollo_waypoint.s = self.map_util.get_apollo_s_coordinate(
            self.map_util.get_lane_by_id(target_apollo_waypoint.id),
            self.map,
            target_carla_waypoint.s,
        )

    def _generate_waypoint(self, start_location, target_location, routing_request):
        start_carla_waypoint = self.map.get_waypoint(location=start_location)
        target_carla_waypoint = self.map.get_waypoint(location=target_location)

        start_location.x = start_location.x
        start_location.y = start_location.y * -1
        target_location.x = target_location.x
        target_location.y = target_location.y * -1

        start_apollo_waypoint = routing_request.waypoint.add()
        start_apollo_waypoint.pose.x = start_location.x
        start_apollo_waypoint.pose.y = start_location.y

        target_apollo_waypoint = routing_request.waypoint.add()
        target_apollo_waypoint.pose.x = target_location.x
        target_apollo_waypoint.pose.y = target_location.y

        return (
            start_carla_waypoint,
            target_carla_waypoint,
            start_apollo_waypoint,
            target_apollo_waypoint,
        )
