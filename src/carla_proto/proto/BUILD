load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "carla_proto_lib",
    srcs = [
        "carla_actor.proto",
        "carla_basic_type.proto",
        "carla_bounding_box.proto",
        "carla_camera_info.proto",
        "carla_clock.proto",
        "carla_control.proto",
        "carla_destroy_object.proto",
        "carla_ego_vehicle.proto",
        "carla_geometry.proto",
        "carla_get_blueprints.proto",
        "carla_marker.proto",
        "carla_object.proto",
        "carla_odometry.proto",
        "carla_spawn_object.proto",
        "carla_status.proto",
        "carla_traffic_light.proto",
        "carla_walker_control.proto",
        "carla_weather_parameters.proto",
        "carla_world_info.proto",
    ],
    deps = [
        "//modules/common/proto:header_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/data/proto:tools_frame_proto_lib",
        "//modules/localization/proto:pose_proto_lib",
    ],
)
