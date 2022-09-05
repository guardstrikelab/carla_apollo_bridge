import os
import time

import redis
import datetime

from modules.control.proto.control_cmd_pb2 import ControlCommand
from modules.localization.proto.localization_pb2 import LocalizationEstimate
from cyber_py import cyber, cyber_time, cyber_timer, parameter

r = redis.Redis(host="172.16.19.58", port=6379, db=1, password="123456", decode_responses=True)
sequence_num = {"sequence_num": 0}

localization = {"pose":
                {
                        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "orientation": {"qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 0.0},
                        "linear_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "linear_acceleration": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "angular_velocity_vrf": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "linear_acceleration_vrf": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "heading": 0.0
                }
                }

control = {
    "throttle": 0,
    "brake": 0,
    "steer": 0
}

def callback(data):
    """
    Reader message callback.
    """
    print("==="*50)
    # print("data.pose.position.x is {}".format(data.pose.position.x))
    # print("data.pose.position.y is {}".format(data.pose.position.y))
    # print("data.pose.position.z is {}".format(data.pose.position.z))
    #
    localization["pose"]["position"]["x"] = data.pose.position.x
    localization["pose"]["position"]["y"] = data.pose.position.y
    localization["pose"]["position"]["z"] = data.pose.position.z

    localization["pose"]["linear_velocity"]["x"] = data.pose.linear_velocity.x
    localization["pose"]["linear_velocity"]["y"] = data.pose.linear_velocity.y
    localization["pose"]["linear_velocity"]["z"] = data.pose.linear_velocity.z

    localization["pose"]["linear_acceleration"]["x"] = data.pose.linear_acceleration.x
    localization["pose"]["linear_acceleration"]["y"] = data.pose.linear_acceleration.y
    localization["pose"]["linear_acceleration"]["z"] = data.pose.linear_acceleration.z

    localization["pose"]["angular_velocity"]["x"] = data.pose.angular_velocity.x
    localization["pose"]["angular_velocity"]["y"] = data.pose.angular_velocity.y
    localization["pose"]["angular_velocity"]["z"] = data.pose.angular_velocity.z

    localization["pose"]["angular_velocity_vrf"]["x"] = data.pose.angular_velocity_vrf.x
    localization["pose"]["angular_velocity_vrf"]["y"] = data.pose.angular_velocity_vrf.y
    localization["pose"]["angular_velocity_vrf"]["z"] = data.pose.angular_velocity_vrf.z

    localization["pose"]["linear_acceleration_vrf"]["x"] = data.pose.linear_acceleration_vrf.x
    localization["pose"]["linear_acceleration_vrf"]["y"] = data.pose.linear_acceleration_vrf.y
    localization["pose"]["linear_acceleration_vrf"]["z"] = data.pose.linear_acceleration_vrf.z

    localization["pose"]["heading"] = data.pose.heading

    sequence_num["sequence_num"] = data.header.sequence_num
    queue_name = "apollo_pose"
    queue_content = str(localization)
    if sequence_num["sequence_num"] % 5 == 0:
        r.lpush(queue_name, queue_content)
        print('push queue msg {}, sequence_num is {}, time is {}'.format(localization, sequence_num, datetime.datetime.now()))

    # sequence_num["sequence_num"] = data.header.sequence_num
    # control["throttle"] = data.throttle / 100.0
    # control["brake"] = data.brake / 100.0
    # control["steer"] = data.steering_target / 100.0

    # queue_name = "apollo_control"
    # queue_content = str(control)
    # if sequence_num["sequence_num"] % 5 == 0:
    #     r.lpush(queue_name, queue_content)
    #     print('push queue msg {}, sequence_num is {}, time is {}'.format(control, sequence_num, datetime.datetime.now()))
    # with open("./control.txt", "a+") as f:
    #     f.write(queue_content+"\n")



def test_listener_class():
    """
    Reader message.
    """
    print("=" * 120)
    test_node = cyber.Node("pose_to_redis")
    # data_type = "apollo.localization.LocalizationEstimate"
    test_node.create_reader("/apollo/localization/pose", LocalizationEstimate, callback)
    # test_node.create_reader("/apollo/control", ControlCommand, callback)
    test_node.spin()
    # while True:
    #     time.sleep(0.002)
    print("*" * 120)


def control_to_redis(data):
    sequence_num["sequence_num"] = data.header.sequence_num
    control["throttle"] = data.throttle
    control["brake"] = data.brake
    control["steer"] = data.steering_target

    queue_name = "apollo_control"
    queue_content = str(control)
    r.lpush(queue_name, queue_content)
    print('push queue msg {}, sequence_num is {}, time is {}'.format(control, sequence_num, datetime.datetime.now()))
    with open("./control.txt", "a+") as f:
        f.write(queue_content+"\n")


def send_control():
    print("=" * 120)
    file_name = "./control.txt"
    if os.path.exists(file_name):
        os.remove(file_name)
    control_node = cyber.Node("redis_control")
    control_node.create_reader("/apollo/control", ControlCommand, control_to_redis)
    control_node.spin()


if __name__ == '__main__':
    cyber.init()
    send_control()
    cyber.shutdown()

