# Used to set positon of the ego_vehicle
from __future__ import print_function
import sys
import glob
import os

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import argparse

def parse_arguments():
    parser = argparse.ArgumentParser(description='Process some floats.')

    parser.add_argument('-x', type=float, required=True, help='Input for x')
    parser.add_argument('-y', type=float, required=True, help='Input for y')
    parser.add_argument('-r', type=float, required=True, help='Input for r')

    args = parser.parse_args()
    
    return args.x, args.y, args.r

def main():
    x, y, r = parse_arguments()
    client = carla.Client("172.17.0.1", 2000)
    client.set_timeout(2.0)
    world = client.get_world()

    # get ego_vehicle through actor id  
    ego_vehicle_list = world.get_actors([197])
    if len(ego_vehicle_list) > 0:
        ego_vehicle = ego_vehicle_list[0]
    else:
        print("Couldn't get ego_vehicle, please make sure you have spawn it or check it's actor id.")
    # create transform
    transform = carla.Transform()
    transform.location.x = x
    transform.location.y = -y
    transform.location.z = 2.0
    transform.rotation.yaw = r

    # set transform
    try:
        ego_vehicle.set_transform(transform)
    except RuntimeError as e:
        print("Failed to set vehicle transform due to: %s" % e)
        

if __name__ == '__main__':
    main()