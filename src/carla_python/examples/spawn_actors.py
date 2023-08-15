from __future__ import print_function
import sys
import glob
import os
import json
import random
import time 

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla


def create_transform_from_json(json_data):
    # get location and rotation
    location_data = json_data['location']
    rotation_data = json_data['rotation']

    location = carla.Location(x=location_data[0], y=location_data[1], z=location_data[2])
    rotation = carla.Rotation(pitch=rotation_data[0], yaw=rotation_data[1], roll=rotation_data[2])

    return carla.Transform(location, rotation)

def create_dist_point(loc):
    loc.z += 1
    return loc

def main():
    client = carla.Client("172.17.0.1", 2000)
    client.set_timeout(2.0)

    world = client.get_world()

    # load json
    with open('actor_settings.json', 'r') as f:
        data = json.load(f)
    batch = []
    actor_vehicles = []
    actor_walkers = []
    controllers = []
    # get vehicles
    for vehicle in data['vehicles']:
        vehicle_type = world.get_blueprint_library().find(vehicle['type'].encode('utf-8'))
        transform = create_transform_from_json(vehicle)
        mode = vehicle['mode']
        try:
            vehicle = world.spawn_actor(vehicle_type, transform)
            actor_vehicles.append(vehicle)
            if vehicle != None and mode == 'autopilot':
                batch.append(carla.command.SetAutopilot(vehicle, True))
        except RuntimeError as e:
            if "collision at spawn position" in str(e):
                print("Failed to spawn vehicle at %s. Skipping...\n" % transform)
                continue
            else:
                raise e
        # print("Vehicle Type:", vehicle_type)
        # print("Rotation:", rotation)
        # print("Location:", location)
        # print("Mode:", mode)
        # print("-------")

    # get walkers
    for walker in data['walkers']:
        walker_type = world.get_blueprint_library().find(walker['type'].encode('utf-8'))
        # transform = create_transform_from_json(walker)
        transform = carla.Transform()
        sp_loc = create_dist_point(world.get_random_location_from_navigation())
        transform.location = sp_loc
        print("sp =", transform)
        mode = walker['mode']
        try:
            walker = world.spawn_actor(walker_type, transform)
            actor_walkers.append(walker)
            if walker != None and mode == 'autopilot':
                map = world.get_map()
                actor_dp = random.choice(map.get_spawn_points())
                blueprint_library = world.get_blueprint_library()
                walker_controller_bp = blueprint_library.find('controller.ai.walker')

                controller_walker = world.spawn_actor(
                    walker_controller_bp,
                    transform,
                    walker)

                world.tick()  # without this, walker vanishes
                controller_walker.start()
                controller_walker.set_max_speed(5.0)
                controller_walker.go_to_location(actor_dp.location)
                controllers.append(controller_walker)

        except RuntimeError as e:
            if "collision at spawn position" in str(e):
                print("Failed to spawn walker at %s. Skipping...\n" % transform)
                continue
            else:
                raise e
    responses = client.apply_batch_sync(batch)

        # print("Walker Type:", walker_type)
        # print("Rotation:", rotation)
        # print("Location:", location)
        # print("Mode:", mode)
        # print("-------")
    
    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("Program interrupted by user. Exiting...")
        destroy_commands = []
        for v in actor_vehicles:
            if v.id != 197:
                destroy_commands.append(carla.command.DestroyActor(v))
        for w in actor_walkers:
            destroy_commands.append(carla.command.DestroyActor(w))
        for c in controllers:
            c.stop()
            destroy_commands.append(carla.command.DestroyActor(c))
        client.apply_batch(destroy_commands)
        time.sleep(5)
        exit()


if __name__ == '__main__':
    main()

