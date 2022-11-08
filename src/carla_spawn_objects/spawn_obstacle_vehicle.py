import time
import carla
import numpy as np

# Create client and connected to the server
client = carla.Client("172.17.0.1", 2000)
client.set_timeout(30.0)

# get world
world = client.get_world()
# world = client.load_world('Town01')
# weather = carla.WeatherParameters(
#     cloudiness=0.0,
#     precipitation=0.0,
#     sun_altitude_angle=50.0)
# world.set_weather(weather)

# get vehicle blueprint
model3_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
# model3_bp.set_attribute('color', '255,255,255')

camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')

# spawn vehicle(Actor)
spawn_point = carla.Transform(carla.Location(x=74.24, y=204.85, z=2), carla.Rotation(pitch=0, yaw=180, roll=0))
vehicle = world.spawn_actor(model3_bp, spawn_point)


# vehicle.set_autopilot(True)


# # spawn camera
# camera = world.spawn_actor(camera_bp,
#                            carla.Transform(carla.Location(x=-5.5, z=2.5), carla.Rotation(pitch=8.0)),
#                            model3,
#                            carla.AttachmentType.SpringArm
#                            )
# camera.listen(lambda image: image.save_to_disk('./output/%06d.png' % image.frame))
#
while True:
    # spectator = world.get_spectator()
    # transform = model3.get_transform()
    # spectator.set_transform(carla.Transform(transform.location + carla.Location(z=20), carla.Rotation(pitch=-90)))
    time.sleep(5)
