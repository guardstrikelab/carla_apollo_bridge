#!usr/bin/env python


from modules.common_msgs.sensor_msgs.imu_pb2 import Imu

'''
carla imu sensor output

Sensor data attribute	Type	                                Description
frame	                int	                Frame number when the measurement took place.
timestamp	            double	            Simulation time of the measurement in seconds since the beginning of the episode.
transform	            carla.Transform	    Location and rotation in world coordinates of the sensor at the time of the measurement.
accelerometer	        carla.Vector3D	    Measures linear acceleration in m/s^2.
gyroscope	            carla.Vector3D      Measures angular velocity in rad/sec.
compass	                float	            Orientation in radians. North is (0.0, -1.0, 0.0) in UE.

'''


class IMU:
    def __init__(self, actor, ego_vehicle, node):
        self.actor = actor
        self.node = node
        self.ego_vehicle = ego_vehicle

        self.imu_writer = self.node.create_writer("/apollo/sensor/gnss/imu", Imu, qos_depth=20)

    def update(self):
        imu_msg = Imu()
        imu_msg.header.timestamp_sec = self.node.get_time()
        # imu_msg.header.frame_id = "ego_vehicle/imu"

        # delta_time = datetime.datetime.fromtimestamp(self.node.get_time()) - datetime.datetime(1980, 1, 6)
        # imu_msg.measurement_time = self.node.get_time()

        # Carla uses a left-handed coordinate convention (X forward, Y right, Z up).
        # Here, these measurements are converted to the right-handed ROS convention
        #  (X forward, Y left, Z up).
        carla_imu_measurement = self.actor.get_sensor_data()

        imu_msg.linear_acceleration.x = carla_imu_measurement.linear_acceleration.x
        imu_msg.linear_acceleration.y = -carla_imu_measurement.linear_acceleration.y
        imu_msg.linear_acceleration.z = carla_imu_measurement.linear_acceleration.z

        imu_msg.angular_velocity.x = carla_imu_measurement.angular_velocity.x
        imu_msg.angular_velocity.y = -carla_imu_measurement.angular_velocity.y
        imu_msg.angular_velocity.z = carla_imu_measurement.angular_velocity.z

        self.imu_writer.write(imu_msg)
