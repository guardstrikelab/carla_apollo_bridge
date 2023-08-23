from modules.data.proto.frame_pb2 import Vector3


class Twist:
	def __init__(self):
		self.linear = Vector3()
		self.angular = Vector3()


class Accel:
	def __init__(self):
		self.linear = Vector3()
		self.angular = Vector3()
		