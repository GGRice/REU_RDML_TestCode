#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from glider_kayak_sim.msg import STU, UnderwaterGeoPose, UnderwaterGeoPoint
import time
import yaml
import socket

class KayakReader(object):
	def __init__(self):
		#subscribes to kayak_0 sensor and position
		stu_sub = rospy.Subscriber("kayak_0/stu_sensor", STU, self.stuCallback)
		pose_sub = rospy.Subscriber("kayak_0/pose", UnderwaterGeoPose, self.poseCallback)
		
		#creates variable to hold kayak_0 values
		self.temperature = None
		self.salinity = None
		self.longitude = None
		self.latitude = None
		self.depth = None
		self.x = None
		self.y = None
		self.z = None
		self.w = None

	#sets the sensor variables equal to subscribes values
	def stuCallback(self, data):
		self.temperature = data.temperature
		self.salinity = data.salinity

	#sets position variables equal to subscribed values
	def poseCallback(self, data):
		self.longitude = data.position.longitude
		self.latitude = data.position.latitude
		self.depth = data.position.depth
		self.x = data.orientation.x
		self.y = data.orientation.y
		self.z = data.orientation.z
		self.w = data.orientation.w

	def sendUDP(self):
		#sets up the UDP IP and port to send message over
		UDP_IP = "127.0.0.1"
		UDP_PORT = 5005

		#message to be sent, holds all variables about kayak_0
		kayak_data = "Temperature = {} \nSalinity = {} \nLongitude = {} \nLatitude = {} \nDepth = {} \nx = {} \ny = {} \nz = {} \nw = {}".format(self.temperature, self.salinity, self.longitude, self.latitude, self.depth, self.x, self.y, self.z, self.w)
		
		#sends variables by UDP to reciever
		sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
		sock.sendto(kayak_data, (UDP_IP, UDP_PORT))

def main():
	rospy.init_node("kayak_to_yaml")
	reader = KayakReader()

	rate = rospy.Rate(.1)

	#while ros is running, constantly send updated message with values
	while not rospy.is_shutdown():
		if reader.temperature is not None and reader.longitude is not None:
			reader.sendUDP()

	rate.sleep()



if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass