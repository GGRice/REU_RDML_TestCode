#!/usr/bin/env python


import argparse
import socket
import rospy
import time
import yaml
from TCP import *
from std_msgs.msg import String
from glider_kayak_sim.msg import STU, UnderwaterGeoPose, UnderwaterGeoPoint


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



def main():

	


	# Parsing user input
	parser = argparse.ArgumentParser()
	parser.add_argument(
			'-p','--port',
			nargs='?',
			type=int,
			default=8118,
			help='TCP Port Number.'
		)
	parser.add_argument(
			'-i','--ip',
			nargs='?',
			type=str,
			default='127.0.0.1',
			help='TCP IP Address.'
		)
	parser.add_argument(
			'-b','--buffer_size',
			nargs='?',
			type=int,
			default=256,
			help='TCP Socket Buffer Size.'
		)
	parser.add_argument(
			'-m','--mode',
			nargs='?',
			action='store',
			default='client',
			choices=['client','server'],
			help='Client or Server mode.'
		)
	args = parser.parse_args()

	tcp = TCP(
		port		= args.port,
		ip			= args.ip,
		buffer_size	= args.buffer_size,
		mode		= args.mode,
	)

	
	# Keeps requesting the most recent data
	if(tcp.mode=='client'):
		try:
			for i in range(15):
				print i, tcp.request('kayak_0')
				#print i, tcp.request('glider-02')
		except:
			pass


	# Keeps providing the most recent data
	if(tcp.mode=='server'):	
		#initiates rospy and the reader class
		rospy.init_node("kayak_tcp")
		reader0 = KayakReader()
		#reader1 = KayakReader()

		rate = rospy.Rate(.1)		
		try:
			while not rospy.is_shutdown():
				k0 = dict(Temperature = reader0.temperature, Salinity = reader0.salinity, Latitude = reader0.latitude, Longitude = reader0.longitude, Depth = reader0.depth, x = reader0.x, y = reader0.y, z = reader0.z, w = reader0.w)
				k1 = dict(Temperature = reader1.temperature, Salinity = reader1.salinity, Latitude = reader1.latitude, Longitude = reader1.longitude, Depth = reader1.depth, x = reader1.x, y = reader1.y, z = reader1.z, w = reader1.w)
				
				tcp.database = {
					'kayak_0': k0,
					'kayak_1': k1,
				}
			rate.sleep()
				
		except:
			tcp.exit_server()



if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass





"""
				tcp.database = {
					'kayak_0':{
						'position': {
							'lat': reader0.latitude,
							'lon': reader0.longitude,
							'depth': reader0.depth,
						},
						'heading': {
							'x': reader0.x,
							'y': reader0.y,
							'z': reader0.z,
							'w': reader0.w,
						},
						'stu sensor': {
							'temperature': reader0.temperature,
							'salinity': reader0.salinity,
						},
					},
					'kayak_1': {
						'position': {
							'lat': reader1.latitude,
							'lon': reader1.longitude,
							'depth': reader1.depth,
						},
						'heading': {
							'x': reader1.x,
							'y': reader1.y,
							'z': reader1.z,
							'w': reader1.w,
						},
						'stu sensor': {
							'temperature': reader1.temperature,
							'salinity': reader1.salinity,
						},
					},
				}
				"""