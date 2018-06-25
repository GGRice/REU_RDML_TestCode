#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from glider_kayak_sim.msg import STU, UnderwaterGeoPose, UnderwaterGeoPoint
import time



# FOR STU
#data.temperature gives temp
#data.salinity gives salinity
def stuCallback(data):
  rospy.loginfo("\nTemperature: %s \nSalinity: %s", data.temperature, data.salinity)

def poseCallback(data):
  rospy.loginfo("\nLongitude: %s \nLatitude: %s \nDepth %s \nx: %s \ny: %s \nz: %s \nw: %s ", data.position.longitude, data.position.latitude, data.position.depth, data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)

def listener():
  rospy.init_node('listener', anonymous=True)
  
  stu_sub = rospy.Subscriber("kayak_0/stu_sensor", STU, stuCallback)
  pose_sub = rospy.Subscriber("kayak_0/pose", UnderwaterGeoPose, poseCallback)

  rospy.spin()

  stu_sub = stu_sub.unregister()
  pose_sub=pose_sub.unregister()



if __name__ == '__main__':
  listener()