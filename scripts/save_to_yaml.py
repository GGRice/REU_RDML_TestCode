#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from glider_kayak_sim.msg import STU, UnderwaterGeoPose, UnderwaterGeoPoint
import time
import yaml

a={"Temperature": 1, "Salinity": 2}

# FOR STU
#data.temperature gives temp
#data.salinity gives salinity
def stuCallback(data):
  global a
  rospy.loginfo("\nTemperature: %s \nSalinity: %s", data.temperature, data.salinity)
  #a =dict(Temperature = data.temperature, Salinity = data.salinity)
  a["Temperature"] = data.temperature
  a["Salinity"] = data.salinity
  
  


def poseCallback(data):
  pass
#  rospy.loginfo("\nLongitude: %s \nLatitude: %s \nDepth %s \nx: %s \ny: %s \nz: %s \nw: %s ", data.position.longitude, data.position.latitude, data.position.depth, data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
#b = dict(Longitude = data.position.longitude, Latitude = data.position.latitude, Depth = data.position.depth, x = data.orientation.x, y = data.orientation.y, z = data.orientation.z, w = data.orientation.w)

def listener():
  rospy.init_node('listener', anonymous=True)
  
  stu_sub = rospy.Subscriber("kayak_0/stu_sensor", STU, stuCallback)
  with open('data.yml', 'w') as outfile:
    yaml.dump(a, outfile, default_flow_style=False)
  pose_sub = rospy.Subscriber("kayak_0/pose", UnderwaterGeoPose, poseCallback)

  rospy.spin()





if __name__ == '__main__':
  try:
    listener()
  except rospy.ROSInterruptException:
    pass