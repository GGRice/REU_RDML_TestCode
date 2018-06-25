#!/usr/bin/env python


import rospy, itertools, random, time, tf, Queue, math, haversine, pdb

import numpy as np
from scipy.spatial import Delaunay
from keck_glider.srv import gpGetWaypoint
from keck_glider.msg import GeoPoseStamped, Heartbeat, GPLocData, ScienceData

from geographic_msgs.msg import GeoPoint
from std_msgs.msg import Bool
from utils import latLonDist, strictlyDecreasing, strictlyIncreasing

from navigators import NullNav, RandomNav, HotspotNav



class navHandler(object):
  """docstring for navHandler"""
  def __init__(self):
    self.glider_lon = None
    self.glider_lat = None
    self.glider_depth = None
    self.glider_pitch = None
    self.glider_status = None


    #Read ROS Params
    self.message_frequency = rospy.get_param("glider/bat_message_period")
    self.gp_data_type = rospy.get_param("glider/gp_data_type")
    self.waypoint_radius = rospy.get_param("glider/waypoint_radius")
    self.replan_period = rospy.get_param("glider/replan_period")
    self.surface_buffer = rospy.get_param("glider/surface_buffer")
    self.science_sensor_type = rospy.get_param("glider/science_sensor_type")

    #Initialize Subscribers + Publishers
    glider_pose_sub = rospy.Subscriber("/glider/geoPose", GeoPoseStamped, self.gliderPoseCallback)
    glider_science_sub = rospy.Subscriber('/glider/science', ScienceData, self.scienceDataCallback)

    self.reset_waypoint_pub = rospy.Publisher('/glider/command/end_nav', Bool, queue_size=10)
    self.waypoint_pub = rospy.Publisher('/glider/command/waypoint', GeoPoint, queue_size=10)
    self.gp_input_pub = rospy.Publisher("/glider/gp_input", GPLocData, queue_size = 10)
    self.heartbeat_pub = rospy.Publisher('/glider/heartbeat/nav', Heartbeat, queue_size=10)


    #Initialize Nav Handler Variables
    self.science_data = []
    self.pitches = []
    self.seq_id = 0
    self.nav_mode = 0
    self.nav_queue = None
    self.last_heartbeat_time = 0.0
    self.last_planning_time = 0.0
    self.current_waypoint = None
    self.n_pitches = 5



  def setMode(self, nav_mode):
    self.nav_mode = nav_mode
    if self.nav_mode == 0:
      rospy.loginfo("Setting Nav Mode: Null")
      self.navigator = NullNav()

    elif self.nav_mode == 1:
      rospy.loginfo("Setting Nav Mode: Random")
      self.navigator = RandomNav()

    elif self.nav_mode == 2:
      rospy.loginfo("Setting Nav Mode: Topological Hotspots")
      self.navigator = HotspotNav()

    else:
      rospy.logwarn("Invalid Nav Mode")
      rospy.loginfo("Setting Nav Mode: Null")
      self.navigator = NullNav()


  def gliderPoseCallback(self, msg_data):
    self.glider_lat = msg_data.geopose.position.latitude
    self.glider_lon = msg_data.geopose.position.longitude
    self.glider_depth = -msg_data.geopose.position.altitude
    q = [msg_data.geopose.orientation.x, msg_data.geopose.orientation.y, msg_data.geopose.orientation.z, msg_data.geopose.orientation.w]
    _, pitch, yaw = tf.transformations.euler_from_quaternion(q)
    self.glider_pitch = pitch


    if self.glider_depth > self.surface_buffer: 

      if len(self.pitches) < self.n_pitches:
        self.pitches.append(self.glider_pitch)
      else:
        self.pitches = self.pitches[1:self.n_pitches-1]
        self.pitches.append(self.glider_pitch)


  def publishGPData(self, data):
    if self.glider_lat is None or self.glider_lon is None or self.glider_depth is None:
      return
    if data is None:
      rospy.logerr("Error: No Data")
      return
    else:
      msg = GPLocData()
      msg.loc.latitude = self.glider_lat
      msg.loc.longitude = self.glider_lon
      msg.loc.altitude = self.glider_depth
      msg.header.stamp = rospy.Time.now()
      msg.header.seq = self.seq_id
      self.seq_id += 1
      msg.header.frame_id = "global_pose"
      msg.data = data 
      self.gp_input_pub.publish(msg)


  def scienceDataCallback(self, msg_data):
    if self.glider_depth > self.surface_buffer:
      if self.science_sensor_type == 0:
        #Temperature Data
        datapoint = msg_data.temperature

      elif self.science_sensor_type == 1:
        # Salinity Data
        datapoint = msg_data.conductivity

      elif self.science_sensor_type == 2:
        # Chlorophyll
        datapoint = msg_data.chlorophyll

      else:
        #Invalid Sensing Mode: Using Temperature
        datapoint = msg_data.temperature

      if not math.isnan(datapoint):
        self.science_data.append([datapoint, self.glider_depth])




  def sendWaypoint(self, waypoint_msg=None):
    if self.nav_mode != 0:
      if self.glider_lat is not None and self.glider_lon is not None and self.glider_depth is not None:
        #waypoint_msg = self.navigator.getWaypoint(self.glider_lat, self.glider_lon)

        if waypoint_msg is not None:
          reset_msg = Bool()
          reset_msg.data = True
          self.reset_waypoint_pub.publish(reset_msg)
          time.sleep(10)

          self.waypoint_pub.publish(waypoint_msg)
          self.current_waypoint = [waypoint_msg.longitude, waypoint_msg.latitude]
          #self.start_time = rospy.get_time() 

  def checkHeartbeat(self): 
    msg = Heartbeat()
    msg.stamp = rospy.Time.now()
    msg.value = float(self.nav_mode)
    self.heartbeat_pub.publish(msg) 
    self.last_heartbeat_time = time.time()


  def getPlan(self):
    budget = 200
    return self.navigator.getPlan(budget, self.glider_lat, self.glider_lon)


  def checkGPPublish(self):
    if self.gp_data_type == 0:
    # Standard Temperature (Used mainly for simulation)
      if len(self.science_data) > 0:
        self.publishGPData(np.mean(self.science_data))
        self.science_data = []

    elif self.gp_data_type == 1:
      # VHTI
      self.gliderYoPublish(vhti)

    elif self.gp_data_type == 2:
      # Mean
      self.gliderYoPublish(depthWindowMean)

    elif self.gp_data_type == 3:
      # Maximum
      self.gliderYoPublish(depthWindowMax)



  def gliderYoPublish(self, scienceFunction=lambda x: np.mean(x[:,0])):
    # Publishes to the GP once per Yo using scienceFunction to collapse 
    # science data to a single data point

    if len(self.pitches) >= self.n_pitches and np.mean(self.pitches) > 0:
      if self.glider_status == "ascending":
        #print len(self.science_data), self.science_data[0]
        self.publishGPData(scienceFunction(self.science_data))
        #print "Beginning Downward Yo"
        self.science_data = []
      self.glider_status = "descending"

    elif len(self.pitches) >= self.n_pitches and np.mean(self.pitches) < 0:
      if self.glider_status == "descending":
        #print len(self.science_data), self.science_data[0]
        self.publishGPData(scienceFunction(self.science_data))
        #print "Beginnign Upward Yo"
        self.science_data = []
      self.glider_status = "ascending"


def vthi(science_data):
  science_data = np.array(science_data)
  res = np.mean(np.abs(temps-np.mean(science_data[:,0])))
  return res

def depthWindowMean(science_data):
  upper_depth = rospy.get_param("glider/sensor_window_upper")
  lower_depth = rospy.get_param("glider/sensor_window_lower")

  science_data = np.array([x for x in science_data if x[1] < lower_depth and x[1] > upper_depth])
  if len(science_data) == 0:
    return None
  else:
    return np.mean(science_data[:,0])

def depthWindowMax(science_data):
  upper_depth = rospy.get_param("glider/sensor_window_upper")
  lower_depth = rospy.get_param("glider/sensor_window_lower")

  science_data = np.array([x for x in science_data if x[1] < lower_depth and x[1] > upper_depth])
  if len(science_data) == 0:
    return None
  else:
    return np.max(science_data[:,0])



def talker():
  rospy.loginfo("Starting Nav Handler")
  rospy.init_node('glider_nav', anonymous=True)
  rate = rospy.Rate(1)
  
  #Initialize Navigation Handler
  nh = navHandler()
  nh.setMode(rospy.get_param("glider/navigation_mode"))


  while nh.glider_lat is None and nh.glider_lon is None:
    pass

  while not rospy.is_shutdown():
    nh.checkHeartbeat()
    nh.checkGPPublish()

    nav_mode = rospy.get_param("glider/navigation_mode")
    if nav_mode != nh.nav_mode:
      nh.setMode(nav_mode)

    if nh.current_waypoint is None:
      rospy.loginfo("No Current Waypoint, Getting New Waypoint")
      try:
        waypoint = nh.nav_queue.get_nowait()
        nh.sendWaypoint(waypoint)

      except Queue.Empty:
        rospy.loginfo("Error: Tried to get waypoint from Empty Queue")
        pass

      except AttributeError:
        rospy.loginfo("No Current Plan")
        pass

    elif latLonDist([nh.glider_lon, nh.glider_lat], nh.current_waypoint) <= nh.waypoint_radius:
      rospy.loginfo("Waypoint Achieved, goto new waypoint")
      try:
        waypoint = nh.nav_queue.get_nowait()
        nh.sendWaypoint(waypoint)

      except Queue.Empty:
        rospy.loginfo("Error: Tried to get waypoint from Empty Queue")
        pass

      except AttributeError:
        rospy.loginfo("No Current Plan")
        pass

    if nh.nav_queue is None or nh.nav_queue.empty():
      rospy.loginfo("No Current Plan, Obtaining New Plan")
      nh.replan_time = time.time()
      nh.nav_queue = nh.getPlan()

    elif time.time()-nh.last_planning_time < nh.replan_period:
      rospy.loginfo("Replaning Window Reached, Obtaining New Plan")
      nh.replan_time = time.time()
      nh.nav_queue = nh.getPlan()

    rate.sleep()


if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass