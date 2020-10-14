#! /usr/bin/env python

import sys,select
import rospy
import time
import math
import numpy as np
from geometry_msgs.msg import Point
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from scipy.stats import uniform
from scipy.stats import levy
from sensor_msgs.msg import Imu
from tf import transformations
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist, Point
from std_srvs.srv import *

pi=3.14159265
global detected
detected=False

def clbk_darknet(msg):
	global detected
	detected=True
	print("In callback rn")   #If this function is called, object has been detected

rospy.init_node('spiral')
rospy.wait_for_service('/gazebo/set_model_state')
srv_client_set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
model_state = ModelState()
model_state.model_name = 'i214'
model_state.pose.position.x = 0
model_state.pose.position.y = 0
resp = srv_client_set_model_state(model_state)

rate = rospy.Rate(10)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
sub_darknet = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, clbk_darknet)
v=0.8
w=2
twist_msg = Twist()

while not rospy.is_shutdown():
	twist_msg.linear.x=v
	twist_msg.angular.z=w
	pub.publish(twist_msg)
	if detected==True:
		v=0
		w=0
		twist_msg.linear.x=v
		twist_msg.angular.z=w
		pub.publish(twist_msg)
		sys.exit()
	time.sleep(pi/(2*w))
	w=w*0.95
