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
from std_msgs.msg import Float64
from std_srvs.srv import *

pi=3.14159265
global detected
detected=0
active_ = False

def clbk_darknet(msg):
	global detected
	detected=detected+1
	print("In callback rn")   #If this function is called, object has been detected

def spiral_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

rospy.init_node('spiral')
rospy.wait_for_service('/gazebo/set_model_state')
srv_client_set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

rospy.wait_for_service('/navigation_switch')
srv_client_navigation_ = rospy.ServiceProxy('/navigation_switch', SetBool)

srv = rospy.Service('spiral_switch', SetBool, spiral_switch)
model_state = ModelState()
model_state.model_name = 'i214'
model_state.pose.position.x = 0
model_state.pose.position.y = 0
resp = srv_client_set_model_state(model_state)

rate = rospy.Rate(10)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
pub_cam=rospy.Publisher('/i214/camera_joint_position_controller/command', Float64, queue_size=1)
sub_darknet = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, clbk_darknet)
v=0.8
w=2
twist_msg = Twist()

while not rospy.is_shutdown():
	if not active_:
		continue
	twist_msg.linear.x=v
	twist_msg.angular.z=w
	pub.publish(twist_msg)
	if detected==1:
		v=0
		w=0.2
		twist_msg.linear.x=v
		twist_msg.angular.z=w
		pub.publish(twist_msg)
		pub_cam.publish(v)
		#sys.exit()
	if detected==2:
		v=0
		w=0
		twist_msg.linear.x=v
		twist_msg.angular.z=w
		pub.publish(twist_msg)
		resp = srv_client_navigation_(True)
		#active_ = False
		sys.exit()
	time.sleep(pi/(2*w))
	w=w*0.95
