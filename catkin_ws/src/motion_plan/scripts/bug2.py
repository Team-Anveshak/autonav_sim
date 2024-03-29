#! /usr/bin/env python

import sys,select
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf import transformations
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import *

import math

srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
srv_client_keyboard_ = None
yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180) # 5 degrees
position_ = Point()
initial_position_ = Point()
initial_position_.x = -10
initial_position_.y = -10
initial_position_.z = 0
desired_position_ = Point()
#desired_position_.x = 10
#desired_position_.y = 10
#desired_position_.z = 0
regions_ = None
state_desc_ = ['Go to point', 'wall following', 'Keyboard input']
state_ = 0
count_state_time_ = 0 # seconds the robot is in a state
count_loop_ = 0
# 0 - go to point
# 1 - wall following

# callbacks
def clbk_odom(msg):
	global position_
	position_ = msg.pose.pose.position


def clk_yaw(msg):
	global yaw_
	quaternion = (msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)
	euler = transformations.euler_from_quaternion(quaternion)
	yaw_ = euler[2]

def clbk_laser(msg):
	global regions_
	regions_ = {
		'right':  min(min(msg.ranges[0:143]), 10),
		'fright': min(min(msg.ranges[144:287]), 10),
		'front':  min(min(msg.ranges[288:431]), 10),
		'fleft':  min(min(msg.ranges[432:575]), 10),
		'left':   min(min(msg.ranges[576:719]), 10),
	}


def change_state(state):
	global state_, state_desc_
	global srv_client_wall_follower_, srv_client_go_to_point_, srv_client_keyboard_
	global count_state_time_
	count_state_time_ = 0
	state_ = state
	log = "state changed: %s" % state_desc_[state]
	rospy.loginfo(log)
	if state_ == 0:
		resp = srv_client_go_to_point_(True)
		resp = srv_client_wall_follower_(False)
		resp=srv_client_keyboard_(False)
	if state_ == 1:
		resp = srv_client_go_to_point_(False)
		resp = srv_client_wall_follower_(True)
		resp=srv_client_keyboard_(False)
	if state_ == 2:
		resp=srv_client_go_to_point_(False)
		resp=srv_client_wall_follower_(False)
		resp=srv_client_keyboard_(True)

def normalize_angle(angle):
	if(math.fabs(angle) > math.pi):
		angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
	return angle

def main():

    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_,srv_client_keyboard_
    global count_state_time_, count_loop_
    


    rospy.init_node('bug2')

    sub_laser = rospy.Subscriber('/camera/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    sub_imu = rospy.Subscriber('/imu', Imu, clk_yaw)

    rospy.wait_for_service('/go_to_point_switch')
    rospy.wait_for_service('/wall_follower_switch')
    rospy.wait_for_service('/gazebo/set_model_state')

    srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)
    srv_client_keyboard_=rospy.ServiceProxy('/keyboard_switch',SetBool)
    srv_client_set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    

    # set robot position
    model_state = ModelState()
    model_state.model_name = 'i214'
    model_state.pose.position.x = initial_position_.x
    model_state.pose.position.y = initial_position_.y
    resp = srv_client_set_model_state(model_state)

    # initialize going to the point
    change_state(0)

    rate = rospy.Rate(10)
    switch='0'
    while not rospy.is_shutdown():
    	
    
	i,o,e=select.select([sys.stdin],[],[],1)
	if(i):
		switch=sys.stdin.readline().strip()
	if regions_ == None:
		continue
	if switch=='1' and not state_==2:
		change_state(2)
	elif switch=='0' and state_==2:
		change_state(0)
	if state_ == 0:
		if regions_['front'] > 0.15 and regions_['front'] < 1:
			change_state(1)
	elif state_ == 1:
		if count_state_time_ > 5:
			change_state(0)
				   
	count_loop_ = count_loop_ + 1
	if count_loop_ == 20:
		count_state_time_ = count_state_time_ + 1
		count_loop_ = 0
        
	rospy.loginfo("position: [%.2f, %.2f]", position_.x, position_.y)
	rate.sleep()

if __name__ == "__main__":
	main()
