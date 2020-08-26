#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf import transformations
from std_srvs.srv import *

import math
import numpy as np
import csv

active_ = False

# robot state variables
position_ = Point()
yaw_ = 100

# machine state
state_ = 1
# goal
desired_position_ = Point()
desired_position_.x = 10
desired_position_.y = 10
desired_position_.z = 0
# parameters
yaw_precision_ = math.pi*2 / 180 # +/- 10 degree allowed
dist_precision_ = 0.3




# publishers
pub = None
pub_next_position = None

#PID control paramters (integral term is not added yet)
P = -2	    # Proportional
D = -30	    # Derivative
deltaT = 10 # Time interval for derivative calculation
prev_err_yaw = 0 #previous error for derivative term in control algorithm
 

# service callbacks
def go_to_point_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

# callbacks
def clbk_odom(msg):
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

def clk_yaw(msg):
    global yaw_
    quaternion = (
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]
    #rospy.loginfo(state_v[2])

def change_state(state):
    global state_
    state_ = state
    print 'State changed to [%s]' % state_

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)

    rospy.loginfo(err_yaw)

    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        derivative = (err_yaw - prev_err_yaw)*D/deltaT
        twist_msg.angular.z = (err_yaw)*P + derivative


    pub.publish(twist_msg)

    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        print 'Yaw error: [%s]' % err_yaw
        change_state(1)

def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_, prev_err_yaw
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.6
       
        derivative = (err_yaw - prev_err_yaw)*D/deltaT
        twist_msg.angular.z = (err_yaw)*P + derivative
        prev_err_yaw = err_yaw
        pub.publish(twist_msg)
    else:
        print 'Position error: [%s]' % err_pos
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print 'Yaw error: [%s]' % err_yaw
  #     change_state(0)   Removed temporarily because it was causing problems (the rover stops abruptly in the middle in presence of an obstacle)

def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)

def main():
    global pub, active_
  
    #read position_data rom file
    position_data = np.genfromtxt("positionData.csv", delimiter=',')
    desired_position_.x = position_data[1][0]
    desired_position_.y = position_data[1][1]
    desired_position_.z = position_data[1][2]

    rospy.init_node('go_to_point')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    pub_next_position = rospy.Publisher('/next_desired_position', Point, queue_size=2)

    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

    sub_imu = rospy.Subscriber('/imu', Imu, clk_yaw)

    srv = rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if not active_:
            continue
        else:
            if state_ == 0:
                fix_yaw(desired_position_)
            elif state_ == 1:
                go_straight_ahead(desired_position_)
            elif state_ == 2:
                #Check if there are any more goals left
                if current_goal < len(position_data) - 1:
                    print 'Goal [%s] reached' % current_goal
                    current_goal += 1
                    desired_position_.x = position_data[current_goal][0]
                    desired_position_.y = position_data[current_goal][1]
                    desired_position_.z = position_data[current_goal][2]
                    change_state(0)
                    pub_next_position.publish(desired_position_)
                elif current_goal == len(position_data) - 1:
                    print 'Final Goal Reached'
                    current_goal += 1  
                else:
                    done()
            else:
                rospy.logerr('Unknown state!')

        rate.sleep()

if __name__ == '__main__':
    main()
