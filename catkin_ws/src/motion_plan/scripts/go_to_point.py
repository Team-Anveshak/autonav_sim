#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from tf import transformations
from std_srvs.srv import *

import math
import numpy as np
import csv

class gpsData:
    latitude = 0.0
    longitude = 0.0

active_ = False

current_goal = 1
position_data = None

# robot state variables
position_ = gpsData()
yaw_ = 100

# machine state
state_ = 1
# goal
desired_position_ = gpsData()
# parameters
yaw_precision_ = math.pi*2 / 180 # +/- 10 degree allowed
dist_precision_ = 0.00014




# publishers
pub = None

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
def clbk_gps(msg):
    global position_

    position_.latitude = msg.latitude
    position_.longitude = msg.longitude

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
    global state_, prev_err_yaw
    # so the derivative of the previous iteration isn't included
    prev_err_yaw = 0
    state_ = state
    print 'State changed to [%s]' % state_

def yawFromGps(lat1, long1, lat2, long2):
    lat1,long1,lat2,long2 = map(np.radians, [lat1,long1,lat2,long2])

    dLon = (long2 - long1)

    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)

    brng = math.atan2(y, x)
    brng = (brng + 2*math.pi) % math.pi
    brng = math.pi - brng

    return brng

def distanceFromGps(lat1,lon1,lat2,lon2):
    R = 6371.0088
    lat1,lon1,lat2,lon2 = map(np.radians, [lat1,lon1,lat2,lon2])

    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2) **2
    c = 2 * np.arctan2(a**0.5, (1-a)**0.5)
    d = R * c
    return d

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = yawFromGps(position_.latitude, position_.longitude, des_pos.latitude, des_pos.longitude)
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
    global yaw_, pub, yaw_precision_, state_, prev_err_yaw, position_
    desired_yaw = yawFromGps(position_.latitude, position_.longitude, des_pos.latitude, des_pos.longitude)
    err_yaw = desired_yaw - yaw_
    err_pos = distanceFromGps(position_.latitude, position_.longitude, des_pos.latitude, des_pos.longitude)

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
        print 'Yaw: [%s]' % desired_yaw
  #     change_state(0)   Removed temporarily because it was causing problems (the rover stops abruptly in the middle in presence of an obstacle)

def set_next_goal():
    global current_goal, desired_position_, position_data

    #Check if there are any more goals left
    if current_goal < len(position_data) - 1:
        print 'Goal [%s] reached' % current_goal
        current_goal += 1
        desired_position_.latitude = position_data[current_goal][0]
        desired_position_.longitude = position_data[current_goal][1]
        change_state(0)
    elif current_goal == len(position_data) - 1:
        print 'Final Goal Reached'
        current_goal += 1  
    else:
        done()

def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)

def main():
    global pub, active_, position_data, desired_position_
  
    #read position_data rom file
    position_data = np.genfromtxt("positionData.csv", delimiter=',')
    desired_position_.latitude = position_data[1][0]
    desired_position_.longitude = position_data[1][1]

    rospy.init_node('go_to_point')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub_gps = rospy.Subscriber('/fix', NavSatFix, clbk_gps)

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
                set_next_goal()
            else:
                rospy.logerr('Unknown state!')

        rate.sleep()

if __name__ == '__main__':
    main()
