#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import *
from motion_plan.msg import BoundingBoxes  #Copy the msg folder from darknet_ros to use it

import math


# publishers
pub = None

#camera properties
camera_width = 640
camera_angle = math.pi/2

xmid = 0
laserData = LaserScan()

#PID control paramters (integral term is not added yet)
P = 2	    # Proportional
D = 30	    # Derivative
deltaT = 10 # Time interval for derivative calculation
prev_err_yaw = 0 #previous error for derivative term
 

# callbacks
def clbk_laser(msg):
    global laserData
    laserData = msg

def clbk_camera(msg):
    global xmid
    # To Do: check which object to detect
    xmid_arr = [(box.xmin+box.xmax)/2 for box in msg.bounding_boxes if box.Class == "sports ball"]
    if(len(xmid_arr) != 0):
        xmid = xmid_arr[0]
    print 'xmid: [%s]' % xmid

def go_straight_ahead():
    global pub, prev_err_yaw, xmid, camera_width, laserData, camera_angle, deltaT, D, P
    err_yaw = (xmid - camera_width/2) * camera_angle / camera_width

    if all(t > 0.5 for t in laserData.ranges): # Stop if an object is detected within 0.5
        twist_msg = Twist()
        twist_msg.linear.x = 0.6
       
        derivative = (err_yaw - prev_err_yaw)*D/deltaT
        twist_msg.angular.z = (err_yaw)*P + derivative
        prev_err_yaw = err_yaw
        pub.publish(twist_msg)
    else:
        done()

def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)

def main():
    global pub

    rospy.init_node('navigation')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub_camera = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, clbk_camera)
    sub_laser = rospy.Subscriber('/i214/laser/scan', LaserScan,  clbk_laser)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        go_straight_ahead()
        rate.sleep()

if __name__ == '__main__':
    main()