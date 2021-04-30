#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math, sys, pygame
from pygame.locals import *

active_ = False
pub_ = None

def keyboard_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

def main():
    global pub_, active_

    rospy.init_node('keyboard')

    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    srv = rospy.Service('keyboard_switch', SetBool, keyboard_switch)

    pygame.init()
    size = width, height = 512, 324
    screen = pygame.display.set_mode(size)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if not active_:
            rate.sleep()
            continue

        msg = Twist()
        key=pygame.key.get_pressed()
        print(key)
        msg.linear.x=0.0
        msg.angular.z=0.0
        if key[K_UP]:
            msg.linear.x = 5.0
        if key[K_LEFT]:
            msg.angular.z = -5.0
        if key[K_RIGHT]:
            msg.angular.z = 5.0
        if key[K_DOWN]:
            msg.linear.x = -5.0
        if key[K_SPACE]:
            msg.linear.x=0.0
            msg.angular.z=0.0
        print(key[K_UP])
        pub_.publish(msg)
        pygame.event.pump()
        rate.sleep()


if __name__ == '__main__':
    main()

