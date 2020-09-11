#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import math
import numpy as np
import matplotlib.pyplot as plt

count = 0
ax = None
bars = None

def vfhCallback(data):
    # no of points = 720
    global ax, bars
    p_occu = 0.5
    p_empty = 0.5
    angles = np.arange(-90,90,0.25)
    ax.cla()

    distances = np.array(data.ranges)
    detected_angles = np.abs(angles)
    detected_angles[distances == np.inf] = 90
    distances[distances == np.inf] = data.range_max

    p_s_occu = ((data.range_max - distances)/data.range_max + (90 - detected_angles)/90) / 2

    p_occu_s = (p_occu * p_s_occu) / (p_occu * p_s_occu + p_empty * (1-p_s_occu))
    p_empty_s = 1 - p_occu_s
    rospy.loginfo(p_occu_s)
    bars = ax.bar(-angles, p_occu_s)
    plt.draw()

def clbk(data):
    global count

    count += 1

    if(count == 30):
        vfhCallback(data)
        count = 0

def main():
    global ax, bars

    ax = plt.gca()
    bars = ax.plot(-np.arange(-90,90,0.25), np.zeros(720))

    rospy.init_node("VFH")
    sub = rospy.Subscriber('/i214/laser/scan', LaserScan,  clbk)

    plt.show()

    rospy.spin()

if __name__ == '__main__':
    main()