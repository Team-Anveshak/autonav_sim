#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, NavSatFix
from tf import transformations
from std_srvs.srv import *

import math
import numpy as np
import matplotlib.pyplot as plt


class gpsData:
    latitude = 0.0
    longitude = 0.0

count = 0
srv_client_spiral_ = None
ax = None
bars = None
pub = None
laser_data = LaserScan()

# machine state
state_ = 1
active_ = True

position_ = gpsData()
yaw_ = 100

# goal
desired_position_ = gpsData()
position_data = None
current_goal = 1

dist_precision_ = 0.001

prev_angle = 0
a = 2
b = 0.7
c = 0.7

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

def clk_laser(msg):
    global laser_data
    laser_data = msg


def change_state(state):
    global state_, prev_angle
    prev_angle = 0
    state_ = state
    print 'State changed to [%s]' % state_

def yawFromGps(lat1, long1, lat2, long2):
    lat1,long1,lat2,long2 = map(np.radians, [lat1,long1,lat2,long2])

    dLon = (long2 - long1)

    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)

    brng = math.atan2(y, x)
    brng = (brng + 2*math.pi) % (2*math.pi)
    brng = 2*math.pi - brng

    if brng > math.pi:
        brng -= 2*math.pi

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

def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)

def plotter(angles, p_occu_s):
    #global ax, bars
    #ax.cla()
    #bars = ax.bar(angles, p_occu_s)
    #plt.draw()

    plt.bar(angles,p_occu_s)
    plt.show()
    plt.pause(0.0001)

def set_next_goal():
    global current_goal, desired_position_, position_data, srv_client_spiral_, active_

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
        active_ = False
        resp = srv_client_spiral_(True)
    else:
        done()

def vfh():
    global count, prev_angle, yaw_, pub, prev_angle, position_, dist_precision_, state_, laser_data
    desired_yaw = yawFromGps(position_.latitude, position_.longitude, desired_position_.latitude, desired_position_.longitude)
    err_pos = distanceFromGps(position_.latitude, position_.longitude, desired_position_.latitude, desired_position_.longitude)
    desired_err_yaw = normalize_angle(desired_yaw - yaw_)

    # no of points = 720
    p_occu = 0.5
    p_empty = 0.5
    angles = np.arange(-90,90,0.25)
    angles_full = np.arange(-180,180,0.25)
    cost_fun = np.zeros(720)
    speed = 0.3

    if err_pos > dist_precision_ and state_ == 1:

        distances = np.array(laser_data.ranges)
        detected_angles = np.abs(angles)
        detected_angles[distances == np.inf] = 90
        distances[distances == np.inf] = laser_data.range_max

        p_s_occu = ((laser_data.range_max - distances)/laser_data.range_max + (90 - detected_angles)/90) / 2

        p_occu_s = (p_occu * p_s_occu) / (p_occu * p_s_occu + p_empty * (1-p_s_occu))
        #p_occu_s = p_occu_s[::-1]

        max_surr = [p_occu_s[:i] for i in range(10, 20)] + [p_occu_s[i:i+20] for i in range(len(p_occu_s)-10)]
        max_surr = np.array([max(x) for x in max_surr])

        for i in range(len(distances)):
            if laser_data.ranges[i] < 1.5:
                max_surr[range(max(i-120, 0), min(i+120, 719))] = 1
                speed = 0.3

        cost_fun[max_surr > 0.2] = 1000
        cost_fun = np.concatenate((np.zeros(360),  cost_fun, np.zeros(360)))
        if(p_occu_s[0] > 0.2):
            cost_fun[:360]=500
        if(p_occu_s[-1] > 0.2):
            cost_fun[-360:] = 500
        cost_fun += a*np.abs(desired_err_yaw - angles_full*np.pi/180) + b*np.abs(angles_full*np.pi/180) + c*np.abs(angles_full*np.pi/180 - prev_angle)
        
        prev_angle = (np.argmin(cost_fun) - 720) * np.pi / 720
        print 'Yaw err: [%s], pos err: [%s][%s]' % (desired_yaw, prev_angle, desired_err_yaw)

        count += 1

        if(count == 80):
            #plotter(angles_full, cost_fun)
            count = 0
        
        twist_msg = Twist()
        twist_msg.linear.x = speed
        twist_msg.angular.z = prev_angle
        #if(np.abs(desired_err_yaw) < np.pi/2 and err_pos < laser_data.ranges[359 - int((desired_err_yaw * 180/np.pi)*4)] * 0.001):
        #    twist_msg.angular.z = desired_err_yaw
        pub.publish(twist_msg)
    else:
        change_state(2)
        done()

def main():
    global ax, bars, pub, desired_position_, active_, position_data, srv_client_spiral_
  
    #read position_data rom file
    position_data = np.genfromtxt("positionData.csv", delimiter=',')
    desired_position_.latitude = position_data[1][0]
    desired_position_.longitude = position_data[1][1]

    ax = plt.gca()
    bars = ax.plot(-np.arange(-90,90,0.25), np.zeros(720))

    rospy.init_node("VFH")

    sub = rospy.Subscriber('/i214/laser/scan', LaserScan,  clk_laser)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub_gps = rospy.Subscriber('/fix', NavSatFix, clbk_gps)

    sub_imu = rospy.Subscriber('/imu', Imu, clk_yaw)


    rospy.wait_for_service('/spiral_switch')

    srv_client_spiral_ = rospy.ServiceProxy('/spiral_switch', SetBool)

    rate = rospy.Rate(20)
    rate.sleep()
    rate.sleep()
    while not rospy.is_shutdown():
        if not active_:
            continue
        else:
            if state_ == 1:
                vfh()
            elif state_ == 2:
                set_next_goal()
            else:
                rospy.logerr('Unknown state!')

        rate.sleep()
    

    plt.show()

if __name__ == '__main__':
    main()
