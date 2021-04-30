#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist



def main():
    rospy.init_node("test_node")

    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
    rospy.sleep(0.5)
    msg = Twist()
    msg.angular.z = 0.5
    msg.linear.x = -1
    
    pub.publish(msg)
    rospy.spin()
if __name__ == '__main__':
    main()
