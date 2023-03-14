#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry

def callback(msg):
    print("Turtlebot actual position : %s", msg.pose.pose)

rospy.init_node('odometry')
odom_sub = rospy.Subscriber('/odom', Odometry, callback)
rospy.spin()
