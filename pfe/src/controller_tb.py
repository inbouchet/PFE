#!/usr/bin/env python3

import rospy
from pfe.msg import Dist
from geometry_msgs.msg import Twist

class Controller:

    def __init__(self):

        self.LEFT_LIMIT = -0.5
        self.RIGHT_LIMIT = 0.5
        self.LINEAR_SPEED = 0.2
        self.ANGULAR_SPEED = 0.1

        rospy.init_node("controller_tb", anonymous=True)

        self.sub = rospy.Subscriber("/aruco_detection/dist", Dist, self.control)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.rate = rospy.Rate(10)
        rospy.spin()

    def control(self, aruco_detection):

        vel = Twist()

        #print(" Control : distance =", aruco_detection.dist_origin, "x =", aru>

        if aruco_detection.origin_found: 

            if aruco_detection.x < self.LEFT_LIMIT:
                vel.angular.z = self.ANGULAR_SPEED

            if aruco_detection.x > self.RIGHT_LIMIT:
                vel.angular.z = - self.ANGULAR_SPEED

        vel.linear.x = self.LINEAR_SPEED

        self.pub.publish(vel)
        self.rate.sleep()

if __name__ == "__main__":
    Controller()

