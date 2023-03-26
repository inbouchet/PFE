#!/usr/bin/env python3

import rospy
from pfe.msg import Dist
from geometry_msgs.msg import Twist

class Controller:
    """
    The "Controller" class is responsible for controlling the movement of a robot based 
    on Aruco marker detection. It initializes the necessary variables and ROS nodes, 
    sets up a subscriber to receive messages from the "/aruco_detection/dist" topic, a
    nd a publisher to send messages to the "/cmd_vel" topic. 
    """

    def __init__(self):

        # Define initial values for class variables
        self.LEFT_LIMIT = -0.5
        self.RIGHT_LIMIT = 0.5
        self.LINEAR_SPEED = 0.2
        self.ANGULAR_SPEED = 0.1

        # Initialize the node and set the name as "controller_tb"
        rospy.init_node("controller_tb", anonymous=True)

        # Set up a subscriber to the "/aruco_detection/dist" topic and call the "control" function
        # when a message is received
        self.sub = rospy.Subscriber("/aruco_detection/dist", Dist, self.control)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.rate = rospy.Rate(10)
        rospy.spin()
        
    def control(self, aruco_detection):
        """
        The "control" function processes 
        the received messages, calculates the required velocity values based on the position of 
        the detected Aruco marker, and publishes them to the "/cmd_vel" topic. 
        """

        vel = Twist()

        #print(" Control : distance =", aruco_detection.dist_origin, "x =", aru>

        if aruco_detection.origin_found: 

            # If the marker is to the left of the left limit, turn right
            if aruco_detection.x < self.LEFT_LIMIT:
                vel.angular.z = self.ANGULAR_SPEED

            # If the marker is to the right of the right limit, turn left
            if aruco_detection.x > self.RIGHT_LIMIT:
                vel.angular.z = - self.ANGULAR_SPEED

        vel.linear.x = self.LINEAR_SPEED

        self.pub.publish(vel)
        self.rate.sleep()

if __name__ == "__main__":
    Controller()

