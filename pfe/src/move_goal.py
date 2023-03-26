#!/usr/bin/python3

import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

class MoveGoal:
    """
    The "MoveGoal" class implements a ROS (Robot Operating System) node allowing a drone 
    to move to a given point. Specifically, the MoveGoal class reads the coordinates and 
    angle of the destination point from a text file, then creates a list of points around 
    the destination point. It then uses the drone's odometry to determine its current position 
    and orientation, and compares the drone's current orientation with the point list angle 
    to ensure the drone is heading in the correct direction. 
    """

    def __init__(self):
        rospy.init_node("move_pose")

        # Initialize the goal point and the radius
        self.goal = Point()
        self.radius = 0.4

        file = open("position_drone.txt","r")

        # Extract the goal point and the angle from the file
        for i in file:
            tab = i.split(" ")

        self.distance = float(tab[2]) + 0.26 # size of turtlebot
        self.angle = float(tab[3]) 

        # Estimate goal position with angle and distance
        self.goal.x = math.cos(self.angle) * self.distance * 0.01
        self.goal.y = math.sin(self.angle) * self.distance * 0.01

        file.close()

        # Create the list of points around the goal point 
        self.around = self.create_list(self.radius, self.goal.x, self.goal.y, self.angle)

        # Subscribe to the "/odom" topic to get the drone's position 
        self.sub = rospy.Subscriber("/odom", Odometry, self.move_pose)

        # Publish to the /cmd_vel topic to move the drone
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

        self.rate = rospy.Rate(75)
        rospy.spin()

    def create_list(self, r, x, y, angle):
        """ 
        The function create_list creates a numpy array that represents a circular 
        path around a specified point (x,y) with a given radius r and angle angle. 
        The function returns an array of coordinates of points on this circular path. 
        The direction of the path is determined by the sign of the angle. 
        """

        # If the angle is positive, create a list of points clockwise around the goal point
        if angle > 0:
            return np.array([[x , y - r] , [x + r, y] , [x, y + r] , [x - r, y], [0, 0]])

        # If the angle is negative, create a list of points counterclockwise around the goal point
        if angle < 0:
            return np.array([[x , y + r] , [x - r, y] , [x, y - r] , [x + r, y], [0, 0]])

    def move_pose(self, msg):
        """
        The "move_pose" is called each time new data is received from the "/odom" topic. 
        It receives an Odometry object which contains the current position of the drone 
        and the orientation, and it updates the commands to send to the drone to reach 
        the desired position.
        """

        # Initialize the Twist message to send movement commands
        speed = Twist()
        
        # Get the drone's current position and orientation
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

        # Calculate the distance between the current position and the first point of the list
        new_x = self.around[0, 0] - x
        new_y = self.around[0, 1] - y

        angle_to_goal = atan2(new_y, new_x)

        # If the drone is not facing the correct angle to move towards the goal point, adjust its rotation
        if (angle_to_goal - theta) > 0.1:
            if (angle_to_goal - theta) < 2.5:
                speed.linear.x = 0.0
                speed.angular.z = 0.3
            else:
                speed.linear.x = 0.0
                speed.angular.z = - 0.3
        elif (angle_to_goal - theta) < - 0.1:
            if (angle_to_goal - theta) > - 2.5:
                speed.linear.x = 0.0
                speed.angular.z = - 0.3
            else:
                speed.linear.x = 0.0
                speed.angular.z = 0.3
        else:
            speed.linear.x = 0.3
            speed.angular.z = 0.0

        # Delete the current position in the list of points to go to the next one
        if new_x < 0.1 and new_x > - 0.1 and new_y < 0.1 and new_y > - 0.1:
            if self.around.shape[0] > 1:
                self.around = np.delete(self.around, 0)
                self.around = np.delete(self.around, 0)
                self.around = np.reshape(self.around, (-1, 2))
            else:
                speed.linear.x = 0.0
                speed.angular.z = 0.0

        self.pub.publish(speed)
        self.rate.sleep()

if __name__ == "__main__":
    MoveGoal()
