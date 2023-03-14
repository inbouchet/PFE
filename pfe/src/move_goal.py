#!/usr/bin/python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

class MoveGoal:

    def __init__(self):
        rospy.init_node("move_pose")

        self.goal = Point()
        self.goal.x = 5
        self.goal.y = 5
        self.rayon = 1

        self.around = self.create_list(self.rayon, self.goal.x, self.goal.y)

        self.sub = rospy.Subscriber("/odom", Odometry, self.move_pose)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

        self.rate = rospy.Rate(75)
        rospy.spin()

    def create_list(self, r, x, y):
        return np.array([[x , y - r] , [x + r, y] , [x, y + r] , [x - r, y], [0, 0]])

    def move_pose(self, msg):

        speed = Twist()
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

        new_x = self.around[0, 0] - x
        new_y = self.around[0, 1] - y

        angle_to_goal = atan2(new_y, new_x)
        print(angle_to_goal - theta)

        if (angle_to_goal - theta) > 0.1:
            speed.linear.x = 0.0
            speed.angular.z = 0.3
        elif (angle_to_goal - theta) < -0.1:
            speed.linear.x = 0.0
            speed.angular.z = -0.3
        else:
            speed.linear.x = 0.5
            speed.angular.z = 0.0

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