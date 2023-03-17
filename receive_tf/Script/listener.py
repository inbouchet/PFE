#!/usr/bin/env python3

import rospy
from receive_tf.msg import Pos
from std_msgs.msg import String

def callback(data):
    print (data.x, data.y, data.dist_goal, data.angle)
def listener():


    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('chatter', Pos, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()


#Pour la communication
#ROS_HOSTNAME=px4vivision
#export ROS_MASTER_URI=http://192.168.1.73:11311