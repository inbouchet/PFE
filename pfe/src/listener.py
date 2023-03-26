#!/usr/bin/env python3

import rospy
from pfe.msg import Pos
from std_msgs.msg import String
import sys

class Listener:
    """
    The "Listener" class is responsible for receiving messages of type "Pos" from the "/chatter" 
    topic in ROS, and saving the position data contained in these messages to a file. 
    It initializes the necessary ROS node, sets up a subscriber to receive messages from the 
    "/chatter" topic, and defines a callback function to process the received messages. 
    """

    def __init__(self):

        rospy.init_node('listener', anonymous=True)

        # Subscribe to the "/chatter" topic to receive messages of type "Pos"
        self.sub = rospy.Subscriber('/chatter', Pos, self.callback)

        self.rate = rospy.Rate(10)
        rospy.spin()

    def callback(self, data):
        """
        The "callback" function redirects the standard output to a file, prints the received position 
        data to the file, and waits for the next loop iteration. The class runs in a loop with a 
        specified rate to continuously listen for messages on the "/chatter" topic.
        """

        # Define the path of the file where the position data will be saved
        path = '/home/inesbouchet/catkin_ws/src/pfe/src/position_drone.txt'

        # Redirect the standard output to the file
        sys.stdout = open(path, 'w')
        print(data.x, data.y, data.dist_goal, data.angle)
        sys.stdout.close()

        print(data.x, data.y, data.dist_goal, data.angle)

        #self.rate.sleep()

if __name__ == '__main__':
    Listener()

#export ROS_MASTER_URI=http://172.20.10.2:11311