#!/usr/bin/env python2.7

import rospy
from std_msgs.msg import String
from pfe.msg import Pos

class Drone_Communication:
    '''
    The "Drone_Communication" class creates a ROS node named "talker" with a message editor to publish "Pos" messages on a topic named "chat". 
    This class also creates a topic subscriber named "dist" to receive "Pos" messages.
    '''

    def __init__(self):
	
	# Create a publisher to the "chatter" topic with message type Pos
        self.pub = rospy.Publisher('chatter', Pos, queue_size=10)
	
	# Create a subscriber to the "dist" topic with message type Pos
    	# When a message is received, the callback() method is called to process it
        self.sub = rospy.Subscriber("dist",Pos,self.callback)
	
	# Initialize a ROS node named "talker"
        rospy.init_node('talker', anonymous=True)

    def callback(self,msg):
	'''
	The callback() method publish the received message on the "chat" topic and prints the information of the received message.
	'''
	
        self.pub.publish(msg)
        print(msg.x,msg.y,msg.dist_goal,msg.angle)

if __name__ == '__main__':
    try:
        Drone_Communication()
	    rospy.spin()
    except rospy.ROSInterruptException:
        pass
