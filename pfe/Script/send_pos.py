#!/usr/bin/env python2.7

import rospy
from std_msgs.msg import String
from pfe.msg import Pos

class Drone_Communication:
    def __init__(self):
        self.pub = rospy.Publisher('chatter', Pos, queue_size=10)
        self.sub = rospy.Subscriber("dist",Pos,self.callback)
        rospy.init_node('talker', anonymous=True)

    def callback(self,msg):

        self.pub.publish(msg)
        print(msg.x,msg.y,msg.dist_goal,msg.angle)

if __name__ == '__main__':
    try:
        Drone_Communication()
	    rospy.spin()
    except rospy.ROSInterruptException:
        pass
