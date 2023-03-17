#!/usr/bin/python2.7
# -*- coding: utf-8 -*-

import rospy
from pfe.msg import Pos
import tf

class SendPos:

    def __init__(self):
        rospy.init_node('drone_broadcaster')
        self.sending =rospy.Subscriber("dist",Pos,self.handle_goal_pose)

    def handle_goal_pose(self,msg):
        br = tf.TransformBroadcaster()
        br.sendTransform((msg.x,msg.y,msg.dist_goal),(0,0,angle,0), rospy.Time.now(),"Turtlebot3","px4vision")

SendPos()        
rospy.spin()