#!/usr/bin/env python2.7

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image



class Vision:
    def __init__(self):
        rospy.init_node('vision', anonymous=True)
        self.image_sub = rospy.Subscriber("/sc/rgb/image",Image,self.callback)
	self.tmp=0

    def callback(self,msg):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
	#hsv=cv2.cvtColor(cv_image,cv2.COLOR_GRAY2RGB)
	#cv2.imwrite("myImage"+str(self.tmp)+".jpg",cv_image)
	#self.tmp+=1
        cv2.imshow("image",cv_image)
	cv2.waitKey(0)
        cv2.destroyAllWindows()

v =Vision()
rospy.spin()

