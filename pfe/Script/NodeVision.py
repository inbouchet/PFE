#!/usr/bin/env python2.7

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image



class Vision:
    '''
    The "Vision" class creates a ROS node to receive images from a specific topic ("/sc/rgb/image"), converts these images to OpenCV format, 
    displays them on a display window and waits for a key to be pressed to close the window.
    '''

    def __init__(self):
        rospy.init_node('vision', anonymous=True)
	
	# Create a subscriber to the "/sc/rgb/image" topic to receive images
   	 # When an image is received, the callback() method is called to process it
        self.image_sub = rospy.Subscriber("/sc/rgb/image",Image,self.callback)
	
	# Variable to store a temporary counter
	self.tmp=0

    def callback(self,msg):
	'''
	The "callback" method is called every time an image is received on the topic "/sc/rgb/image". This method uses the CvBridge 
	library to convert the image of its ROS format to OpenCV format, then displays it on a window named "image".
	'''
	
        bridge = CvBridge()
	
	# Convert the received image into OpenCV format
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
	
	# Convert the image to grayscale or HSV if needed
	#hsv=cv2.cvtColor(cv_image,cv2.COLOR_GRAY2RGB)
	
	# Save the converted image as a JPEG file (optional)
	#cv2.imwrite("myImage"+str(self.tmp)+".jpg",cv_image)
	#self.tmp+=1
	
        cv2.imshow("image",cv_image)
	cv2.waitKey(0)
        cv2.destroyAllWindows()

v =Vision()
rospy.spin()

