#!/usr/bin/python2.7
# -*- coding: utf-8 -*-

import numpy as np
import time
import math
import cv2
from pfe.msg import Pos
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ArucoDetection:
    '''
    The "ArucoDetection" class uses the aruco_display function to detect an aruco marker in an image. The detection function is called 
    to calculate the distance between the drone and the aruco marker, as well as the offset angle. To do this, the function converts 
    the ROS image into a CV2 image, defines an ArUco dictionary, initializes a PoseStamped message, retrieves the camera matrix and 
    distortion coefficients from the camera_info subject, detects ArUco markers in the image, and calculates the distance and offset angle 
    between the drone and the Aruco.
    '''

    def __init__(self):
        rospy.init_node("aruco_detection", anonymous=True)

	# Subscribe to camera image topic
        self.cam = rospy.Subscriber("/sc/rgb/image",Image,self.detection)
	
	# Create publisher "dist" to publish position data
        self.pub = rospy.Publisher("dist",Pos,queue_size=10)
	
	# Set initial state variables
        self.find_goal=False
        self.first_distance=False
        self.send=False
	self.tvec1=0
       
        
    #The function aruco_display is from https://github.com/niconielsen32/ComputerVision/blob/master/ArUco/arucoDetection.py
    def aruco_display(self,corners, ids, rejected, image):
	'''
	The "aruco_display" function is used to display ArUco markers on an image. It takes in the detected corners, ids, rejected, and the 
	image itself as inputs. The corners and ids are from the ArUco detection algorithm. The rejected parameter is not used in this function.
	
	Return: 
	- The function returns the marker ID of the last detected marker. If no markers were detected, the value is -1.
	'''
	
        markerID=-1
	if len(corners) > 0:
		
                ids = ids.flatten()
		
                # loop over detected markers and draw lines around them
                for (markerCorner, markerID) in zip(corners, ids):
                
                        corners = markerCorner.reshape((4, 2))
                        (topLeft, topRight, bottomRight, bottomLeft) = corners
                        
                        topRight = (int(topRight[0]), int(topRight[1]))
                        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                        topLeft = (int(topLeft[0]), int(topLeft[1]))

                        cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                        cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                        cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                        cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
                        
			# calculate center of marker and draw a circle
                        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                        cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
                        
			# draw the marker ID
                        cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 255, 0), 2)
                        print("[Inference] ArUco marker ID: {}".format(markerID))
			
	return markerID


    def detection(self,msg):
	'''
	The function "detection" detects ArUco markers in an image and calculates the position and orientation of the marker.
	The function calculates the pose of the goal marker and the intrinsic camera and extrinsic distortion matrices obtained 
	from the ROS camera information message. She then calculates the distance and angle between the current position of the 
	robot and the position of the goal marker, and publishes this information on a ROS topic.
	'''
		
	# Convert the ROS Image message to a CV2 image
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
	
	# Define the ArUco dictionary
        dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
	
	# Initialize the PoseStamped message
        pos=Pos()
        param = cv2.aruco.DetectorParameters_create()
	
	# Get the camera matrix and distortion coefficients from the camera_info topic
	cam_matrix = np.array([[265.8909912109375, 0.0, 316.2030029296875], [0.0, 266.125, 248.2239990234375], [0.0, 0.0, 1.0]],dtype=float)
	dist_coef = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]],dtype=float)
	
	# Initialize variables
	x=-1
	y=-1
	
	# Detect ArUco markers in the image
        corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, dict, parameters=param)
        detected= self.aruco_display(corners,ids,rejected,cv_image)

	# If a marker is detected
        if detected==1:
                print("goal")
                self.find_goal=True
		for i in range (len(ids)):
			corners[i] = np.asarray(corners[i])
	
	# If a marker was previously detected but is no longer detected
        if self.find_goal==True and detected==0:
		
		# If this is the first time the marker is no longer detected
                if(not(self.first_distance)):
                        print("origin")
                        for i in range (len(ids)):
                                corners[i] = np.asarray(corners[i])
                                rvec1, self.tvec1 = cv2.aruco.estimatePoseSingleMarkers(corners[i], 8.0,cam_matrix,dist_coef)
                                self.first_distance=True
				
		# If this is not the first time the marker is no longer detected
                else:
                        for i in range (len(ids)):
                                corners[i] = np.asarray(corners[i])
				
                                rvec2, tvec2 = cv2.aruco.estimatePoseSingleMarkers(corners[i], 8.0,cam_matrix,dist_coef)
				
				# Calculation of the orientation angle between the two points of positions x and y
                                angle=np.arctan2(round(tvec2[0][0][1],1),round(tvec2[0][0][0],1))
                                print(angle)
				
				# If the drone is in the axis of the detector
                                if (angle<-1.56 and angle>-1.58):
                                        
                                        if corners[i].size==8:

                                                pos.dist_goal = np.linalg.norm(self.tvec1)
                                                pos.x=round(self.tvec1[0][0][0],1)
                                                pos.y=round(self.tvec1[0][0][1],1)
						
						# Calculation of the angle thanks to the two drone-marker distances
                                                pos.angle=math.acos(np.linalg.norm(tvec2)/np.linalg.norm(self.tvec1))
						
                                                if(not(self.send)):
                                                        self.pub.publish(pos)
                                                        print("send")
                                                        print("y=",pos.y," x=",pos.x," distance=",pos.dist_goal,"angle",pos.angle)
                                	                self.send=True
                        





ArucoDetection()
rospy.spin()
