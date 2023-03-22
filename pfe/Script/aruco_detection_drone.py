#!/usr/bin/python2.7
# -*- coding: utf-8 -*-

import numpy as np
import time
import cv2
from pfe.msg import Pos
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ArucoDetection:

    def __init__(self):
        rospy.init_node("aruco_detection_origin", anonymous=True)

        self.cam = rospy.Subscriber("/sc/rgb/image",Image,self.detection)
        self.pub = rospy.Publisher("dist",Pos,queue_size=10)
        self.find_goal=False
        self.send=False
	self.tvec1=0
       
        
#The function aruco_display is from https://github.com/niconielsen32/ComputerVision/blob/master/ArUco/arucoDetection.py

    def aruco_display(self,corners, ids, rejected, image):
        markerID=-1
	if len(corners) > 0:
		
                ids = ids.flatten()
            
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
                        
                        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                        cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
                        
                        cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 255, 0), 2)
                        print("[Inference] ArUco marker ID: {}".format(markerID))
			
	return markerID


    def detection(self,msg):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        pos=Pos()
        param = cv2.aruco.DetectorParameters_create()
	#These parameters are from sc/rgb/camera_info topic
	cam_matrix = np.array([[265.8909912109375, 0.0, 316.2030029296875], [0.0, 266.125, 248.2239990234375], [0.0, 0.0, 1.0]],dtype=float)
	dist_coef = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]],dtype=float)
	x=-1
	y=-1
        corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, dict, parameters=param)
        detected= self.aruco_display(corners,ids,rejected,cv_image)

        if detected==1:
                print("goal")
                self.find_goal=True
		for i in range (len(ids)):
			corners[i] = np.asarray(corners[i])
			rvec1,self.tvec1 = cv2.aruco.estimatePoseSingleMarkers(corners[i],8.0,cam_matrix,dist_coef)
        if self.find_goal==True and detected==0:
                print("origin")
                for i in range (len(ids)):
                        corners[i] = np.asarray(corners[i])
                        rvec2, tvec2 = cv2.aruco.estimatePoseSingleMarkers(corners[i], 8.0,cam_matrix,dist_coef)
			if corners[i].size==8:
                        	pos.dist_goal = np.linalg.norm(self.tvec1-tvec2)
                        	pos.x=round(tvec2[0][0][0],1)
                        	pos.y=round(tvec2[0][0][1],1)
                        	pos.angle=np.arctan2(pos.y-round(self.tvec1[0][0][0],1),pos.x-round(self.tvec1[0][0][1],1))
                        	if(not(self.send)):
                                	self.pub.publish(pos)
                                	print("send")
                                	print("y=",pos.y," x=",pos.x," distance=",pos.dist_goal,"angle",pos.angle)
                                	self.send=True
                        





ArucoDetection()
rospy.spin()
