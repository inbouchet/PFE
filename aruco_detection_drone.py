#!/usr/bin/python3

import numpy as np
import time
import cv2
from pfe_pi.msg import Dist
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ArucoDetection:

    def __init__(self):
        rospy.init_node("aruco_detection_origin", anonymous=True)

        self.pub = rospy.Publisher("/aruco_detection/dist", Dist, queue_size=10)
        self.cam = rospy.Subscriber("/sc/rgb/image",Image,self.detection)
        self.rate = rospy.Rate(75)
        self.detection()

        rospy.spin()

    def detection(self,msg):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        matrix = np.load("/home/pi/catkin_ws/src/pfe_pi/MultiMatrix.npz")
        cam_matrix = matrix["camMatrix"]
        dist_coef = matrix["distCoef"]

        dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)

        param = cv2.aruco.DetectorParameters_create()

        

        dist = Dist()


        corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, dict, parameters=param)
        
        if corners:
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 8, cam_matrix, dist_coef)
            corners = np.asarray(corners)

            if corners.size == 8:
            
                corners = corners.astype(int)
                corners = corners.reshape(4,2)

                distance = np.sqrt(tvec[0][0][2] ** 2 + tvec[0][0][0] ** 2 + tvec[0][0][1] ** 2)

                point = cv2.drawFrameAxes(cv_image, cam_matrix, dist_coef, rvec[0], tvec[0], 4, 4)

                dist.origin_found = True
                dist.dist_origin = distance
                dist.x = round(tvec[0][0][0],1)
                dist.y = round(tvec[0][0][1],1)

                # x -> à gauche/à droite
                # y -> en haut/en bas

                if dist.origin_found:
                    print(" Detection : distance =", dist.dist_origin, "x =", dist.x, "y =", dist.y)

                self.pub.publish(dist)

            #cv2.imshow("Image", img)

        
        cv2.destroyAllWindows()
        self.cam.release()

        self.rate.sleep()


if __name__ == "__main__":
    ArucoDetection()