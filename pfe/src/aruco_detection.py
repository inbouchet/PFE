#!/usr/bin/python3

import numpy as np
import time
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pfe.msg import Dist
import rospy

class ArucoDetection:
    """
    This class implements Aruco marker detection using OpenCV library 
    and publishes the distance and position of the marker from the 
    camera in the form of custom message.
    """

    bridge = CvBridge()

    def __init__(self):
        
        """ 
        Initializes the node, subscribes to the camera topic, and publishes
        the detected marker's distance and position.
        """

        rospy.init_node("aruco_detection", anonymous=True)

        self.sub = rospy.Subscriber("/camera/image", Image, self.detection)
        self.pub = rospy.Publisher("/aruco_detection/dist", Dist, queue_size=10)

        self.rate = rospy.Rate(75)

        rospy.spin()

    def detection(self, data):

        """
        Performs the Aruco marker detection and publishes the distance
        and position of the detected marker.

        Parameters:
            - data (sensors_msg.msg.Image): The image data from the 
              camera topic.
        """
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Load the camera matrix and distortion coefficients from file
        matrix = np.load("/home/inesbouchet/catkin_ws/src/pfe/src/MultiMatrix.npz")
        cam_matrix = matrix["camMatrix"]
        dist_coef = matrix["distCoef"]

        # Define the dictionary used for Aruco marker detection
        dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)

        # Define the parameters used for Aruco marker detection
        param = cv2.aruco.DetectorParameters()

        dist = Dist()

        corners, ids, rejected = cv2.aruco.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), dict, parameters=param)
        
        if corners:
            # Estimate the pose of the detected marker
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 8, cam_matrix, dist_coef)
            
            corners = np.asarray(corners)

            if corners.size == 8:
            
                corners = corners.astype(int)
                corners = corners.reshape(4,2)

                # Calculate distance to marker and draw axes
                distance = np.sqrt(tvec[0][0][2] ** 2 + tvec[0][0][0] ** 2 + tvec[0][0][1] ** 2)

                point = cv2.drawFrameAxes(img, cam_matrix, dist_coef, rvec[0], tvec[0], 4, 4)

                # Fill in data for message and publish
                dist.origin_found = True
                dist.dist_origin = distance
                dist.x = round(tvec[0][0][0],1)
                dist.y = round(tvec[0][0][1],1)

                if dist.origin_found:
                    print(" Detection : distance =", dist.dist_origin, "x =", dist.x, "y =", dist.y)

                self.pub.publish(dist)

        
        cv2.destroyAllWindows()

        self.rate.sleep()


if __name__ == "__main__":
    ArucoDetection()
