#!/usr/bin/env python

from __future__ import print_function
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class Turtlebot_Follower:
    def __init__(self):
        self.pub = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(1)
        self.position = Twist()

        self.cx = 0
        self.cy = 0

        self.bridge = CvBridge()
        self.image_received = False

        # Connect image topics
        depth_topic = "/tb3_1/camera/depth/image_raw"
        #depth_topic = "/tb3_1/camea/rgb/compressedDepth"
        rgb_topic = "/tb3_1/camera/rgb/image_raw"

        self.depth_sub = rospy.Subscriber(depth_topic, CompressedImage, self.depth_callback)
        self.rgb_sub = rospy.Subscriber(rgb_topic, CompressedImage, self.rgb_callback)

        # Allow up to one second for connection
        rospy.sleep(1)

    def depth_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)

        self.image_received = True
        self.depth_image = cv_image
        self.find_object_depth(cv_image)
        self.move_to_object()

    def rgb_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.image_received = True
        self.rgb_image = cv_image
        self.find_object_rgb(cv_image)
        self.move_to_object()

    # Shows the image from the depth camera and returns the depth of the center pixel
    def find_object_depth(self, img):
        img = cv2.resize(img, (640, 300))
        center_depth = img[self.cy, self.cx]
        cv2.imshow("Depth_Camera", img)
        cv2.waitKey(3)

        return center_depth

    # Shows the RGB Camera image and locks on to an object within the HSV value range
    def find_object_rgb(self, img):
        hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsv_frame = cv2.resize(hsv_frame,(640,300))
        img = cv2.resize(img, (640, 300))

        low_H = 0
        low_S = 175
        low_V = 200
        high_H = 180
        high_S = 255
        high_V = 255

        mask_frame = cv2.inRange(hsv_frame, (low_H, low_S, low_V), (high_H, high_S, high_V))
        contours, _ = cv2.findContours(mask_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]

    
        largest_contour = None
        largest_area = 0
        X,Y,W,H=0,0,0,0

        for contour in contours:
            area = cv2.contourArea(contour)

            if area > largest_area:
                largest_area = area
                largest_contour = contour

        if largest_contour is not None:
            x, y, w, h = cv2.boundingRect(largest_contour)

            if w * h > largest_area:
                X, Y, W, H = x, y, w, h


        cv2.rectangle(img, (X, Y), (X + W, Y + H), (0, 0, 255), 2)
        self.cx = X + W // 2
        self.cy = Y + H // 2

        cv2.imshow("RGB_Camera", img)
        cv2.waitKey(3)

    # Moves towards an object until its within a safe distance.
    def move_to_object(self):
        error = self.cx - 320
        dead_zone = 20
        center_depth = self.find_object_depth(self.depth_image)

        if self.cx == 0:
            text = "Searching"
            self.position.angular.z = -0.4
            self.position.linear.x = 0
        else:
            if error <= dead_zone and error >= -dead_zone:
                self.position.angular.z = 0
                if center_depth < 0.8 and center_depth > 0.5:
                    text="Stopped"
                    self.position.linear.x = 0
                elif center_depth < 0.5:
                    text="Backward"
                    self.position.linear.x = -0.2
                else:
                    text="Forward"
                    self.position.linear.x = 0.2  
            elif error > dead_zone:
                text = "Right"
                self.position.angular.z = -0.4 * (error) / 320
                self.position.linear.x = 0
            else:
                text = "Left"
                self.position.angular.z = 0.4 * (-error) / 320
                self.position.linear.x = 0

        self.pub.publish(self.position)
        print(text)

if __name__ == '__main__':
    rospy.init_node('Turtlebot_Follower', anonymous=False)
    camera = Turtlebot_Follower()

    rospy.spin()

    # Close all OpenCV windows before exiting
    cv2.destroyAllWindows()



