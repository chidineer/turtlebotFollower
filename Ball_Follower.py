#!/usr/bin/env python3

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class Ball_Follower:
    def __init__(self):
        self.pub = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(1)
        self.position = Twist()

        self.cx = 0
        self.cy = 0

        self.bridge = CvBridge()
        self.image_received = False

        # Connect image topic
        img_topic = "/tb3_1/camera/rgb/image_raw"
        self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)

        # Allow up to one second to connection
        rospy.sleep(1)

    def callback(self, data):
        # Convert image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.image_received = True
        self.image = cv_image
        self.find_object(cv_image)
        self.move_to_object()

    def find_object(self, img):
        hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsv_frame = cv2.resize(hsv_frame, (640, 300))
        img = cv2.resize(img, (640, 300))

        low_H = 0
        low_S = 200
        low_V = 50
        high_H = 180
        high_S = 255
        high_V = 255

        mask_frame = cv2.inRange(hsv_frame, (low_H, low_S, low_V), (high_H, high_S, high_V))
        contours, _ = cv2.findContours(mask_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]

        largest_contour = None
        largest_area = 0
        X, Y, W, H = 0, 0, 0, 0

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

        print(self.cx)
        cv2.imshow("Window", img)
        cv2.waitKey(3)

    def move_to_object(self):
        if self.cx == 0:
            text = "Searching"
            self.position.angular.z = -0.4
            self.position.linear.x = 0
        else:
            # Calculate the error (difference from the desired center)
            error = self.cx - 320

            # Define a dead zone to prevent small movements
            dead_zone = 20

            if error > dead_zone:
                text = "Left"
                self.position.angular.z = -0.1
                self.position.linear.x = 0
            else:
                text = "Right"
                self.position.angular.z = 0.1
                self.position.linear.x = 0

        self.pub.publish(self.position)
        print(text)


if __name__ == '__main__':
    rospy.init_node('Ball_Follower', anonymous=False)
    camera = Ball_Follower()

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        rospy.spin()

    # Close all OpenCV windows before exiting
    cv2.destroyAllWindows()

