#!/usr/bin/env python
PKG = 'zed_cam'

import sys
import cv2
import numpy as np
import rospy as rp
import roslib; roslib.load_manifest(PKG)
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String, Float32
from cv_bridge import CvBridge, CvBridgeError

# Determines if the program is being killed
kill = False

# Set up the ROS publisher
rp.init_node('cone_detection')
detection_msg  = rp.Publisher('cone_detect_msg', Bool, queue_size=10)
stopping_msg = rp.Publisher('cone_stop_detect_msg', Bool, queue_size=10)
percent_msg  = rp.Publisher('cone_center_percent_msg', Float32, queue_size=10)
rate = rp.Rate(20) # 20hz

# Structure that holds HSV color thresholds
class ColorThresh():
    def __init__(self):
        self.hsv_orange_lower = np.uint8([3, 150, 150])
        self.hsv_orange_upper = np.uint8([15, 255, 255])

# Class for cone detection
class ConeDetection():
    # Display the main video stream and the applied masks using cv2's imshow.
    def viewDisplay(self, stream, left_mask, right_mask):
        cv2.imshow('subcribed cone stream', stream)
        cv2.imshow('subcribed left mask stream', left_mask)
        cv2.imshow('subcribed right mask stream', right_mask)

        # Stop the loop and stop subscribing when q is pressed.
        if cv2.waitKey(1) & 0xFF ==ord('q'):
            cv2.destroyAllWindows()
            rp.signal_shutdown("Testing terminated")
            kill = True

    # Display the vales that will be set via the publisher.
    def viewPublishing(self, detected, stopping, percentage):
        print(" [Bool] Line detected:  ", detected)
        print(" [Bool] Is stopping     ", stopping)
        print("[Float] Center Percent: ", percentage)
        print("====================================")

    # Publish the messages
    def publish(self, detected, stopping, percentage):
        detection_msg.publish(detected)
        stopping_msg.publish(stopping)
        percent_msg.publish(percentage)

    # Calculate the percentage offset from the center of the camera.
    def calcPercentage(self, is_orange, x_avg):
        if is_orange == False:
            return None
        else:
            x_left  = 0.0 + x_avg
            x_right = 672.0 - x_avg

            average_diff = (x_left + x_right) / 2

            percentage_diff = abs(x_left - x_right) / average_diff
            direction = max({x_left, x_right})

            if direction != x_right:
                percentage_diff *= -1.0

            return percentage_diff

    # Checks the mask for any white pixels.
    def checkMask(self, image):
        points = cv2.findNonZero(image)
        num_points = cv2.countNonZero(image)

        valid = False
        stop = False
        count = 0
        x = 0
        y = 0

        if num_points is 0 or num_points is None:
            valid = False
        else:
            for i in points:
                count += 1

                x += i[0][0]
                y += i[0][1]

                if count >= 100:
                    valid = True
                else:
                    valid = False
                if count >= 1000:
                    # print('! TOO MANY PIXELS !')
                    break

        if num_points >= 25000:
            stop = True

        if count < 100:
            return valid, stop, None, None
        else:
            return valid, stop, x/count, y/count

    # Take an image, detect orange, and mask everything that's not orange.
    def applyMask(self, image):
        # Calls the ColorThresh structure
        color = ColorThresh()

        # Convert a BGR image to HSV
        converted = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Looks for a range of orange and masks it white. All non-orange is masked black.
        conv_thresh = cv2.inRange(converted, color.hsv_orange_lower, color.hsv_orange_upper)

        # Return the binary image
        return conv_thresh

    # Split the ZED camera image into two seperate images.
    def splitImage(self, image):
        left_image  = image[0:376, 0:672]
        right_image = image[0:376, 673:1344]

        return left_image, right_image

    # Main method
    def run(self, frame):
        mask = self.applyMask(frame)
        left_mask, right_mask = self.splitImage(mask)

        cone_in_left,  left_stop,  left_x,  left_y  = self.checkMask(left_mask)
        cone_in_right, right_stop, right_x, right_y = self.checkMask(left_mask)
        left_percentage  = self.calcPercentage(cone_in_left, left_x)
        right_percentage = self.calcPercentage(cone_in_right, right_x)

        # Combining both cone booleans to see if the cam sees the cone.
        if cone_in_left == True or cone_in_right == True:
            cone_detected = True
        else:
            cone_detected = False

        # Combining both stop booleans to see if the car should stop.
        if left_stop == True or right_stop == True:
            cone_stop = True
        else:
            cone_stop = False

        # Combining both line percents to grab the average percentage.
        if left_percentage == None and right_percentage == None:
            avg_percentage = None
        elif left_percentage == None:
            avg_percentage = (right_percentage - 0.15) * 0.155
        elif right_percentage == None:
            avg_percentage = (left_percentage + 0.15) * 0.155
        else:
            avg_percentage = ((left_percentage + right_percentage) / 2) * 0.155

        self.publish(cone_detected, cone_stop, avg_percentage)
        # For testing only
        # self.viewDisplay(frame, left_mask, right_mask)
        # self.viewPublishing(cone_detected, cone_stop, avg_percentage)

class Listener():
    def run(self):
        # Loop
        while not rp.is_shutdown():
            rp.Subscriber('image_msg', Image, callback)

            # Kill the program if kill is true
            if kill:
                break

            # Keeps python from exiting until this node is stopped.
            rp.spin()

def callback(data):
    ConeDetection().run(CvBridge().imgmsg_to_cv2(data))

if __name__ == '__main__':
    try:
        Listener().run()
    except rp.ROSInterruptException:
        pass
