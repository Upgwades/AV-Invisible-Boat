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
rp.init_node('tape_detection')
detection_msg  = rp.Publisher('tape_detect_msg', Bool, queue_size=10)
percent_msg  = rp.Publisher('tape_center_percent_msg', Float32, queue_size=10)
rate = rp.Rate(20) # 20hz

# Structure that holds HSV color thresholds
class ColorThresh():
    def __init__(self):
        self.hsv_orange_lower = np.uint8([110, 125, 100])
        self.hsv_orange_upper = np.uint8([130, 255, 255])

class TapeDetection():
    # Display the main video stream and the applied masks using cv2's imshow.
    def viewDisplay(self, stream, left_mask, right_mask):
        cv2.imshow('subcribed tape stream', stream)
        cv2.imshow('subcribed left mask stream', left_mask)
        cv2.imshow('subcribed right mask stream', right_mask)

        # Stop the loop and stop subscribing when q is pressed.
        if cv2.waitKey(1) & 0xFF ==ord('q'):
            cv2.destroyAllWindows()
            rp.signal_shutdown("Testing terminated")
            kill = True

    # Display the vales that will be set via the publisher.
    def viewPublishing(self, detected, percentage):
        print(" [Bool] Line detected:  ", detected)
        print("[Float] Center Percent: ", percentage)
        print("====================================")

    # Publish the messages
    def publish(self, detected, percentage):
        detection_msg.publish(detected)
        percent_msg.publish(percentage)

    # Calculate the percentage offset from the center of the camera.
    def calcPercentage(self, is_blue, x_avg):
        if is_blue == False:
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
        count = 0
        x = 0
        y = 0

        if num_points is 0 or num_points is None:
            valid = False
        else:
            for i in points[::-1]:
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

        if count < 100:
            return valid, None, None
        else:
            return valid, x/count, y/count

    # Apply a mask based off the currently set HSV color values.
    def applyMask(self, image):
        # Calls the ColorThresh structure
        color = ColorThresh()

        # Convert a BGR image to HSV
        converted = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Looks for a range of orange and masks it white. All non-orange is masked black.
        conv_thresh = cv2.inRange(converted, color.hsv_orange_lower, color.hsv_orange_upper)

        # Return the binary image
        return conv_thresh

    # Takes an image and crops out everything outside a region of interest
    def filterRegion(self, image):
        # Get the dimensions of the image
        rows, cols = image.shape[:2]

        # Define the polygon by vertices
        bottom_left     = [int(cols*0.0), int(rows*0.5)]
        bottom_right    = [int(cols*1.0), int(rows*0.5)]
        top_left        = [int(cols*0.0), int(rows*1.0)]
        top_right       = [int(cols*1.0), int(rows*1.0)]

        # Store the vertices into a numpy array
        vertices = np.int32(np.array([[bottom_left, top_left, top_right, bottom_right]]))

        # Set up an image mask
        mask = np.zeros_like(image)

        # Set up a mask within the polygon
        if len(mask.shape) == 2:
            cv2.fillPoly(mask, vertices, 255)
        else:
            cv2.fillPoly(mask, vertices, (255)*mask.shape[2])

        # Crop out everything outside of the region of interest
        roi = cv2.bitwise_and(image, mask)

        # Return the region of interest image
        return roi

    # Split the ZED image into two seperate images.
    def splitImage(self, image):
        left_image  = image[0:376, 0:672]
        right_image = image[0:376, 673:1344]

        # Return the split image
        return left_image, right_image

    # Main method
    def run(self, frame):
        mask = self.applyMask(frame)
        roi_mask = self.filterRegion(mask)
        left_mask, right_mask = self.splitImage(roi_mask)

        line_in_left,  left_x,  left_y  = self.checkMask(left_mask)
        line_in_right, right_x, right_y = self.checkMask(left_mask)
        left_percentage  = self.calcPercentage(line_in_left, left_x)
        right_percentage = self.calcPercentage(line_in_right, right_x)

        # Combining both line booleans to see if the cam sees the tape.
        if line_in_left == True or line_in_right == True:
            line_detected = True
        else:
            line_detected = False

        # Combining both line percents to grab the average percentage.
        if left_percentage == None and right_percentage == None:
            avg_percentage = None
        elif left_percentage == None:
            avg_percentage = (right_percentage - 0.15) * 0.155
        elif right_percentage == None:
            avg_percentage = (left_percentage + 0.15) * 0.155
        else:
            avg_percentage = ((left_percentage + right_percentage) / 2) * 0.155

        self.publish(line_detected, avg_percentage)
        # For testing only
        # self.viewDisplay(frame, left_mask, right_mask)
        # self.viewPublishing(line_detected, avg_percentage)

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
    TapeDetection().run(CvBridge().imgmsg_to_cv2(data))

if __name__ == '__main__':
    try:
        Listener().run()
    except rp.ROSInterruptException:
        pass
