#!/usr/bin/env python
PKG = 'zed_cam'

import sys
import cv2
import numpy as np
import rospy as rp
import roslib; roslib.load_manifest(PKG)
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ReadZED():
    def viewDisplay(self, stream):
        cv2.imshow('published stream', stream)

    def run(self):
        # Set up the ROS publisher
        rp.init_node('zed_data_pub')
        zed_msg  = rp.Publisher('image_msg', Image, queue_size=10)
        rate = rp.Rate(20) # 20hz

        # Get video capture from the ZED
        video = cv2.VideoCapture(0)

        # Loop
        while not rp.is_shutdown():
            ret, frame = video.read()

            # Convert the image into something ROS can understand.
            image_msg = CvBridge().cv2_to_imgmsg(frame)

            # For testing only.
            # self.viewDisplay(frame)

            # Publish the messages.
            zed_msg.publish(image_msg)

            # Stop the loop when q is pressed.
            if cv2.waitKey(1) & 0xFF ==ord('q'):
                break

        # Release everything
        video.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        ReadZED().run()
    except rp.ROSInterruptException:
        pass
