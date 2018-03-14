#!/usr/bin/env python
"""
Author: Will Irwin
Last Modified: 2/12/2018
Title:zed.py
Inputs:

Outputs:

"""
import cv2
import rospy as rp
import numpy as np
from sensor_msgs.msg import Image
# import sensor_msgs.msg.Image
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
cap = cv2.VideoCapture(0)
def get_zed_data():
    pub = rp.Publisher('zed_images',Image,queue_size=1)
    rate = rp.Rate(60)#Hz
    while (not rp.is_shutdown()) and (cap.isOpened()):
        ret, frame = cap.read()
        try:
            output = bridge.cv2_to_imgmsg(frame, "bgr8")
        except CvBridgeError as e:
            print(e)
        #rp.loginfo(output)
        pub.publish(output)
        rate.sleep()
    cap.release()

if __name__=="__main__":
    try:
        rp.init_node('Zed',anonymous = True)
        get_zed_data()
    except rp.ROSInterruptException:
        cap.release()

