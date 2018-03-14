#!/usr/bin/env python
"""
Author: Will Irwin
Last Modified: 2/22/2018
Title:driver.py
Inputs:
 
Outputs:
 
"""
import cv2
import rospy as rp
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
 
pub=rp.Publisher("/vesc/ackermann_cmd_mux/input/navigation",AckermannDriveStamped,queue_size=10)#i think this needs to change
def callback(data):
    try:
        pub.publish(data)
        print(data)
    except:
        print("couldnt publish driving instructions, worthless garbage")
 
def full_stop(data):
    try:
        pub.publish(data)
    except:
        print("full stop instructions failed to execute, trash software")
 
 
def driver():
    rp.Subscriber('movement_instructions',AckermannDriveStamped,callback)
    rp.Subscriber('full_stop',AckermannDriveStamped,full_stop)
    rp.spin()
 
 
if __name__=="__main__":
    try:
        rp.init_node('Driver',anonymous = True)
        driver()
    except rp.ROSInterruptException:
        pass
    cv2.destroyAllWindows()
