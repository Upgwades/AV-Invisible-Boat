#!/usr/bin/env python
import cv2
import rospy as rp
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from sensor_msgs.msg import Image,LaserScan

#move_pub = rp.Publisher('movement_instructions',AckermannDriveStamped,queue_size=10)

def callback(data):
    pass
    #print data


def get_laser_data():
    rp.Subscriber('/scan',LaserScan,callback)
    pub = rp.Publisher('lidar_instructions',AckermannDriveStamped,queue_size=1)
    rate = rp.Rate(60)#Hz    
    while not rp.is_shutdown():
        
        rate.sleep()
    

if __name__=="__main__":
    try:
        rp.init_node('Lidar',anonymous = True)
        get_laser_data()
    except rp.ROSInterruptException:
        cap.release()

