#!/usr/bin/env python
import cv2
import rospy as rp
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from sensor_msgs.msg import Image,LaserScan

#move_pub = rp.Publisher('movement_instructions',AckermannDriveStamped,queue_size=10)

def callback(data):
    straight = data.ranges[540]
    left = data.ranges[900]
    right = data.ranges[180]

    # 22.5 Degree Angles
    narrowRange = [data.ranges[630], data.ranges[450]]
    # 45 Degree Angles
    wideRange = [data.ranges[720], data.ranges[360]]

    # Distance of the Hypotenuse
    narrowHypotenuse = ((narrowRange[0] * narrowRange[0]) + (narrowRange[1] * narrowRange[1]) - (narrowRange[0] * narrowRange[1] * 1.4142)) ** 0.5
    wideHypotenuse = ((wideRange[0] * wideRange[0]) + (wideRange[1] * wideRange[1])) ** 0.5

    print narrowHypotenuse, wideHypotenuse
    #print data


def get_laser_data():  
    rate = rp.Rate(1)#Hz 
    while not rp.is_shutdown():
        #rp.Subscriber('/scan',LaserScan,callback)
        #pub = rp.Publisher('lidar_instructions',AckermannDriveStamped,queue_size=1)
 
        rate.sleep()
    

if __name__=="__main__":
    try:
        rp.init_node('Lidar',anonymous = True)
        get_laser_data()
    except rp.ROSInterruptException:
        pass

