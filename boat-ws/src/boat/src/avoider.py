#!/usr/bin/env python
import cv2
import rospy as rp
import numpy as np
import operator
from math import sqrt
from car_utils import timer,pid_steering,deg
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from sensor_msgs.msg import Image,LaserScan

move_pub = rp.Publisher('lidar_instructions',AckermannDriveStamped,queue_size=10)
drive_msg_stamped = AckermannDriveStamped()
drive_msg = AckermannDrive()

clock = timer()
clock.set_scale(-6)
clock.set_name("Avoider")

steering = pid_steering()
steering.set_pid(kp=0.85,ki=0,kd=0)

def callback(data):
    straight = 540
    left = 900
    right = 180
    rays = {straight:data.ranges[straight],left:data.ranges[left],right:data.ranges[right]}
    #print data.ranges[left],data.ranges[right]

    if rays[left] < 5 and rays[right] < 5:
        accuracy = 1  # 4 = 1 degree

        clock.update()

        for x in range(right,left,accuracy):
            if data.ranges[x] <= 10:
                rays[x] = data.ranges[x]
        clock.get_time("Lidar trimming")

        settled = False

        while not settled and bool(rays):
            index = max(rays.iteritems(), key=operator.itemgetter(1))[0]
            theta = (90 - (index - 180.0)/4)
            longest = rays[index]
            side_w = 0.2
            car_l = 0.5
            h = sqrt((side_w**2) + (car_l**2))
            LorR = -1
            if theta < 0:
                LorR = 1
            wheel = int(round(index+(LorR*((90-deg(np.arctan((1*car_l)/side_w)))*4))))
            #print data.ranges[wheel],h
            if data.ranges[wheel] > h:
                settled = True
		print theta
                theta/=90
                steering.add_error(theta)
                try:
                    drive_msg.speed,drive_msg.steering_angle,drive_msg.acceleration,drive_msg.jerk,drive_msg.steering_angle_velocity = steering.steer(theta)

                    print drive_msg.steering_angle
                    clock.get_time("Drive instructions")

                    drive_msg_stamped.drive = drive_msg
                    move_pub.publish(drive_msg_stamped)

                    clock.get_time("Publish drive instructions")

                    clock.get_all_avg()
                except Exception as e:
                    pass
            else:
                rays.pop(index)
        
        clock.get_time("Settling")
        clock.get_all_avg()

def get_laser_data():  
    #rate = rp.Rate(1)#Hz 
    #while not rp.is_shutdown():
    rp.Subscriber('/scan',LaserScan,callback)
    rp.spin()
    

if __name__=="__main__":
    try:
        rp.init_node('Lidar',anonymous = True)
        get_laser_data()
    except rp.ROSInterruptException:
        pass

