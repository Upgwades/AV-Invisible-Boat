#!/usr/bin/env python
import cv2
import rospy as rp
import numpy as np
import operator
from math import sqrt
from car_utils import timer,pid_steering,deg,clamp
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from sensor_msgs.msg import Image,LaserScan

move_pub = rp.Publisher('avoider_instructions',AckermannDriveStamped,queue_size=10)
stop_pub = rp.Publisher('full_stop',AckermannDriveStamped,queue_size=10)
drive_msg_stamped = AckermannDriveStamped()
drive_msg = AckermannDrive()

clock = timer()
clock.set_scale(-6)
clock.set_name("Avoider")

steering = pid_steering()
steering.set_pid(kp=0.75,ki=0,kd=0)

def callback(data):
    straight = 540
    left = 900
    right = 180
    max_width = 10
    max_width /= 2
    rays = {straight:data.ranges[straight],left:data.ranges[left],right:data.ranges[right]}
    #print data.ranges[left],data.ranges[right]

    if rays[left] < max_width and rays[right] < max_width:
        accuracy = 10  # 4 = 1 degree

        clock.update()

        for x in range(right,left,accuracy):
            if data.ranges[x] <= 10:
                rays[x] = data.ranges[x]
        clock.get_time("Lidar trimming")

        settled = False
        count = 0
	Front = 130
	Sides = int(round((270-Front)/4))
        R_Slice = data.ranges[:right+(Sides*4):accuracy]
        S_Slice = data.ranges[right+(Sides*4):left-(Sides*4):accuracy]
        L_Slice = data.ranges[left-(Sides*4)::accuracy]
        R_avg = sum(R_Slice)/len(R_Slice)
        L_avg = sum(L_Slice)/len(L_Slice)
        S_avg = sum(S_Slice)/len(S_Slice)
        if S_avg < 0.4:
            drive_msg.speed,drive_msg.steering_angle,drive_msg.acceleration,drive_msg.jerk,drive_msg.steering_angle_velocity = 0,0,0,0,0
	    drive_msg_stamped.drive = drive_msg
            stop_pub.publish(drive_msg_stamped)
        else:
            longest = (right+(45*4))+(S_Slice.index(max(S_Slice))*accuracy)
            center = R_avg - L_avg
            center/=max_width
            theta = 540 - longest
            theta/=(4.0*45)
            impulse = ((2*center)+(0.01*theta))/2
            impulse *= 10
            impulse = clamp(impulse,-0.999,0.999)
            #print center,theta,impulse
            if -1 < impulse < 1:
                steering.add_error(impulse)
                try:
                    drive_msg.speed,drive_msg.steering_angle,drive_msg.acceleration,drive_msg.jerk,drive_msg.steering_angle_velocity = steering.steer(impulse)

                    clock.get_time("Drive instructions")

                    drive_msg_stamped.drive = drive_msg
                    move_pub.publish(drive_msg_stamped)

                    clock.get_time("Publish drive instructions")

                    clock.get_all_avg()
                except Exception as e:
                    print e
            clock.get_time("Settling")
            clock.get_all_avg()

def get_laser_data():
    #rate = rp.Rate(1)#Hz
    #while not rp.is_shutdown():
    rp.Subscriber('/scan',LaserScan,callback)
    rp.spin()


if __name__=="__main__":
    try:
        rp.init_node('Avoider',anonymous = True)
        get_laser_data()
    except rp.ROSInterruptException:
        pass
