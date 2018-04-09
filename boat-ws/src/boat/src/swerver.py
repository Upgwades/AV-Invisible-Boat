#!/usr/bin/env python
import cv2
import rospy as rp
import numpy as np
import operator
from math import sqrt
from car_utils import timer,pid_steering,deg,clamp
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from sensor_msgs.msg import Image,LaserScan

move_pub = rp.Publisher('swerver_instructions',AckermannDriveStamped,queue_size=10)
stop_pub = rp.Publisher('full_stop',AckermannDriveStamped,queue_size=10)
drive_msg_stamped = AckermannDriveStamped()
drive_msg = AckermannDrive()

clock = timer()
clock.set_scale(-6)
clock.set_name("Swerver")

orbit = orbiter()
cone_counter = 0
orbit_start = cv2.getTickCount()
orbit_end = cv2.getTickCount()
orbit_durations = []
returning = False

previous_dir = ""
current_dir = ""
future_dir = ""



steering = pid_steering()
steering.set_pid(kp=0.75,ki=0,kd=0)

def swerve(lidar_cones,lidar_raw):
    keys = np.array(lidar_cones.keys())

    current_dir = previous_dir
    orbit.set_dir(current_dir)
    orbit.set_data(lidar_cones)

    if keys[(890<keys) && (910>keys)].size > 0 && previous_dir != "left":   #left whisker hit and the current direction your turning is not already left
        orbit.set_dir("left")
        bookKeeping()
    elif  keys[(170<keys) && (190>keys)].size > 0 && previous_dir != "right":
        orbit.set_dir("right")
        bookKeeping()

    if current_dir == "left":
        orbit.orbit_circular()
    elif current_dir == "straight":
        straight()
    elif cone_counter == 0:
        park()

    if keys[540<keys].size > keys[540>keys].size:   #there are more outliers on the left than right
        future_dir = "left" #this is probably where you will be turning
    else:
        future_dir = "right"



    previous_dir = current_dir



def bookKeeping():
    if cone_counter != 0:
        orbit_end = cv2.getTickCount()
        orbit_duration.append(orbit_end - orbit_begin)
        avg = np.mean(np.array(orbit_durations))
        if orbit_end - orbit_begin > 2*avg:
            cone_counter--
            returning = True
        orbit_start = cv2.getTickCount()
        if !returning:
            cone_counter++
        else:
            cone_counter--
    else:
        orbit_start = cv2.getTickCount()
        if !returning:
            cone_counter++
        else:
            cone_counter--

# def proc_zed():

def proc_lidar(data):
    data = np.array(data.ranges)
    cone_dict = {}
    outliers = find_outliers()
    for outlier in outliers:
        if outlier != 0:
            idx = np.where(outliers == outlier)
            cone_dict.update({idx: outlier})

    return cone_dict, data

def find_outliers(data, m=6):
    data[abs(data - np.mean(data)) < m * np.std(data)] = 0
    return data

def straight():

def orbit_left():

def orbit_right():

def get_laser_data():
    # rp.Subscriber('zed_images',Image,proc_zed,queue_size=1)
    lidar_cones,lidar_raw = rp.Subscriber('/scan',LaserScan,proc_lidar)
    swerve(lidar_cones,lidar_raw)
    rp.spin()

if __name__=="__main__":
    try:
        rp.init_node('Swerver',anonymous = True)
        get_laser_data()
    except rp.ROSInterruptException:
        pass
