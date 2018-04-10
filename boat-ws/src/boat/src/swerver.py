#!/usr/bin/env python
import cv2
import rospy as rp
import numpy as np
import operator
from math import sqrt
from car_utils import timer,pid_steering,deg,clamp,orbiter,deg
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
    global previous_dir
    global current_dir
    global future_dir
    keys = np.array(lidar_cones.keys())

    current_dir = previous_dir
    orbit.set_dir(current_dir)
    orbit.set_data(lidar_cones)

    if keys[(890<keys) & (910>keys)].size > 0 and previous_dir != "left":   #left whisker hit and the current direction your turning is not already left
        orbit.set_dir("left")
        bookKeeping()
    elif  keys[(170<keys) & (190>keys)].size > 0 and previous_dir != "right":
        orbit.set_dir("right")
        bookKeeping()

    if current_dir == "left":
        orbit.orbit_circular()
    elif current_dir == "straight":
        straight()
    elif cone_counter == 0:
        pass#park()

    if keys[540<keys].size > keys[540>keys].size:   #there are more outliers on the left than right
        future_dir = "left" #this is probably where you will be turning
    else:
        future_dir = "right"



    previous_dir = current_dir



def bookKeeping():
    global cone_counter
    global orbit_start
    global orbit_end
    global orbit_durations
    global returning
    if cone_counter != 0:
        orbit_end = cv2.getTickCount()
        orbit_durations.append(orbit_end - orbit_start)
        avg = np.mean(np.array(orbit_durations))
        if orbit_end - orbit_start > 2*avg:
            cone_counter-=1
            returning = True
        orbit_start = cv2.getTickCount()
        if not returning:
            cone_counter+=1
        else:
            cone_counter-=1
    else:
        orbit_start = cv2.getTickCount()
        if not returning:
            cone_counter+=1
        else:
            cone_counter-=1

# def proc_zed():

def proc_lidar(data):
    accuracy = 4
    rays = {}
    right = 8
    left = 1080 - right
    diameter = 0.15
    max_dist = 1 #meters
    min_diff = 0.075
    for x in range(right,left,accuracy):
        if data.ranges[x] <= max_dist:
            rays[x] = data.ranges[x]
    cones = 0
    groups = {}
    keys = sorted(rays.keys())
    if len(keys)>0:
        last_x = keys[0]
        for x in keys:
            size = int(round(deg(np.arctan((2*diameter)/rays[x]))))
            is_cone = True
            if x == last_x or x > last_x+size:
                last_x = x
                group=[x]
                for y in range(x+accuracy,x+accuracy+(int(round((size*0.9)))),accuracy):
                    if y not in rays or abs(rays[x]-rays[y]) > min_diff:
                        is_cone = False
                    else:
                        group+=[y]
            else:
                is_cone = False
            if is_cone:
                cones += 1
                groups[cones] = group
    for x in groups:
        print "Cone: %d, Index: %d"%(x,int(round(sum(groups[x])/len(groups[x]))))

    #swerve(rays, data)

def find_outliers(data, m=1):
    mean = np.mean(data)
    std = np.std(data)
    data[abs(data - mean) < m * std] = 0
    data[data > mean] = 0
    return data

def get_laser_data():
    # rp.Subscriber('zed_images',Image,proc_zed,queue_size=1)
    rp.Subscriber('/scan',LaserScan,proc_lidar)
    rp.spin()

if __name__=="__main__":
    try:
        rp.init_node('Swerver',anonymous = True)
        get_laser_data()
    except rp.ROSInterruptException:
        pass
