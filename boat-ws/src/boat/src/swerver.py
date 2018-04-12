#!/usr/bin/env python
import cv2
import rospy as rp
import numpy as np
import operator
from math import sqrt
from cv_bridge import CvBridge, CvBridgeError
from car_utils import timer,pid_steering,deg,clamp,orbiter,deg,imgproc
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from sensor_msgs.msg import Image,LaserScan

yellow = [([10,235,80],[30,255,255])]
orange = [([0,180,15],[10,255,255])]
red = [([0,250,60],[0,255,255]),([175,180,60],[180,255,255])]

bridge = CvBridge()

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

cones = {}

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

zed = imgproc()


h,w = 376,1344
zed.set_ROI([(0,h*1/4),(w,h*1/4),(w,h),(0,h)]) # bottom 3/4 of ZED image

def callback(data):
    try:
        global cones
        zed.set_display(rp.get_param('~display'))
        zed.set(bridge.imgmsg_to_cv2(data, "bgr8"))
        cones = zed.get_shapes([yellow]) # First range in list will be considered the cone color later

    except Exception as e:
        print e

def proc_lidar(data):
    accuracy = 4    # Degrees
    diameter = 0.15 # Diameter of cone
    min_diff = 0.2  # These 2 values are for determining cone-ness
    max_diff = 0.5
    if len(cones) > 0:
        distances = {}
        for x in cones[0]:
            distances[540 -(x*4)] = data.ranges[540 -(x*4)]
        angle = min(distances,key=distances.get)  # Find the lidar-angle to the nearest detected contour
        dist = data.ranges[angle]   # Distance to nearest detected
        size = int(round(deg(np.arctan((2*diameter)/data.ranges[angle]))))  # Estimated size (angular range) of detected given distance and known diameter (Don't ask me why it has to be doubled to be accurate)
        is_cone = True
        reason = "N/A"
        for y in range(angle,angle+(size*4)+(10*4)):    # Scan from nearest angle (not necessarily edge) to 1 full "size" counter-clockwise of it
            if y <= angle+size - (size*0.4) and abs(data.ranges[y]-dist) > max_diff:    # If scanned distance at short range from contour centroid is too different
                reason = "Too small"
                is_cone = False
            elif y >= angle + size*4 + (size*0.4) and  not abs(data.ranges[y]-dist) > min_diff: # If scanned distance at long range from contour centroid isn't different enough
                reason = "Too big"
                is_cone = False
        if is_cone:
            print "Nearest object (at %d degrees, %.2f meters) is a cone!"%(int(round((540-angle)/4)),dist)
        else:
            print "Nearest bject (at %d degrees, %.2f meters) is NOT a cone! %s!"%(int(round((540-angle)/4)),dist,reason)

def cing_cones():
    rp.Subscriber('zed_images',Image,callback,queue_size=1)
    rp.Subscriber('/scan',LaserScan,proc_lidar)
    rp.spin()

if __name__=="__main__":
    try:
        rp.init_node('Swerver',anonymous = True)
        cing_cones()
    except rp.ROSInterruptException:
        pass
