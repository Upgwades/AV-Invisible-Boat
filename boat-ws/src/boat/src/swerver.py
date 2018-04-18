#!/usr/bin/env python
import cv2
import rospy as rp
import numpy as np
import operator
from math import sqrt
from cv_bridge import CvBridge, CvBridgeError
from car_utils import timer,pid_steering,rad,deg,clamp,orbiter,deg,imgproc
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

orbital_steering = pid_steering()
orbital_steering.set_pid(kp=0.75,ki=0,kd=0)
origin = 0


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
        zed.set_display(rp.get_param('~display'))
        zed.set(bridge.imgmsg_to_cv2(data, "bgr8"))

    except Exception as e:
        print e

step_count = 0
right = 160
left = 940
Sides = 30

def get_cones(data,objects):
    accuracy = 4    # Degrees
    diameter = 0.15 # Diameter of cone
    min_diff = 0.00  # These 2 values are for determining cone-ness
    max_diff = 0.05
    distances = {}
    for x in objects:
        distances[540 -(x*4)] = data.ranges[540 -(x*4)]
    angles = sorted(distances, key=distances.get)
    verified = []
    for angle in angles:
        dist = data.ranges[angle]   # Distance to nearest detected
        size = int(round(deg(np.arctan((2*diameter)/data.ranges[angle]))))  # Estimated size (angular range) of detected given distance and known diameter (Don't ask me why it has to be doubled to be accurate)
        small_size = angle + size/3
        big_size = angle + size*2
        is_cone = True
        reason = "N/A"
        for y in range(angle,angle+(size*4)+(10*4)):    # Scan from nearest angle (not necessarily edge) to 1 full "size" counter-clockwise of it
            if y <= small_size and abs(data.ranges[y]-dist) >= max_diff:    # If scanned distance at short range from contour centroid is too different
                #reason = "Too small! Checked Angle: %d (%d), Lidar: %.2f, Difference: %.2f"%((540-small_size)/4, small_size,data.ranges[y], abs(data.ranges[y]-dist))
                is_cone = False
            elif y >= big_size and  not abs(data.ranges[y]-dist) >= min_diff: # If scanned distance at long range from contour centroid isn't different enough
                #reason = "Too big! Checked Angle: %d (%d), Lidar: %.2f, Difference: %.2f"%((540-big_size)/4, big_size,data.ranges[y], abs(data.ranges[y]-dist))
                is_cone = False
        if is_cone:
            verified += [angle]
            #print "Nearest object (at %d degrees (%d), %.2f meters, %d (%d) degrees wide) is a cone!"%(int(round((540-angle)/4)),angle,dist,size, size*4)
        else:
            pass#print "Nearest object (at %d degrees (%d), %.2f meters, %d (%d) degrees wide) is NOT a cone! %s!"%(int(round((540-angle)/4)),angle,dist,size,size*4,reason)
    return verified

def get_blocks(data,objects):
    items = []
    for x in objects:
        items += [540-(x*4)]
    return items

def proc_lidar(data,color):
    objects = zed.get_shapes(color) # First range in list will be considered the cone color later
    cones,blocks = [],[]
    if 0 in objects:
        cones = get_cones(data,objects[0])
    if 1 in objects:
        blocks = get_blocks(data,objects[1])
    return cones,blocks

tolerance = 0
aligned = False
slice
direction = ""
orb_count = 1
def swerve(data):
    global orb_count
    global step_count
    global tolerance
    global aligned
    global slice
    global origin
    global direction
    global previous_dir
    R_Slice = data.ranges[right:(right)+(Sides*4):4]
    L_Slice = data.ranges[(left)-(Sides*4):left:4]
    objects = {}
    while zed.image.shape[:2]!=(376,1344):
        pass
    objects["Cones"],objects["Blocks"] = proc_lidar(data,[yellow,orange])
    diameter = 0.15
    if len(objects["Blocks"])+len(objects["Cones"]) > 0:
        if len(objects["Cones"]) > 0 and len(objects["Blocks"]) > 0:
            if data.ranges[objects["Blocks"][0]] < data.ranges[objects["Cones"][0]]:
                nearest = "Blocks"
            else:
                nearest = "Cones"
        elif len(objects["Cones"]) > 0:
            nearest = "Cones"
        else:
            nearest = "Blocks"
        #print step_count, nearest, objects
        if step_count == 0 and len(objects["Cones"]) > 0:
            if nearest == "Blocks":
                pass
            else:
                if objects["Cones"][0] < 540:
                    direction = "Right"
                else:
                    direction = "Left"
                tolerance = abs(np.sin(rad((540-objects["Cones"][0])/4))*data.ranges[objects["Cones"][0]])
                step_count += 1
    if step_count == 1:
        if direction == "Left":
            slice = L_Slice
        else:
            slice = R_Slice
        if not aligned:
            steering.add_error(0)
            drive_msg.speed,drive_msg.steering_angle,drive_msg.acceleration,drive_msg.jerk,drive_msg.steering_angle_velocity = steering.steer(0)
            drive_msg_stamped.drive = drive_msg
            move_pub.publish(drive_msg_stamped)
            count = 0
            print tolerance
            for x in slice:
                if abs(x-tolerance) > 0.2:
                    count += 1
            if count < 30:
                aligned = True
                step_count +=1
                drive_msg.speed,drive_msg.steering_angle,drive_msg.acceleration,drive_msg.jerk,drive_msg.steering_angle_velocity = 0,0,0,0,0
                drive_msg_stamped.drive = drive_msg
                stop_pub.publish(drive_msg_stamped)
    elif step_count == 2:
        print "ORBIT",direction
        previous_dir = direction
        if orb_count%175 == 0:
            pass
        orbit(direction)
        previous_dir = direction
        if direction == "Left":
            slice = data.ranges[right:(right)+(Sides*4):4]
        else:
            slice = data.ranges[(left)-(Sides*4):left:4]
        count = 0
        for x in slice:
            if abs(x-1.5) > 0.2:
                count += 1
        if count < 30:
            if direction == "Left":
                direction = "Right"
            else:
                direction = "Left"

def orbit(direction):
    global previous_dir
    if direction != previous_dir:
        orbital_steering.wipe_errors()
    if direction == "Left":
        go = -1
    else:
        go = 1
    orbital_steering.add_error(go)
    drive_msg.speed,drive_msg.steering_angle,drive_msg.acceleration,drive_msg.jerk,drive_msg.steering_angle_velocity = orbital_steering.steer(go)
    drive_msg_stamped.drive = drive_msg
    move_pub.publish(drive_msg_stamped)


def cing_cones():
    rp.Subscriber('zed_images',Image,callback,queue_size=1)
    rp.Subscriber('/scan',LaserScan,swerve)
    rp.spin()

if __name__=="__main__":
    try:
        rp.init_node('Swerver',anonymous = True)
        cing_cones()
    except rp.ROSInterruptException:
        pass
