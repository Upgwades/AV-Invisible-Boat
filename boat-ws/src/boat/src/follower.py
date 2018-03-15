#!/usr/bin/env python
'''
Author(s):  Invisible Boat Team

'''
import rospy as rp
import cv2
from cv_bridge import CvBridge, CvBridgeError
from operator import itemgetter
from sensor_msgs.msg import Image
from math import pi,sqrt
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from car_utils import imgproc,timer

clock = timer()
clock.set_scale(-6)
clock.set_name("Follower")

red = [([0,100,100],[7,255,255]),([172,100,100],[179,255,255])]
blue = ([100,80,60],[131,255,255])
green = ([41,100,100],[75,255,255])

move_pub = rp.Publisher('movement_instructions',AckermannDriveStamped,queue_size=10)


# ------ Helper Functions -----

def rad(degree):
	return (degree*pi)/180

old_errors = [0]
def steeringControl(error):
    kp = 0.1
    ki = 0.0
    kd = 0.0
    
    proportional = error
    integral = sum(old_errors)
    derivative = old_errors[-2]+error

    output = ((kp * proportional) + (ki*integral) + (kd*derivative))
    if(output<-1):
        output = -1
    if(output>1):
        output = 1

    return -output

bridge = CvBridge()
drive_msg_stamped = AckermannDriveStamped()
drive_msg = AckermannDrive()
zed = imgproc()

clock.get_time("Initialization")

h,w = 376,1344
zed.set_ROI([((w*0/8),(h*3/3)),((w*0/8),(h*2/3)),((w*1/3),(h*1/2)),((w*1/2),(h*2/3)),((w*2/3),(h*1/2)),((w*8/8),(h*2/3)),((w*8/8),(h*3/3))])

clock.get_time("Set ROI")

def callback(data):
    global old_errors
    try:
        zed.set_display(rp.get_param('~display'))
        zed.set(bridge.imgmsg_to_cv2(data, "bgr8"))

        clock.get_time("Pull ZED feed")

        impulse = zed.get_impulse(4,13,blue)

        clock.get_time("Get impulse")

        if impulse != None:
            impulse/=(180)
            old_errors += [impulse]
            if len(old_errors) > 100:
                old_errors = old_errors[1:]
            drive_msg.speed,drive_msg.steering_angle,drive_msg.acceleration,drive_msg.jerk,drive_msg.steering_angle_velocity = follow_line(impulse)

            clock.get_time("Drive instructions")

            drive_msg_stamped.drive = drive_msg
            move_pub.publish(drive_msg_stamped)

            clock.get_time("Publish drive instructions")

            clock.get_all_avg()

    except CvBridgeError as e:
        print(e)

def follow_line(impulse):
    try:
        steering_angle = steeringControl(impulse)
        speed = ((-1*sqrt(abs(0.3*steering_angle)))+1)-0.15
        acceleration = ((0.273861*steering_angle)/(abs(steering_angle)**(3/2)))
        jerk = ((0.136931*(steering_angle**2))/(abs(steering_angle)**(7/2)))
        steering_angle_velocity = 0
	    #steering_angle_velocity = ((-1*sqrt(abs(0.3*steering_angle)))+1)-0.15
        #print "Steering: %.2f, Speed: %.2f, Acceleration: %.2f, Jerk: %.2f, Angle: %.2f"%(steering_angle,speed,acceleration,jerk,steering_angle_velocity)
        return speed,steering_angle,acceleration,jerk,steering_angle_velocity
    except:
        print("not implemented yet")

def listener():
    rp.Subscriber('zed_images',Image,callback,queue_size=1)
    rp.spin()


if __name__=="__main__":
    try:
        rp.init_node('Follower',anonymous = True)
        listener()
    except rp.ROSInterruptException:
        pass
    cv2.destroyAllWindows()
