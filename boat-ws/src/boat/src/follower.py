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
from car_utils import imgproc,timer,pid_steering

clock = timer()
clock.set_scale(-6)
clock.set_name("Follower")

red = [([0,100,100],[7,255,255]),([172,100,100],[179,255,255])]
blue = ([100,80,60],[131,255,255])
green = ([41,100,100],[75,255,255])

move_pub = rp.Publisher('movement_instructions',AckermannDriveStamped,queue_size=10)


# ------ Helper Functions -----

bridge = CvBridge()
drive_msg_stamped = AckermannDriveStamped()
drive_msg = AckermannDrive()
zed = imgproc()

clock.get_time("Initialization")

h,w = 376,1344
zed.set_ROI([((w*0/8),(h*3/3)),((w*0/8),(h*2/3)),((w*1/3),(h*1/2)),((w*1/2),(h*2/3)),((w*2/3),(h*1/2)),((w*8/8),(h*2/3)),((w*8/8),(h*3/3))])

clock.get_time("Set ROI")

steering = pid_steering()

def callback(data):
    try:
        zed.set_display(rp.get_param('~display'))
        zed.set(bridge.imgmsg_to_cv2(data, "bgr8"))

        clock.get_time("Pull ZED feed")

        impulse = zed.get_impulse(5,blue)

        clock.get_time("Get impulse")

        if impulse != None:
            impulse/=180
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

    except CvBridgeError as e:
        print(e)

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
