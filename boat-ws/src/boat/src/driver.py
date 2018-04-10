#!/usr/bin/env python
"""
Author: Will Irwin
Last Modified: 2/22/2018
Title:driver.py
Inputs:

Outputs:

"""
import rospy as rp
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
class drive:
    def __init__(self):
        self.pub=rp.Publisher("/vesc/ackermann_cmd_mux/input/navigation",AckermannDriveStamped,queue_size=10)#i think this needs to change
        self.priorities={}

    def full_stop(self,data):
    	priority = 0
	self.priorities[priority] = data
    def zed_instructions(self,data):
        priority = 3
        self.priorities[priority] = data

    def avoider_instructions(self,data):
        priority = 1
        self.priorities[priority] = data

    def move(self,data):
        try:
            self.pub.publish(data)
        except Exception as e:
            print e

    def prioritize(self):
        if len(self.priorities.keys()) > 0:
            priority = min(self.priorities.keys())
            self.move(self.priorities[priority])
            if rp.get_param('show_motor'):
                print "Priority %d motor instruction"%priority
            self.priorities.pop(priority,None)


motor = drive()

def driver():
    rp.Subscriber('avoider_instructions',AckermannDriveStamped,motor.avoider_instructions)
    rp.Subscriber('movement_instructions',AckermannDriveStamped,motor.zed_instructions)
    #rp.Subscriber('swerver_instructions',AckermannDriveStamped,motor.swerver_instructions)
    rp.Subscriber('full_stop',AckermannDriveStamped,motor.full_stop)
    while True: motor.prioritize()


if __name__=="__main__":
    try:
        rp.init_node('Driver',anonymous = True)
        driver()
    except rp.ROSInterruptException:
        pass
