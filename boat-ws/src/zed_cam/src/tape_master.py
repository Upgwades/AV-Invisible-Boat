#!/usr/bin/env python

import rospy
from pid import *
from std_msgs.msg import Bool, Float32
from ackermann_msgs.msg import AckermannDriveStamped
from smach import State, StateMachine

rate = None
pub = None
is_tape = False
center_tape = 0.0
ackermann_msg = AckermannDriveStamped()
forward_speed = 1.0
controller = PID(1.0, 0.1, 0.1, 1.0, -1.0)


def is_tape_callback(msg):
    global is_tape
    is_tape = msg.data


def center_tape_callback(msg):
    global center_tape
    center_tape = msg.data


class decision(State):
    def __init__(self):
        State.__init__(self, outcomes=['decide_tape',
                                       'no_tape',
                                       'follow_tape'])


    def execute(self, userdata):
        global is_tape

        if rospy.is_shutdown():  # Allow the program to exit
            exit()

        if is_tape:
            return 'follow_tape'
        else:
            return 'no_tape'

        return 'decide_tape'

class stop(State):
    def __init__(self):
        State.__init__(self, outcomes=['decide_tape'])


    def execute(self, userdata):
        global ackermann_msg
        global pub

        if rospy.is_shutdown():  # Allow the program to exit
            exit()
        rate.sleep()

        ackermann_msg.drive.speed = 0.0
        pub.publish(ackermann_msg)
        return 'decide_tape'


class follow(State):
    def __init__(self):
        State.__init__(self, outcomes=['decide_tape'])


    def execute(self, userdata):
        global rate
        global forward_speed
        global center_tape
        global ackermann_msg
        global pub

        if rospy.is_shutdown():  # Allow the program to exit
            exit()
        rate.sleep()

        ackermann_msg.drive.speed = forward_speed
        controller.update_PID(-center_tape)
        ackermann_msg.drive.steering_angle = controller.cmd + 0.12

        pub.publish(ackermann_msg)
        return 'decide_tape'


def main():
    global rate
    global is_tape
    global center_tape
    global pub
    rospy.init_node('tape_master')
    rate = rospy.Rate(20)
    is_tape_sub = rospy.Subscriber('tape_detect_msg', Bool, is_tape_callback)
    center_tape_sub = rospy.Subscriber('tape_center_percent_msg', Float32, center_tape_callback)

    pub = rospy.Publisher('vesc/ackermann_cmd_mux/input/navigation',
                                    AckermannDriveStamped, queue_size=1)

    tape = StateMachine(outcomes = ['decide_tape',
                                    'no_tape',
                                    'follow_tape'])

    with tape:
        StateMachine.add('DECIDE', decision(), transitions={
            'decide_tape': 'DECIDE',
            'no_tape': 'STOP',
            'follow_tape': 'FOLLOW'
        })

        StateMachine.add('STOP', stop(), transitions={'decide_tape': 'DECIDE'})

        StateMachine.add('FOLLOW', follow(), transitions={'decide_tape': 'DECIDE'})

    tape.execute()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
