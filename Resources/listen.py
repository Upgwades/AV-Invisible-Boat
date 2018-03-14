"""
Author: Will Irwin
Last Modified: 2/12/2018
Title:listen.py
Inputs:

Outputs:

"""
import rospy as rp
from stdmsgs.msg import String

def callback(data):
    rp.loginfo(rp.get_caller_id())+" I heard %s", data.data)


def listener():
    rp.init_node('polylistener'Anonymous = True)
    rp.Subscriber('polycrier',String,callback)
    rp.spin()

if __name__=="__main__":
    try:
        listener()
    except rp.ROSInterruptException:
        pass
