"""
Author: Will Irwin
Last Modified: 2/12/2018
Title:crier.py
Inputs:

Outputs:

"""

import rospy as rp
from random import randint
from stdmsgs.msg import String

if __name__=="__main__":
    try:
        poly_talker()
    except rp.ROSInterruptException:
        pass

def poly_talker():
    pub = rp.Publisher('polycrier',String,queue_size = 10)
    rp.init_node('Crier',anonymous = True)
    rate = rp.rate(21)#21Hz

    while not rp.is_shutdown():
        num = random.randint(0,3)
        name = ["Will","Brett","Henrique","David"]
        output = name[num]
        rp.loginfo(output)
        pub.publish(output)
        rate.sleep()
