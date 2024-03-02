#!/usr/bin/env python

import rospy
import math
from lab_1.msg import complex_number

def callback(data):
    a=data.real
    b=data.imag
    
    mag=math.sqrt(a**2+b**2)
    phase=math.atan2(b,a)*(180/math.pi)
    rospy.loginfo("Magnitude : %f",mag)
    rospy.loginfo("Phase (deg) : %f", phase)
    
def sub():

    rospy.init_node('complex_number_subscriber')
    rospy.Subscriber("/complex_number", complex_number, callback)
    rospy.spin()




if __name__ == '__main__':
    try:
        sub()
    except rospy.ROSInterruptException: pass

