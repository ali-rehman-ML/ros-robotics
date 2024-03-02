#!/usr/bin/env python

import rospy

from lab_1.msg import complex_number


def pub():
    pub = rospy.Publisher('complex_number', complex_number,queue_size=10)
    rospy.init_node('complex_number_publisher')
    r = rospy.Rate(10) #10hz
    msg = complex_number()
    msg.real = float(rospy.get_param('~real'))
    msg.imag = float(rospy.get_param('~imag'))

    while not rospy.is_shutdown():
#        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        pub()
    except rospy.ROSInterruptException: pass

