#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def string_publisher():
    pub = rospy.Publisher('string_topic', String, queue_size=10)
    rospy.init_node('string_publisher', anonymous=True)

    name = rospy.get_param('~Name')  # Retrieve the argument value

    rate = rospy.Rate(10)  # Publish at 10 Hz
    while not rospy.is_shutdown():
        msg = String()
        msg.data = name
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        string_publisher()
    except rospy.ROSInterruptException:
        pass

