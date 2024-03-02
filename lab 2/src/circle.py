#!/usr/bin/env python

import rospy
import turtlesim.srv
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
import time
import copy

class turtle_circle:
    def __init__(self):
                # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlesim_circle', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                                  Twist, queue_size=10)

       

        self.pose = Pose()
        self.rate = rospy.Rate(10)


    


    def circle_publisher(self):
        vel_msg=Twist()

        while not rospy.is_shutdown():
            
            vel_msg.linear.x = 2
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 1

            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # Angular velocity in the z-axis.
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        self.velocity_publisher.publish(vel_msg)



if __name__ == '__main__':
    try:
        x = turtle_circle()
        x.circle_publisher()
    except rospy.ROSInterruptException:
        pass


        

    



    
