#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
from math import pow, atan2, sqrt
from geometry_msgs.msg import Twist
import numpy as np
class my_turtlebot3:
    def __init__(self):
        self.x=0
        self.y=0
        self.theta=0
        rospy.init_node('turtlebot3_controller')
        self.goal_x=float(input("Set your x goal: "))
        self.goal_y=float(input("Set your y goal: "))
        self.distance_tolerance = input("Set your tolerance: ")
        self.velocity_publisher = rospy.Publisher('/cmd_vel',
                                                  Twist, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.call_back)

        
        self.rate = rospy.Rate(10)
    

    def call_back(self,data):
        self.x=round(data.pose.pose.position.x,4)
        self.y=round(data.pose.pose.position.y,4)
        
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion (orientation_list)
        
        yaw=math.degrees(yaw)

        self.theta=math.radians(yaw)
        

        
        
    def euclidean_distance(self):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((self.goal_x - self.x), 2) +
                    pow((self.goal_y - self.y), 2))

    def linear_vel(self, constant=0.25):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return min(0.25,0.25 * self.euclidean_distance())

    def steering_angle(self):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        goal_ang=atan2(self.goal_y - self.y, self.goal_x - self.x)

        return goal_ang

    def angular_vel(self, constant=0.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        ang=self.theta

        error=0.5 *(self.steering_angle()-ang)
        errors=np.clip(error,-0.5,0.5)
        

        
        return error

    def move2goal(self):
        """Moves the turtle to the goal."""

        # Please, insert a number slightly greater than 0 (e.g. 0.01).

        vel_msg = Twist()


        while self.euclidean_distance() >= self.distance_tolerance:

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel()
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel()

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

 

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()
        print("Reached at: x: ",self.x, "& y: ",self.y)



            
if __name__ == '__main__':
    robo=my_turtlebot3()
    time.sleep(1)
    robo.move2goal()
            
            
            
            
            
            
        
        
        
            
        
        
    
    


