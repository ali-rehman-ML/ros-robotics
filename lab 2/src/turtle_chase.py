#!/usr/bin/env python



import rospy
import turtlesim.srv
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
import time
import copy

import sys
import os
class Turtle_chaser:

    def __init__(self):
    

        rospy.init_node('turtle_chase')

        rospy.wait_for_service('/spawn')

        self.spawner = rospy.ServiceProxy('/spawn', turtlesim.srv.Spawn)

        turtle2_x = 1#random.uniform(-11, 11)
        turtle2_y = 1#random.uniform(-11, 11)
        turtle2 = self.spawner(turtle2_x, turtle2_y, 0, "turtle2")
        rospy.wait_for_service('/turtle2/set_pen')
        self.pen_setter = rospy.ServiceProxy('/turtle2/set_pen', turtlesim.srv.SetPen)
        self.pen_setter(255, 0, 0,2,0)  

        self.velocity_publisher = rospy.Publisher('/turtle2/cmd_vel',
                                                  Twist, queue_size=10)
        self.pose_subscriber_turtle_1 = rospy.Subscriber('/turtle1/pose',
                                                Pose, self.update_pose_turtle_1)
        self.pose_subscriber_turtle_2 = rospy.Subscriber('/turtle2/pose',
                                                Pose, self.update_pose_turtle_2)
        
        self.pose = Pose()
        self.goal_pose=Pose()
        self.rate = rospy.Rate(10)
        self.move_falg=False
        self.previou_pose=Pose()
        self.count=0
        self.linear_vel_constant=float(rospy.get_param('~linear_vel_constant'))
        self.angular_vel_constant=float(rospy.get_param('~angular_vel_constant'))

        print(self.linear_vel_constant,self.angular_vel_constant)



    def update_pose_turtle_1(self, data1):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        temp=copy.deepcopy(self.goal_pose)
        
        self.goal_pose = data1
        self.goal_pose.x = round(self.goal_pose.x, 4)
        self.goal_pose.y = round(self.goal_pose.y, 4)

        if self.count==0:
            self.previou_pose=self.goal_pose
            self.count=1
        else:
            self.previou_pose=temp


            

        if (not self.move_falg):

            d=sqrt(pow((self.goal_pose.x - 5.5), 2) +
                    pow((self.goal_pose.y - 5.5), 2))
            
            
            if d>0.1:
                self.move_falg=True


    def update_pose_turtle_2(self, data2):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data2
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)


        #print("turtle 1 ",self.pose.x,self.pose.y)

    def euclidean_distance(self):

        """Euclidean distance between current pose and the goal."""
        #print(self.goal_pose.x,self.goal_pose.y)
        return sqrt(pow((self.goal_pose.x - self.pose.x), 2) +
                    pow((self.goal_pose.y - self.pose.y), 2))
    
    def linear_vel(self, constant=0.25):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return  self.linear_vel_constant * self.euclidean_distance()
    
    def steering_angle(self):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(self.goal_pose.y - self.pose.y, self.goal_pose.x - self.pose.x)
    
    def angular_vel(self, constant=1):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return self.angular_vel_constant * (self.steering_angle() - self.pose.theta)
    


    def chase(self):
        """Moves the turtle to the goal."""


        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = 0.1

        vel_msg = Twist()
        #print("here ",self.euclidean_distance())

        while self.euclidean_distance() >= distance_tolerance:
            #print(self.linear_vel())

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
            if (self.move_falg):
                self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        os.system("rosnode kill /turtlesim_teleop")
        print("Your are busted !")

        # If we press control + C, the node will stop.
        


if __name__ == '__main__':
    try:

        x = Turtle_chaser()
        time.sleep(1)
        x.chase()
    except rospy.ROSInterruptException:
        pass




    

        
