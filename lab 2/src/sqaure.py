#!/usr/bin/env python

import rospy
import turtlesim.srv
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
import time
import copy
import math
class turtle_circle:
    def __init__(self):
                # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlesim_circle', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                                  Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',
                                                Pose, self.update_pose)

       

        self.pose = Pose()
        self.rate = rospy.Rate(10)
        self.angle=0


    def update_pose(self,data):
        self.pose=data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        self.angle=math.degrees(self.pose.theta)
        if (self.angle<0):
            self.angle=self.angle+360
        # print(self.angle)


    def straight_line(self,distance):
        t0 = rospy.Time.now().to_sec()
        current_distance = 0
        vel_msg = Twist()
        #Loop to move the turtle in an specified distance
        while(current_distance < distance):
            vel_msg.linear.x=1
            vel_msg.linear.y=0
            vel_msg.linear.z=0
            vel_msg.angular.x=0
            vel_msg.angular.y=0
            vel_msg.angular.z=0
            #Publish the velocity
            self.velocity_publisher.publish(vel_msg)
            #Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            #Calculates distancePoseStamped
            current_distance= 1*(t1-t0)
        #After the loop, stops the robot
        vel_msg.linear.x = 0
        #Force the robot to stop
        self.velocity_publisher.publish(vel_msg)


    def turn(self,theta):

        angular_speed=(10)*(math.pi/180)
        vel_msg=Twist()
        print(self.angle)
        final_angle=self.angle+(theta)
        if final_angle>360:
            final_angle=final_angle-360
        final_angle=math.ceil(final_angle)
        print("final angle",final_angle,"angle",self.angle)
        while(abs(self.angle-final_angle)>0.3):
            # print(self.angle)
            # print(math.degrees(abs(self.angle-final_angle)))
            
            vel_msg.linear.x=0
            vel_msg.linear.y=0
            vel_msg.linear.z=0
            vel_msg.angular.x=0
            vel_msg.angular.y=0
            vel_msg.angular.z=angular_speed
            self.velocity_publisher.publish(vel_msg)

        vel_msg.angular.z=0
        self.velocity_publisher.publish(vel_msg)



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

    def square_move(self):
        while(1):
            for i in range(12):
                self.straight_line(1)
                self.turn(30)
    



if __name__ == '__main__':
    try:
        x = turtle_circle()
        x.square_move()
    except rospy.ROSInterruptException:
        pass


        

    



    
