#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
from math import pow, atan2, sqrt,sin,cos
from geometry_msgs.msg import Twist
import numpy as np
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import keyboard
class my_turtlebot3_map:
    def __init__(self):
        self.x=0
        self.y=0
        self.theta=0
        rospy.init_node('turtlebot3_controller')
        self.minimun_distance=float(input("Enter Minimum distance to maintain: "))
        self.velocity_publisher = rospy.Publisher('/cmd_vel',
                                                  Twist, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.call_back)

        rospy.Subscriber('/scan', LaserScan, self.scan_call_back)
        self.rate = rospy.Rate(25)
        
        self.scan_ranges=None
        
        
        
        
    def scan_call_back(self,msg):
        self.scan_ranges=msg.ranges
        
    

    def call_back(self,data):
        self.x=round(data.pose.pose.position.x,4)
        self.y=round(data.pose.pose.position.y,4)
        
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw_rad = euler_from_quaternion (orientation_list)
        
        yaw=math.degrees(yaw_rad)
        
        if yaw<0:
            yaw=yaw+360
            
        if yaw>360:
            yaw=yaw-360
            
        self.theta=math.radians(yaw)
        
    def move_minimum_distance(self):
        vel_msg = Twist()
        while(self.scan_ranges[0]>self.minimun_distance):
            print("here ",self.scan_ranges[0])
            
            vel_msg.linear.x = 0.1
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0
            

            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
            
        vel_msg.linear.x = 0
        self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()
        
        while(self.theta<math.radians(90)):
            vel_msg.angular.z = math.radians(20)
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()

            
            
    def rotate_minimum(self):
        min_dis=np.min(self.scan_ranges)
        vel_msg = Twist()
        print('min_dis ',min_dis, np.argmin(self.scan_ranges))
        time.sleep(1)
        while(abs(self.scan_ranges[269]-min_dis)>0.1):
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = math.radians(-25)
            

            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
            print("at 90 here",self.scan_ranges[270] )
            
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()
        print("exisiting")

            
    def obj_coor(self,min_dis, ref_angle):
        ref_angle=math.radians(ref_angle)
        transformation_matrix=np.array([[cos(self.theta),-1*sin(self.theta),self.x],
        [sin(self.theta),cos(self.theta),self.y],
        [0,0,1]])
        
        obs_robot_coordinates=np.array([min_dis*cos(ref_angle),min_dis*sin(ref_angle),1]).reshape(3,1)
        
        obs_world=np.matmul(transformation_matrix,obs_robot_coordinates)
        return obs_world[0],obs_world[1]

    def rotate_goal(self,goal):
        des_angle=np.argmin(self.scan_ranges)
        
        
        error=des_angle-goal
        vel_msg = Twist()
        print("error : ",error)

        while (abs(error)>3):
            print("rotating")
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = math.radians(error)
            

            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
            des_angle=np.argmin(self.scan_ranges)
            error=des_angle-goal

            

        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()
            
            
        
    def map_object(self):
        self.rotate_goal(0)
        self.move_minimum_distance()
        vel_msg = Twist()
        
        while not rospy.is_shutdown():
            
            vel_msg.linear.x = 0.1
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0
            

            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
            
            if self.scan_ranges[270]<10:
                print("mapping ")
                obs_x_w,obs_y_w=self.obj_coor(self.scan_ranges[269],270)
                print(obs_x_w,obs_y_w)
                plt.plot(obs_x_w,obs_y_w,'.r')
                plt.draw()
                plt.xlim(0, 3)
                plt.ylim(0,3)
                plt.pause(0.00001)
            print("at 90 ",self.scan_ranges[269] )
            
            min_range=min(self.scan_ranges)
            if abs(self.scan_ranges[269]-self.minimun_distance)>0.1:
                self.rotate_goal(270)
                
        vel_msg.linear.x=0
        self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()
        
        plt.savefig("map.png")
        print("figure is saved ")
        plt.show()

if __name__ == '__main__':
    robo=my_turtlebot3_map()
    time.sleep(1)
    robo.map_object()
