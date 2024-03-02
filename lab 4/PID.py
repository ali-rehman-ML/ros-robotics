from robomaster import robot
import numpy as np
import time
from math import atan2, degrees, radians, atan
import keyboard

class PID_goto_goal:
    def __init__(self,dt,kp,ki,kd,linear_velocity):

        self.x=0
        self.y=0
        self.yaw=0
        self.linear_velocity=linear_velocity
        self.kp=kp
        self.ki=ki
        self.kd=kd
        self.dt=dt
        self.robot=robot.Robot()
        self.robot.initialize(conn_type="ap")
        self.chassis=self.robot.chassis
        self.chassis.sub_position(freq=10, callback=self.sub_position_handler)
        self.chassis.sub_attitude(freq=10, callback=self.sub_attitude_info_handler)
        self.goal_x=float(input("Enter your goal x : "))
        self.goal_y=float(input("Enter your goal y : "))
        self.tolerance=float(input("Enter your tolernace : "))

        self.current_e=0
        self.previous_e=0
        self.E_int=0



    def sub_position_handler(self,position_info):


        self.x, self.y, _ = position_info

    def sub_attitude_info_handler(self,attitude_info):
        self.yaw, _, _ = attitude_info




    def PID(self,e):
        self.current_e=e

        e_d=(self.current_e-self.previous_e)/self.dt
        e_i=self.E_int+self.current_e*self.dt
        self.previous_e=self.current_e

        E_pid=self.kp*self.current_e + self.ki*e_i + self.kd*e_d
        return E_pid
    

    def goto_goal(self):


        current_location=np.array([self.x,self.y])
        goal_location=np.array([self.goal_x,self.goal_y])

        distance=np.linalg.norm(current_location-goal_location)
        while(distance>self.tolerance):
            des_angle=atan2((self.goal_y-self.y),(self.goal_x-self.x))



            angle_error=des_angle-radians(self.yaw)

            angular_velocity=degrees(self.PID(angle_error))

            angular_velocity=np.clip(angular_velocity,-90,90)

            self.chassis.drive_speed(x=self.linear_velocity, y=0, z=angular_velocity, timeout=5)
            time.sleep(self.dt)
            current_location=np.array([self.x,self.y])
            distance=np.linalg.norm(current_location-goal_location)

            if keyboard.is_pressed('b'):
                break
        
        self.chassis.drive_speed(x=-0.1, y=0, z=0, timeout=5)
        time.sleep(self.dt)
        self.chassis.drive_speed(x=0, y=0, z=0, timeout=5)
        time.sleep(0.5)
        self.robot.close()


if __name__ == '__main__':
    robo=PID_goto_goal(dt=0.01,kp=1.5,ki=0.1,kd=0.1,linear_velocity=0.5)
    robo.goto_goal()
     

    



    


