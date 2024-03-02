from robomaster import robot
import numpy as np
import time
from math import atan2, degrees, radians, atan, cos ,sin
import keyboard

class pf_goto_goal:
    def __init__(self,dt,K_gains):

        self.x=0
        self.y=0
        self.yaw=0
        self.dt=dt
        self.K=K_gains
        self.robot=robot.Robot()
        self.robot.initialize(conn_type="ap")
        self.chassis=self.robot.chassis
        self.chassis.sub_position(freq=10, callback=self.sub_position_handler)
        self.chassis.sub_attitude(freq=10, callback=self.sub_attitude_info_handler)
        self.goal_x=float(input("Enter your goal x : "))
        self.goal_y=float(input("Enter your goal y : "))
        self.tolerance=float(input("Enter your tolernace : "))




    def sub_position_handler(self,position_info):
        self.x, self.y, _ = position_info

    def sub_attitude_info_handler(self,attitude_info):
        self.yaw, _, _ = attitude_info

    def get_rotation_matrix(self,yaw_rad):
        inv_R=np.array([[cos(yaw_rad), sin(yaw_rad)],   
             [-sin(yaw_rad), cos(yaw_rad)]])
        
        return inv_R
    

    def potential_gains(self,e):

        u=np.multiply(-self.K,e)

        return u


    
    def goto_goal(self):


        current_location=np.array([self.x,self.y])
        goal_location=np.array([self.goal_x,self.goal_y])

        distance=np.linalg.norm(current_location-goal_location)
        while(distance>self.tolerance):


            e=current_location-goal_location

            u=self.potential_gains(e)

            yaw_rad=radians(self.yaw)

            inv_R=self.get_rotation_matrix(yaw_rad)


            velocities=inv_R.dot(u)

            lin_vel=velocities[0]

            angular_velocity=velocities[1]

            angular_velocity=atan2(sin(angular_velocity),cos(angular_velocity)) #not necessary to do but recomended for some goal points
            angular_velocity=degrees(angular_velocity)

            #limiting velocities in a appropriate range
            lin_vel=np.clip(lin_vel,-1,1)
            angular_velocity=np.clip(angular_velocity,-90,90)

            self.chassis.drive_speed(x=lin_vel, y=0, z=angular_velocity, timeout=5)
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
    k_gains=np.array([1.5,2])
    robo=pf_goto_goal(dt=0.1,K_gains=k_gains)
    robo.goto_goal()
     

    



    


