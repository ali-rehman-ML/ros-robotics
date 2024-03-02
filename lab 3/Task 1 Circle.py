#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def move_turtlebot3_circle():
    """
    Moves the TurtleBot3 in a circle with a linear velocity of 1 m/s and an angular velocity of 2 rad/s.

    This function assumes that the TurtleBot3 is running in a simulated environment,
    such as Gazebo, and that the ROS network is already established.

    Args:
        None

    Returns:
        None
    """

    # Initialize ROS node
    rospy.init_node('move_turtlebot3_circle')

    # Create a publisher for the /cmd_vel topic
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Set circular motion parameters
    linear_vel = 1.0  # m/s
    angular_vel = 1.0  # rad/s

    # Create a Twist message
    twist = Twist()
    twist.linear.x = linear_vel
    twist.linear.y=0
    twist.linear.z=0
    twist.angular.z = angular_vel
    twist.angular.x=0
    twist.angular.y=0
    

    # Keep publishing twist commands at a rate of 10 Hz
    rate = rospy.Rate(10)  # Hz

    while not rospy.is_shutdown():
        # Publish the twist message
        pub.publish(twist)

        # Sleep for a period to maintain the desired rate
        rate.sleep()

if __name__ == '__main__':
    try:
        move_turtlebot3_circle()
    except rospy.ROSInterruptException:
        pass

