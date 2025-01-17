#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def cmd_callback(msg):
    # Extract linear and angular velocity
    linear = msg.linear.x
    angular = msg.angular.z
    # Translate velocities into motor commands
    left_motor_speed = linear - angular
    right_motor_speed = linear + angular

    # Send motor commands to hardware (e.g., via serial or GPIO)
    rospy.loginfo(f"Left Motor: {left_motor_speed}, Right Motor: {right_motor_speed}")
    # Add actual motor driver communication here

def main():
    rospy.init_node('motor_driver')
    rospy.Subscriber('/cmd_vel', Twist, cmd_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
