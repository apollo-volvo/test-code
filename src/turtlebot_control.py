#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import requests


class RobotControl:

    def __init__(self, robot_name="turtlebot", arduino_ip="10.183.111.239"):
        rospy.init_node('robot_control_node', anonymous=True)

        rospy.loginfo("Robot Turtlebot...")
        cmd_vel_topic = '/cmd_vel'

        # Start the publisher
        self.vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self.cmd = Twist()

        self.arduino_ip = arduino_ip
        self.ctrl_c = False
        self.rate = rospy.Rate(1)
        rospy.on_shutdown(self.shutdownhook)

    def publish_once_in_cmd_vel(self):
        while not self.ctrl_c:
            connections = self.vel_publisher.get_num_connections()
            if connections > 0:
                self.vel_publisher.publish(self.cmd)
                break
            else:
                self.rate.sleep()

    def shutdownhook(self):
        self.ctrl_c = True
        self.stop_robot()

    def stop_robot(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()

    def move_straight_time(self, motion, speed, duration):
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        self.cmd.linear.x = speed if motion == "forward" else -speed

        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < duration:
            self.vel_publisher.publish(self.cmd)
            self.rate.sleep()

        self.stop_robot()

    def turn(self, clockwise, speed, duration):
        self.cmd.linear.x = 0
        self.cmd.angular.z = speed if clockwise != "clockwise" else -speed

        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < duration:
            self.vel_publisher.publish(self.cmd)
            self.rate.sleep()

        self.stop_robot()

    def send_command(self, command):
        url = f"http://{self.arduino_ip}/{command}"
        try:
            response = requests.get(url, timeout=5)
            if response.status_code == 200:
                rospy.loginfo(f"Command '{command}' successful: {response.text}")
            else:
                rospy.logwarn(f"Command '{command}' failed with status code: {response.status_code}")
        except requests.exceptions.RequestException as e:
            rospy.logerr(f"Error sending command '{command}': {e}")

    def execute_sequence(self):
        rospy.loginfo("Starting sequence...")

        # Step 1: Move out of parking lot
        rospy.loginfo("Exiting parking lot...")
        self.move_straight_time("forward", 0.2, 3)
        self.turn("clockwise", 0.5, 2)

        # Step 2: Open gantry
        rospy.loginfo("Opening gantry...")
        self.send_command("open")
        rospy.sleep(2)

        # Step 3: Move past gantry
        rospy.loginfo("Moving past gantry...")
        self.move_straight_time("forward", 0.2, 5)

        # Step 4: Close gantry
        rospy.loginfo("Closing gantry...")
        self.send_command("close")

        rospy.loginfo("Sequence complete!")


if __name__ == '__main__':
    robotcontrol = RobotControl()
    try:
        robotcontrol.execute_sequence()
    except rospy.ROSInterruptException:
        pass
