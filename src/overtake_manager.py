#!/usr/bin/env python3

import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist

class OvertakeManager:
    def __init__(self):
        rospy.init_node('overtake_manager')
        self.bounding_boxes_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.detect_callback)
        self.vehicle_detected = False

        cmd_vel_topic = '/cmd_vel'
        self.vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        self.cmd = Twist()

        self.rate = rospy.Rate(1)
        
    def detect_callback(self, msg):
        for box in msg.bounding_boxes:
            if (box.Class == 'car' or 'suitcase' or 'phone') and box.probability > 0.5:
                # Assume a bounding box position correlates with the lane center
                self.bounding_boxes_sub.unregister()
                self.vehicle_detected = True
                self.overtake()
                self.bounding_boxes_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.detect_callback)
                return
            if (box.Class == 'person' or 'bottle') and box.probability > 0.5:
                # Assume a bounding box position correlates with the lane center
                self.bounding_boxes_sub.unregister()
                self.vehicle_detected = True
                self.wait_for_pedestrian()
                self.bounding_boxes_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.detect_callback)
                return
        self.vehicle_detected = False

    def stop_robot(self):
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0
        self.vel_publisher.publish(self.cmd)

    def move_straight_time(self, motion, speed, duration):
        # Set linear speed based on motion direction
        linear_speed = speed if motion == "forward" else -speed
        
        # Record the start time
        start_time = rospy.Time.now()
        
        # Loop until the specified duration has passed
        while (rospy.Time.now() - start_time).to_sec() < duration:
            # Set the robot's linear and angular velocities
            self.cmd.linear.x = linear_speed
            self.cmd.angular.z = 0.0
            
            # Publish the velocity command
            self.vel_publisher.publish(self.cmd)
            
            # Sleep for the remainder of the loop cycle
        
        # Stop the robot after moving
        self.stop_robot()

    def turn(self, clockwise, speed, duration):
        turn_speed = speed if clockwise != "clockwise" else -speed
        start_time = rospy.Time.now()
        while ((rospy.Time.now() - start_time).to_sec() < duration):
            # Move forward for duration seconds
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = turn_speed
            self.vel_publisher.publish(self.cmd)
        self.stop_robot()

    def overtake(self):
        rospy.loginfo("Starting overtake sequence...")

        # Step 1: stop in front of car
        self.stop_robot()
        rospy.loginfo("Stop car")
        # self.move_straight_time("forward", 0.15, 2)

        # Step 2: head towards other lane
        self.turn("clockwise", 0.2, 1)
        self.move_straight_time("forward", 0.1, 1.5)
        self.turn("anticlockwise", 0.2, 1)
        rospy.loginfo("moving to next lane")

        # Step 3: move in other lane
        self.move_straight_time("forward", 0.1, 3)
        rospy.loginfo("Moving past obstacle")

        # Step 4: head back to original lane
        self.stop_robot()
        self.turn("anticlockwise", 0.2, 1)
        self.move_straight_time("forward", 0.1, 1.5)
        self.turn("clockwise", 0.2, 1)
        rospy.loginfo("Moving back to original lane")

        self.stop_robot()
        rospy.loginfo("overtake Sequence complete!")

    def wait_for_pedestrian(self):
        rospy.loginfo("wait for pedestrian...")

        # Step 1: stop in front of car
        self.stop_robot()
        rospy.sleep(3)

if __name__ == '__main__':
    try:
        manager = OvertakeManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass