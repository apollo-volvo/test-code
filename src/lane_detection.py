#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

bridge = CvBridge()
cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def image_callback(msg):
    try:
        # Convert ROS Image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Edge detection
        edges = cv2.Canny(gray, 50, 150)

        # Detect lines using Hough Transform
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=10)

        # Compute lane center and offset
        height, width = gray.shape
        lane_center = width // 2  # Default to center of the image
        detected_lines = []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                detected_lines.append(((x1, y1), (x2, y2)))

            # Calculate lane center based on detected lines (simplified example)
            left_lane_x = min(line[0][0] for line in detected_lines)  # Example logic
            right_lane_x = max(line[1][0] for line in detected_lines)
            lane_center = (left_lane_x + right_lane_x) // 2

        # Compute offset from the image center
        image_center = width // 2
        offset = lane_center - image_center

        # Calculate steering angle
        steering_angle = -offset / float(width)  # Normalize offset to [-1, 1]

        # Publish motor commands
        twist = Twist()
        twist.linear.x = 0.5  # Constant speed
        twist.angular.z = steering_angle
        cmd_pub.publish(twist)

        # Display processed image
        cv2.imshow("Lane Detection", cv_image)
        cv2.waitKey(1)

    except Exception as e:
        rospy.logerr(f"Error processing image: {e}")

def main():
    rospy.init_node('lane_detection_motor_control')
    rospy.Subscriber('/camera/image', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
