#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_tracker')
        self.bridge = CvBridge()

        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.pos_pub = self.create_publisher(Point, '/object_position', 10)
        self.debug_pub = self.create_publisher(Image, '/object_tracking/debug', 10)

        # Track red objects (HSV range example)
        self.lower = np.array([0, 120, 70])
        self.upper = np.array([10, 255, 255])

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            c = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(c)
            if radius > 10:  # ignore noise
                point = Point()
                point.x, point.y, point.z = float(x), float(y), float(radius)
                self.pos_pub.publish(point)
                cv2.circle(frame, (int(x), int(y)), int(radius), (0,255,0), 2)

        debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.debug_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
