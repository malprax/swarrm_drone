#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import random

class ObjectDetection2D:
    def __init__(self):
        rospy.init_node('object_detection_2d', anonymous=True)
        self.rate = rospy.Rate(1)

    def detect(self):
        while not rospy.is_shutdown():
            # dummy detection logic
            if random.randint(0, 100) < 5:  # 5% chance per second
                rospy.loginfo("Red box detected!")
            self.rate.sleep()

if __name__ == '__main__':
    od = ObjectDetection2D()
    od.detect()