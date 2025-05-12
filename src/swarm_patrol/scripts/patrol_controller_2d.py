#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import random
import time

class PatrolController2D:
    def __init__(self):
        rospy.init_node('patrol_controller_2d', anonymous=True)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.hover_time = 5
        self.detected = False
        self.rate = rospy.Rate(0.5)

    def patrol(self):
        while not rospy.is_shutdown():
            if self.detected:
                rospy.loginfo("Hovering after detection...")
                self.hover()
                self.return_home()
                break

            move_cmd = Twist()
            move_cmd.linear.x = random.uniform(-1.0, 1.0)
            move_cmd.linear.y = random.uniform(-1.0, 1.0)
            self.cmd_pub.publish(move_cmd)
            self.rate.sleep()

    def hover(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0
        move_cmd.linear.y = 0
        self.cmd_pub.publish(move_cmd)
        rospy.sleep(self.hover_time)

    def return_home(self):
        rospy.loginfo("Returning to Start Point...")
        move_cmd = Twist()
        move_cmd.linear.x = -1.0
        move_cmd.linear.y = -1.0
        self.cmd_pub.publish(move_cmd)

if __name__ == '__main__':
    pc = PatrolController2D()
    pc.patrol()