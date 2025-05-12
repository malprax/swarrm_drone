#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import random

class BatteryMonitor:
    def __init__(self):
        rospy.init_node('battery_monitor', anonymous=True)
        self.pub = rospy.Publisher('battery_level', Float32, queue_size=10)
        self.battery = 100.0
        self.rate = rospy.Rate(1)

    def start(self):
        while not rospy.is_shutdown():
            self.battery -= random.uniform(0.5, 1.0)
            self.pub.publish(self.battery)
            if self.battery <= 0:
                self.battery = 0
            self.rate.sleep()

if __name__ == '__main__':
    bm = BatteryMonitor()
    bm.start()