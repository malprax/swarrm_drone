#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

class HeartbeatPublisher:
    def __init__(self):
        rospy.init_node('heartbeat_publisher', anonymous=True)
        self.pub = rospy.Publisher('heartbeat', String, queue_size=10)
        self.rate = rospy.Rate(2)

    def start(self):
        while not rospy.is_shutdown():
            self.pub.publish("leader_alive")
            self.rate.sleep()

if __name__ == '__main__':
    hb = HeartbeatPublisher()
    hb.start()