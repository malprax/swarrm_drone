#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import time

class HeartbeatListener:
    def __init__(self):
        rospy.init_node('heartbeat_listener', anonymous=True)
        self.sub = rospy.Subscriber('heartbeat', String, self.heartbeat_callback)
        self.last_heartbeat = time.time()
        self.is_leader = False
        self.drone_id = rospy.get_param('~drone_id', 1)

    def heartbeat_callback(self, msg):
        self.last_heartbeat = time.time()

    def start(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if time.time() - self.last_heartbeat > 3.0:
                if not self.is_leader:
                    self.become_leader()
            rate.sleep()

    def become_leader(self):
        rospy.loginfo(f"Drone {self.drone_id} is becoming the new leader!")
        self.is_leader = True

if __name__ == '__main__':
    hl = HeartbeatListener()
    hl.start()