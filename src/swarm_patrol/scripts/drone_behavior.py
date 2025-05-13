#!/usr/bin/env python3
import rospy
import random
import time
import os
print("🔍 Script path:", os.path.abspath(__file__))

from std_msgs.msg import String

class Drone:
    def __init__(self, drone_id, is_leader=False):
        self.drone_id = drone_id
        self.is_leader = is_leader
        self.room = random.choice(['Room 1', 'Room 2', 'Room 3'])
        self.found_red_box = False

        # rospy.init_node(f'drone_{drone_id}', anonymous=False)
        self.pub = rospy.Publisher('/swarm/log', String, queue_size=10)
        self.sub = rospy.Subscriber('/swarm/found', String, self.callback_found)

        rospy.sleep(1)
        rospy.loginfo(f"Drone {self.drone_id} assigned to {self.room}")
        self.search()

    def callback_found(self, msg):
        if not self.is_leader:
            rospy.loginfo(f"Drone {self.drone_id} received: {msg.data}")

    def search(self):
        rospy.loginfo(f"Drone {self.drone_id} is searching in {self.room}")
        rospy.sleep(random.randint(2, 5))
        self.found_red_box = random.choice([True, False])

        if self.found_red_box:
            if self.is_leader:
                msg = f"Leader {self.drone_id} found red box in {self.room}"
                self.pub.publish(msg)
                rospy.loginfo(msg)
            else:
                msg = f"Drone {self.drone_id} (member) found red box in {self.room}"
                leader_msg = f"Leader logs: {msg}"
                self.pub.publish(leader_msg)
                rospy.loginfo(leader_msg)

        rospy.sleep(10)
        rospy.loginfo(f"Drone {self.drone_id} returning to start point.")

if __name__ == '__main__':
    try:
        rospy.init_node('drone_behavior', anonymous=False)  # 🟢 Pindah ke sini dulu
        is_leader_param = rospy.get_param('~is_leader', False)
        drone_id_param = rospy.get_param('~drone_id', 'droneX')
        print(f"🟢 Param drone_id yang diterima: {drone_id_param}")
        Drone(drone_id_param, is_leader=is_leader_param)
    except rospy.ROSInterruptException:
        pass