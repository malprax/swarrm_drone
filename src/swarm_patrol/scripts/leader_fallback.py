#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import time

class SwarmDrone:
    def __init__(self, drone_id, is_leader=False):
        rospy.init_node(f'swarm_{drone_id}', anonymous=True)
        self.drone_id = drone_id
        self.is_leader = is_leader
        self.last_heartbeat = time.time()
        self.found_red_box = False

        self.log_pub = rospy.Publisher('/swarm/log', String, queue_size=10)
        self.heartbeat_pub = rospy.Publisher('/swarm/heartbeat', String, queue_size=10)
        self.heartbeat_sub = rospy.Subscriber('/swarm/heartbeat', String, self.heartbeat_callback)
        self.found_sub = rospy.Subscriber('/swarm/found', String, self.found_callback)

        rospy.sleep(1)
        rospy.loginfo(f"[{self.drone_id}] Started as {'LEADER' if self.is_leader else 'MEMBER'}")
        self.run()

    def heartbeat_callback(self, msg):
        if "LEADER" in msg.data and not self.is_leader:
            self.last_heartbeat = time.time()

    def found_callback(self, msg):
        rospy.loginfo(f"[{self.drone_id}] RECEIVED: {msg.data}")

    def publish_heartbeat(self):
        msg = f"HEARTBEAT from LEADER {self.drone_id}"
        self.heartbeat_pub.publish(msg)

    def check_leader_alive(self):
        now = time.time()
        if not self.is_leader and (now - self.last_heartbeat > 5.0):
            self.is_leader = True
            rospy.logwarn(f"[{self.drone_id}] Leader timed out. I am the new LEADER.")
            self.log_pub.publish(f"Drone {self.drone_id} has become the new LEADER.")

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.is_leader:
                self.publish_heartbeat()
            else:
                self.check_leader_alive()
            rate.sleep()

if __name__ == "__main__":
    try:
        drone_id = rospy.get_param('~drone_id', 'droneX')
        is_leader = rospy.get_param('~is_leader', False)
        SwarmDrone(drone_id, is_leader)
    except rospy.ROSInterruptException:
        pass