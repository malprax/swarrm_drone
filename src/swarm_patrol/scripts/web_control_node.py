#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import os

class WebControlNode:
    def __init__(self):
        rospy.init_node('web_control_node')

        self.room_sub = rospy.Subscriber('/webui/room_selection', String, self.set_red_box_location)
        self.leader_kill_sub = rospy.Subscriber('/webui/disable_leader', String, self.kill_leader)

        self.log_pub = rospy.Publisher('/swarm/log', String, queue_size=10)

        rospy.loginfo("Web Control Node aktif. Menunggu perintah dari UI...")
        rospy.spin()

    def set_red_box_location(self, msg):
        room = msg.data.strip()
        rospy.set_param('/red_box_room', room)
        self.log_pub.publish(f"[WEB_UI] Red box ditetapkan ke {room}")
        rospy.loginfo(f"Red box moved to {room}")

    def kill_leader(self, msg):
        try:
            os.system("rosnode kill /swarm_drone1")
            self.log_pub.publish("[WEB_UI] Leader (/swarm_drone1) telah dinonaktifkan!")
            rospy.logwarn("Leader node killed by Web UI!")
        except Exception as e:
            rospy.logerr(f"Error killing leader: {e}")

if __name__ == '__main__':
    try:
        WebControlNode()
    except rospy.ROSInterruptException:
        pass