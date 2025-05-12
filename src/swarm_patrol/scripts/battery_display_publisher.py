#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32

class BatteryDisplayPublisher:
    def __init__(self):
        rospy.init_node('battery_display_publisher', anonymous=True)
        self.pub = rospy.Publisher('battery_marker', Marker, queue_size=10)
        self.sub = rospy.Subscriber('battery_level', Float32, self.callback)
        self.battery = 100.0
        self.rate = rospy.Rate(1)

    def callback(self, msg):
        self.battery = msg.data

    def publish_battery_marker(self):
        while not rospy.is_shutdown():
            marker = Marker()
            marker.header.frame_id = "world"
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.scale.z = 0.3
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.pose.position.x = 0
            marker.pose.position.y = 0
            marker.pose.position.z = 1
            marker.text = f"Battery: {int(self.battery)}%"

            self.pub.publish(marker)
            self.rate.sleep()

if __name__ == '__main__':
    bdp = BatteryDisplayPublisher()
    bdp.publish_battery_marker()