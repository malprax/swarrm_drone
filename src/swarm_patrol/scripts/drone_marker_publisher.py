#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class DroneMarkerPublisher:
    def __init__(self):
        rospy.init_node('drone_marker_publisher', anonymous=True)
        self.pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.rate = rospy.Rate(1)

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        points = []
        points.append(Point(-0.05, 0, 0))
        points.append(Point(0.05, 0, 0))
        points.append(Point(0, -0.05, 0))
        points.append(Point(0, 0.05, 0))
        marker.points = points

        while not rospy.is_shutdown():
            self.pub.publish(marker)
            self.rate.sleep()

if __name__ == '__main__':
    dmp = DroneMarkerPublisher()
    dmp.publish_marker()