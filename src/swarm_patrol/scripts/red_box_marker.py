#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
import random

def publish_marker():
    rospy.init_node('red_box_marker')
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=1)

    rooms = {
        "Room 1": (-2, 2),
        "Room 2": (0, 2),
        "Room 3": (2, 2)
    }
    selected_room = random.choice(list(rooms.keys()))
    pos_x, pos_y = rooms[selected_room]

    rospy.loginfo(f"Red box placed in {selected_room} at ({pos_x}, {pos_y})")

    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "red_box"
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.position.x = pos_x
    marker.pose.position.y = pos_y
    marker.pose.position.z = 0.1
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = 0.3
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(marker)
        rate.sleep()

if __name__ == "__main__":
    try:
        publish_marker()
    except rospy.ROSInterruptException:
        pass