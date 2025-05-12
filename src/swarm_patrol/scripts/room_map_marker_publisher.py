#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def publish_room_map():
    rospy.init_node('room_map_marker_publisher')
    marker_pub = rospy.Publisher('/room_map_marker', Marker, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz

    # Marker for Room A
    room_a_marker = Marker()
    room_a_marker.header.frame_id = "world"
    room_a_marker.header.stamp = rospy.Time.now()
    room_a_marker.ns = "room_map"
    room_a_marker.id = 1
    room_a_marker.type = Marker.CUBE
    room_a_marker.action = Marker.ADD
    room_a_marker.pose.position.x = 2
    room_a_marker.pose.position.y = 2
    room_a_marker.pose.position.z = 0
    room_a_marker.scale.x = 1
    room_a_marker.scale.y = 1
    room_a_marker.scale.z = 0.1
    room_a_marker.color.a = 0.5  # transparency
    room_a_marker.color.r = 0.0
    room_a_marker.color.g = 1.0
    room_a_marker.color.b = 0.0  # green room

    # Marker for Room B
    room_b_marker = Marker()
    room_b_marker.header.frame_id = "world"
    room_b_marker.header.stamp = rospy.Time.now()
    room_b_marker.ns = "room_map"
    room_b_marker.id = 2
    room_b_marker.type = Marker.CUBE
    room_b_marker.action = Marker.ADD
    room_b_marker.pose.position.x = 2
    room_b_marker.pose.position.y = -2
    room_b_marker.pose.position.z = 0
    room_b_marker.scale.x = 1
    room_b_marker.scale.y = 1
    room_b_marker.scale.z = 0.1
    room_b_marker.color.a = 0.5
    room_b_marker.color.r = 0.0
    room_b_marker.color.g = 0.0
    room_b_marker.color.b = 1.0  # blue room

    # Marker for Room C (where the Red Box will be)
    room_c_marker = Marker()
    room_c_marker.header.frame_id = "world"
    room_c_marker.header.stamp = rospy.Time.now()
    room_c_marker.ns = "room_map"
    room_c_marker.id = 3
    room_c_marker.type = Marker.CUBE
    room_c_marker.action = Marker.ADD
    room_c_marker.pose.position.x = -2
    room_c_marker.pose.position.y = 2
    room_c_marker.pose.position.z = 0
    room_c_marker.scale.x = 1
    room_c_marker.scale.y = 1
    room_c_marker.scale.z = 0.1
    room_c_marker.color.a = 0.5
    room_c_marker.color.r = 1.0
    room_c_marker.color.g = 0.0
    room_c_marker.color.b = 0.0  # red room (Red Box area)

    # Marker for Red Box in Room C
    red_box_marker = Marker()
    red_box_marker.header.frame_id = "world"
    red_box_marker.header.stamp = rospy.Time.now()
    red_box_marker.ns = "room_map"
    red_box_marker.id = 4
    red_box_marker.type = Marker.CUBE
    red_box_marker.action = Marker.ADD
    red_box_marker.pose.position.x = -2
    red_box_marker.pose.position.y = 2
    red_box_marker.pose.position.z = 0.1  # slightly raised from ground
    red_box_marker.scale.x = 0.2
    red_box_marker.scale.y = 0.2
    red_box_marker.scale.z = 0.2
    red_box_marker.color.a = 1.0
    red_box_marker.color.r = 1.0
    red_box_marker.color.g = 0.0
    red_box_marker.color.b = 0.0  # red box

    while not rospy.is_shutdown():
        # Publish the markers
        marker_pub.publish(room_a_marker)
        marker_pub.publish(room_b_marker)
        marker_pub.publish(room_c_marker)
        marker_pub.publish(red_box_marker)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_room_map()
    except rospy.ROSInterruptException:
        pass