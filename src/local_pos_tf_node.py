#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped

from constants import *

POSE = None


def update_transform(msg):
    global POSE
    POSE = msg


if __name__ == "__main__":
    rospy.init_node("static_tf2_broadcaster_node")
    rospy.Subscriber(MAVROS_POSE_TOPIC, PoseStamped, update_transform)

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    drone_tf = TransformStamped()
    drone_tf.header.frame_id = LOCAL_ORIGIN_FRAME_ID
    drone_tf.child_frame_id = DRONE_FRAME_ID

    while not rospy.is_shutdown():
        if POSE is None:
            rospy.loginfo("Waiting for drone local pose")
            continue
        drone_tf.header.stamp = POSE.header.stamp
        drone_tf.transform.translation.x = POSE.pose.position.x
        drone_tf.transform.translation.y = POSE.pose.position.y
        drone_tf.transform.translation.z = POSE.pose.position.z
        drone_tf.transform.rotation = POSE.pose.orientation
        br.sendTransform(drone_tf)
        rospy.spin()
