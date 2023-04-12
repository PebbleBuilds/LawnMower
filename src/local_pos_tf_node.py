#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped, PointStamped

from constants import *
from pose_utils import pose2np
import numpy as np
from tf.transformations import quaternion_matrix, quaternion_from_matrix
POSE = None


def update_transform(msg):
    global POSE
    POSE = msg


if __name__ == "__main__":
    rospy.init_node("static_tf2_broadcaster_node")
    rospy.Subscriber(MAVROS_POSE_TOPIC, PoseStamped, update_transform)

    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(100.0)

    drone2local_tf = TransformStamped()
    drone2local_tf.header.frame_id =  BASE_LINK_FRAME_ID
    drone2local_tf.child_frame_id = LOCAL_ORIGIN_FRAME_ID
    while not rospy.is_shutdown():
        if POSE is None:
            rospy.loginfo("Waiting for drone local pose")
            rate.sleep()
            continue
        drone2local_tf.header.stamp = POSE.header.stamp

        rot = quaternion_matrix([POSE.pose.orientation.x, POSE.pose.orientation.y, POSE.pose.orientation.z, POSE.pose.orientation.w]).T
        pose = pose2np(POSE.pose)
        pose = -np.dot(rot[0:3, 0:3], pose)
        quat = quaternion_from_matrix(rot)
        drone2local_tf.transform.translation.x = pose[0]
        drone2local_tf.transform.translation.y = pose[1]
        drone2local_tf.transform.translation.z = pose[2]
        drone2local_tf.transform.rotation.x = quat[0]
        drone2local_tf.transform.rotation.y = quat[1]
        drone2local_tf.transform.rotation.z = quat[2]
        drone2local_tf.transform.rotation.w = quat[3]
        print(pose[2])
        br.sendTransform(drone2local_tf)
        rate.sleep()
