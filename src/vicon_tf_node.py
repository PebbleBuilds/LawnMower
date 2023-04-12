#!/usr/bin/env python
import rospy
import tf2_ros

from constants import *

if __name__ == "__main__":
    rospy.init_node("vicon_tf_pub")

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    last_trans = None
    while not rospy.is_shutdown():
        try:
            trans = tf_buffer.lookup_transform(
                LOCAL_ORIGIN_FRAME_ID, 
                VICON_ORIGIN_FRAME_ID, 
                rospy.Time(),
                rospy.Duration(0, 1e8)
            )
            last_trans = trans
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            if last_trans is None:
                rospy.loginfo("Waiting for vicon transform")
            else:
                # latch on to last known transform
                last_trans.header.stamp = rospy.Time.now()
                br.sendTransform(last_trans)
        rate.sleep()
