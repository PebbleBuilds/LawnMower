#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

from constants import *

if __name__ == "__main__":
    rospy.init_node("vicon_tf_pub")

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(20)
    # initialize to identity
    last_trans = None
    while not rospy.is_shutdown():
        try:
            trans = tf_buffer.lookup_transform(
                LOCAL_ORIGIN_FRAME_ID, 
                VICON_ORIGIN_FRAME_ID, 
                rospy.Time(0),
            )
            last_trans = trans
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            print(e)
            rospy.logwarn("vicon tf Failed to get transform from vicon")
        if last_trans is not None:
            # latch on to last known transform
            # last_trans.header.stamp = rospy.Time.now()
            last_trans.header.frame_id = LOCAL_ORIGIN_FRAME_ID
            last_trans.child_frame_id = VICON_DUMMY_FRAME_ID
            br.sendTransform(last_trans)
        rate.sleep()
