#!/usr/bin/env python  
import rospy

import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('vicon_tf_pub')
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    trans = tf_buffer.lookup_transform('/vicon/world', 'vicon/ROB498_Drone/ROB498_Drone', rospy.Time())
    while not rospy.is_shutdown():
        msg = geometry_msgs.msg.TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/vicon/world"
        msg.child_frame_id = "map"
        msg.transform = trans
        br.sendTransform(msg)