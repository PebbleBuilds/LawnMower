#!/usr/bin/env python
import rospy

# to get command line arguments
import sys

# because of transformations
import tf

import tf2_ros
import geometry_msgs.msg import TransformStamped, PoseStamped

POSE = None

def update_transform(msg):
    global POSE
    POSE = msg

if __name__ == '__main__':
    update = False

    rospy.init_node('static_tf2_broadcaster_node')

    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, update_transform)

    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "map"
    static_transformStamped.child_frame_id = "vicon_world"

    listener = tf.TransformListener()


    while not rospy.is_shutdown():
        if POSE is None:
            continue

        (trans, rot) = listener.lookupTransform('vicon_drone', 'vicon_world', rospy.Time(0))

        # Update the transform and rotation in static_transformStamped
        static_transformStamped.transform = trans
        static_transformStamped.rotation = rot

        # Update the translation by subtracting the current position of the drone
        static_transformStamped.transform.translation.x -= POSE.pose.position.x
        static_transformStamped.transform.translation.y -= POSE.pose.position.y
        static_transformStamped.transform.translation.z -= POSE.pose.position.z

        # TODO: Figure out how to update the rotation! 

        # Send the transform
        static_broadcaster.sendTransform(static_transformStamped)
        rospy.spin()
