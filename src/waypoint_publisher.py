#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from constants import *

def pub_node():
    rospy.init_node('pub_node')
    pub = rospy.Publisher(WAYPOINTS_TOPIC, PoseArray, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    pose_array = PoseArray()
    pose_array.header.frame_id = VICON_DUMMY_FRAME_ID
    pose_array.poses = [Pose() for i in range(4)]
    for pose_stamped in pose_array.poses:
        pose_stamped.position.z = 1
        pose_stamped.orientation.w = 1

    pose_array.poses[0].position.x = -1
    pose_array.poses[0].position.y = 0

    pose_array.poses[1].position.x = -2
    pose_array.poses[1].position.y = 0

    pose_array.poses[2].position.x = -2
    pose_array.poses[2].position.y = 1.5

    pose_array.poses[3].position.x = 2
    pose_array.poses[3].position.y = 1.5
    
    while not rospy.is_shutdown():
        pub.publish(pose_array)
        rate.sleep()

if __name__ == '__main__':
    try:
        pub_node()
    except rospy.ROSInterruptException:
        pass