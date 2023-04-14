#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from pose_utils import *

def talker():
    pub = rospy.Publisher('dummy_obs_pose', PoseStamped, queue_size=10)
    rospy.init_node('dummy_obs_pose_publisher', anonymous=True)

    obs_position = create_posestamped(pose_xyz=[4, 5, 0])
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(obs_position)
        rate.sleep()
 
if __name__ == '__main__':
    talker()