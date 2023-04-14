#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Bool

def talker():
    pub = rospy.Publisher('dummy_obs_type', Bool, queue_size=10)
    rospy.init_node('dummy_obs_pose_publisher', anonymous=True)

    is_clockwise = True
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(is_clockwise)
        rate.sleep()
 
if __name__ == '__main__':
    talker()