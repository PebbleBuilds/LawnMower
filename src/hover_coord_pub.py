#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

# x y z given in meters, yaw given in radians
HOVER_COORD = [0, 0, 1.5, 0] # x, y, z, yaw
PUB_RATE = 10 # Hz

def hover_coord_pub():
    pub = rospy.Publisher('waypoint', PoseStamped, queue_size=10)
    rospy.init_node('hover_coord_pub', anonymous=True)
    rate = rospy.Rate(PUB_RATE) # 10hz
    pose = PoseStamped()
    pose.header.frame_id = "vicon"
    pose.pose.position.x = HOVER_COORD[0]
    pose.pose.position.y = HOVER_COORD[1]
    pose.pose.position.z = HOVER_COORD[2]
    q = quaternion_from_euler(0, 0, HOVER_COORD[3])
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    while not rospy.is_shutdown():
        pose_str = "x: %s, y: %s, z: %s, yaw: %s" % (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, HOVER_COORD[3])
        rospy.loginfo(pose_str)
        pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        hover_coord_pub()
    except rospy.ROSInterruptException:
        pass