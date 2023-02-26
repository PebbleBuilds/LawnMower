#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PoseStamped
import tf_conversion as tfc
import tf2_ros
import geometry_msgs.msg

PUB_RATE = 10 # Hz

class RealsensePoseHandler:
    def __init__(self) -> None:
        self.br = tf2_ros.TransformBroadcaster()

    def realsense_pose_callback(self, msg: PoseStamped):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "vicon"
        t.child_frame_id = "realsense"

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        
        t.transform.rotation.x = msg.pose.orientation.x
        t.transform.rotation.y = msg.pose.orientation.y
        t.transform.rotation.z = msg.pose.orientation.z
        t.transform.rotation.w = msg.pose.orientation.w

        self.br.sendTransform(t)
    


if __name__ == '__main__':
    rospy.init_node('realsense_pose_handler', anonymous=True)
    realsense_pose_handler = RealsensePoseHandler()
    rospy.Subscriber("realsense_pose", PoseStamped, realsense_pose_handler.realsense_pose_callback)
    rospy.spin()
