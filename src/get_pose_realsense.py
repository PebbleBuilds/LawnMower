#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PoseStamped

PUB_RATE = 10 # Hz

class GetRealsense:
    def __init__(self) -> None:
        self.pub = rospy.Publisher('pose_realsense', PoseStamped, queue_size=10)
        self.sub = rospy.Subscriber('realsense_measurement', PoseStamped, self.realsense_callback)
        rospy.init_node('get_pose_realsense', anonymous=True)
        self.rate = rospy.Rate(PUB_RATE) # 10hz
        
    def realsense_callback(self, realsense_measurement):
        # convert realsense measurement to pose in vicon frame
        # publish pose
        self.pub.publish(realsense_measurement)

    def get_pose_realsense(self):
        pass
        # perform calibration to get the pose of the realsense camera in the vicon frame
        # compute tf from vicon to realsense


if __name__ == '__main__':
    get_realsesnse = GetRealsense()
    rospy.spin()
