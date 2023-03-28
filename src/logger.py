import time
import numpy as np
import rospy
import pose_utils
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped

class Logger:
    def __init__(self, prefix="drone_positions"):
        self.path = "home/ubuntu/logger_results/"
        self.prefix = prefix
        self.filepath = self.path + self.prefix + time.strftime("%d%b%Y-%H:%M:%S", time.localtime())
        self.pose_data = np.empty()
        self.vicon_data = np.empty()

    def callback_vicon(msg):
        self.vicon_data = np.vstack(self.vicon_data,pose_utils.posestamped2np(msg,True))

    def callback_pose(msg):
        self.pose_data = np.vstack(self.pose_data,pose_utils.posestamped2np(msg,True))

    def export():
        np.savetxt(self.filepath+"-vicon.csv",self.vicon_data,delimiter=",")
        np.savetxt(self.filepath+"-pose.csv",self.pose_data,delimiter=",")
        
    def start():
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'logger' node so that multiple loggers can
        # run simultaneously.
        rospy.init_node('logger', anonymous=True)
        rospy.Subscriber("/vicon/ROB498_Drone/ROB498_Drone", TransformStamped, callback_vicon)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback_pose)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    log = Logger()
    try:
        log.start()
    finally:
        log.export()