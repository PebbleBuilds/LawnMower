from geometry_msgs.msg import Point, PoseStamped
import numpy as np
def create_posestamped(pose_xyz, orientation=[1, 0, 0, 0]):
    pose_stamped = PoseStamped()
    x_pose, y_pose, z_pose = pose_xyz
    pose_stamped.pose.position.x = x_pose
    pose_stamped.pose.position.y = y_pose
    pose_stamped.pose.position.z = z_pose
    w_o, x_o, y_o, z_o = orientation
    pose_stamped.pose.orientation.w = w_o
    pose_stamped.pose.orientation.x = x_o
    pose_stamped.pose.orientation.y = y_o
    pose_stamped.pose.orientation.z = z_o
    return pose_stamped

def posestamped2np(pose):
    return np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])

def np2posestamped(pose):
    return create_posestamped(pose)