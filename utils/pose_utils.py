from geometry_msgs.msg import Point, PoseStamped
import numpy as np
def create_posestamped(pose, orientation=[1, 0, 0, 0])->PoseStamped:
    pose = PoseStamped()
    x_pose, y_pose, z_pose = pose
    pose.pose.position.x = x_pose
    pose.pose.position.y = y_pose
    pose.pose.position.z = z_pose
    w_o, x_o, y_o, z_o = orientation
    pose.pose.orientation.w = w_o
    pose.pose.orientation.x = x_o
    pose.pose.orientation.y = y_o
    pose.pose.orientation.z = z_o
    return pose

def posestamped2np(pose:PoseStamped)->np.ndarray:
    return np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])

def np2posestamped(pose:np.ndarray)->PoseStamped:
    return create_posestamped(pose)