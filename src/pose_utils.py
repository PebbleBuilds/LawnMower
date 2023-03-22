from geometry_msgs.msg import Point, PoseStamped
import tf2_geometry_msgs
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

def vicontf_to_Hvi(vicon_tf):
    H_vi = np.eye(4,4)
    C_vi = tf2_geometry_msgs.transform_to_matrix(vicon_tf.rotation)
    t_vi_i = np.array([
        vicon_tf.translation.x,
        vicon_tf.translation.y,
        vicon_tf.translation.z
    ])
    t_iv_v = np.matmul(-C_vi, t_vi_i)
    H_vi[0:3,0:3] = C_vi
    H_vi[0:3,3] = t_vi_i
    return H_vi
