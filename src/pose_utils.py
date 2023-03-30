from geometry_msgs.msg import Point, PoseStamped
from tf.transformations import quaternion_matrix
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

def transformstamped2np(transformstamped,include_time=False):
    t_vector = transformstamped.transform.translation
    stamp = transformstamped.header.stamp
    if include_time:
        return np.array([t_vector.x, t_vector.y, t_vector.z, stamp])
    else:
        return np.array([t_vector.x, t_vector.y, t_vector.z])

def posestamped2np(posestamped, include_time=False):
    pose_np = pose2np(posestamped.pose)
    if include_time:
        stamp = posestamped.header.stamp
        return np.concatenate(pose_np, stamp) # (x,y,z,stamp)
    else:
        return pose_np

def pose2np(pose):
    return np.array([pose.position.x, pose.position.y, pose.position.z])

def np2posestamped(pose):
    return create_posestamped(pose)

def vicontf_to_Hvi(vicon_tf):
    H_vi = np.eye(4,4)
    vicon_tf_array = [
        vicon_tf.rotation.x,
        vicon_tf.rotation.y,
        vicon_tf.rotation.z,
        vicon_tf.rotation.w
        ]
    C_vi = quaternion_matrix(vicon_tf_array).T[0:3, 0:3]
    
    t_vi_i = np.array([
        vicon_tf.translation.x,
        vicon_tf.translation.y,
        vicon_tf.translation.z
    ])
    t_iv_v = -np.matmul(C_vi, t_vi_i)
    H_vi[0:3,0:3] = C_vi
    H_vi[0:3,3] = t_iv_v
    return H_vi
