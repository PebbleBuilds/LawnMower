#!/usr/bin/env python

import numpy as np
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
import rospy
from constants import *


def create_posestamped(
    pose_xyz=[0, 0, 0], orientation=[0, 0, 0, 1], frame_id=VICON_DUMMY_FRAME_ID
):
    pose_stamped = PoseStamped()
    assert len(pose_xyz) == 3, "Pose must be a 3D vector"
    assert len(orientation) == 4, "Orientation must be a 4D quaternion vector"
    x_pose, y_pose, z_pose = pose_xyz
    pose_stamped.pose.position.x = x_pose
    pose_stamped.pose.position.y = y_pose
    pose_stamped.pose.position.z = z_pose
    x_o, y_o, z_o, w_o = orientation
    pose_stamped.pose.orientation.x = x_o
    pose_stamped.pose.orientation.y = y_o
    pose_stamped.pose.orientation.z = z_o
    pose_stamped.pose.orientation.w = w_o

    pose_stamped.header.frame_id = frame_id
    pose_stamped.header.stamp = rospy.Time(0)
    return pose_stamped

def tfstamped2posestamped(tfstamped):
    posestamped = PoseStamped()
    posestamped = tf2_geometry_msgs.do_transform_pose(posestamped, tfstamped)
    return posestamped

def tfstamped2np(transformstamped, include_time=False):
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
        return np.concatenate(pose_np, stamp)  # (x,y,z,stamp)
    else:
        return pose_np


def pose2np(pose):
    return np.array([pose.position.x, pose.position.y, pose.position.z])


def np2posestamped(pose):
    return create_posestamped(pose)

def get_yaw(point1, point2):
    dx = point2[0] - point1[0]
    dy = point2[1] - point1[1]
    
    yaw = np.arctan2(dy, dx)
    
    return yaw

def euler_to_quaternion(roll, pitch, yaw):
    """
    Converts Euler angles to quaternion
    :param roll: Roll angle in radians
    :param pitch: Pitch angle in radians
    :param yaw: Yaw angle in radians
    :return: Quaternion tuple (w, x, y, z)
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr

    return [w, x, y, z]

def quaternion_to_euler(q):
    """
    Converts a quaternion to Euler angles
    :param q: Quaternion in the form [w, x, y, z]
    :return: Euler angles in radians in the form [roll, pitch, yaw]
    """
    # Normalize the quaternion
    q_norm = q / np.linalg.norm(q)

    # Extract the quaternion components
    w = q_norm[0]
    x = q_norm[1]
    y = q_norm[2]
    z = q_norm[3]

    # Calculate the Euler angles
    roll = np.arctan2(2*(w*x + y*z), 1-2*(x**2 + y**2))
    pitch = np.arcsin(2*(w*y - z*x))
    yaw = np.arctan2(2*(w*z + x*y), 1-2*(y**2 + z**2))

    return roll,pitch,yaw