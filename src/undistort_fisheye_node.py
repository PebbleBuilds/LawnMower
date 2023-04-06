#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Image, CameraInfo
from threading import Lock
import cv2
import numpy as np
from cv_bridge import CvBridge

from constants import *
import copy

UNDISTORT_PUB1 = None
CAMERA_INFO_PUB1 = None
UNDISTORT_PUB2 = None
CAMERA_INFO_PUB2 = None

MAPX1 = None
MAPY1 = None
MAPX2 = None
MAPY2 = None

CAM_INFO1 = None
CAM_INFO2 = None

BRIDGE = CvBridge()


def img_cb1(msg):
    img_cb(msg, UNDISTORT_PUB1, MAPX1, MAPY1)

def img_cb2(msg):
    img_cb(msg, UNDISTORT_PUB2, MAPX2, MAPY2)

def img_cb(msg, undistort_pub, mapx, mapy):
    if mapx is None or mapy is None or undistort_pub is None:
        return
    img_distorted = BRIDGE.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    img_undistorted = cv2.remap(
        img_distorted,
        mapx,
        mapy,
        interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
    )
    # downscale
    img_undistorted = cv2.resize(
        img_undistorted, STEREO_SIZE_WH, interpolation=cv2.INTER_AREA
    )
    # convert from mono8 to bgr8
    img_undistorted = cv2.cvtColor(img_undistorted, cv2.COLOR_GRAY2BGR)
    output_msg = BRIDGE.cv2_to_imgmsg(img_undistorted, encoding="bgr8")
    output_msg.header = msg.header
    undistort_pub.publish(output_msg)

def camera_info_cb1(msg):
    global CAM_INFO1
    CAM_INFO1 = copy.deepcopy(msg)
    camera_info_cb(msg, CAMERA_INFO_PUB1)

def camera_info_cb2(msg):
    global CAM_INFO2
    CAM_INFO2 = copy.deepcopy(msg)
    camera_info_cb(msg, CAMERA_INFO_PUB2)

def camera_info_cb(msg, camera_info_pub):
    msg.distortion_model = "plumb_bob"
    msg.D = [0, 0, 0, 0, 0]
    msg.K = [msg.K[0]/DOWNSCALE, 0, msg.K[2]/DOWNSCALE, 
             0, msg.K[4]/DOWNSCALE, msg.K[5]/DOWNSCALE,
               0, 0, 1]
    camera_info_pub.publish(msg)

def init_maps():
    global MAPX1, MAPY1, MAPX2, MAPY2
    K1 = np.array(CAM_INFO1.K).reshape(3,3)
    D1 = np.array(CAM_INFO1.D)
    K2 = np.array(CAM_INFO2.K).reshape(3,3)
    D2 = np.array(CAM_INFO2.D)
    T = np.array([BASELINE, 0, 0]) # 64 mm baseline

    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        K1, D1, K2, D2, IMG_SIZE_WH, R=np.eye(3), T=T
    )
    MAPX1, MAPY1 = cv2.fisheye.initUndistortRectifyMap(
        K1, D1, R1, P1, size=IMG_SIZE_WH, m1type=cv2.CV_32FC1
    )
    MAPX2, MAPY2 = cv2.fisheye.initUndistortRectifyMap(
        K2, D2, R2, P2, size=IMG_SIZE_WH, m1type=cv2.CV_32FC1
    )

def main():
    global UNDISTORT_PUB1, CAMERA_INFO_PUB1, UNDISTORT_PUB2, CAMERA_INFO_PUB2
    rospy.init_node("undistort_img_node")
    # subscribers
    rospy.Subscriber("/camera/fisheye1/image_raw", Image, img_cb1)
    rospy.Subscriber("/camera/fisheye1/camera_info", CameraInfo, camera_info_cb1)
    rospy.Subscriber("/camera/fisheye2/image_raw", Image, img_cb2)
    rospy.Subscriber("/camera/fisheye2/camera_info", CameraInfo, camera_info_cb2)
    # publishers
    UNDISTORT_PUB1 = rospy.Publisher("undistorted1/image_raw", Image, queue_size=1)
    CAMERA_INFO_PUB1 = rospy.Publisher(
        "undistorted1/camera_info", CameraInfo, queue_size=1
    )
    UNDISTORT_PUB2 = rospy.Publisher("undistorted2/image_raw", Image, queue_size=1)
    CAMERA_INFO_PUB2 = rospy.Publisher(
        "undistorted2/camera_info", CameraInfo, queue_size=1
    )
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if CAM_INFO1 is not None and CAM_INFO2 is not None and MAPX1 is None:
            init_maps()
        rate.sleep()


if __name__ == "__main__":
    main()
