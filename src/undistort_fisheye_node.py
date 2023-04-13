#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Image, CameraInfo
from threading import Lock
import cv2
import numpy as np
from cv_bridge import CvBridge
from message_filters import TimeSynchronizer, Subscriber

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

CAM_INFO1_ORIGINAL = None
CAM_INFO2_ORIGINAL = None
CAM_INFO1_MODIFIED = None
CAM_INFO2_MODIFIED = None

BRIDGE = CvBridge()

DROP_FRAMES = 3
DROP_IND = 0

def img_cb1(msg):
    img_cb(msg, UNDISTORT_PUB1, MAPX1, MAPY1)

def img_cb2(msg):
    img_cb(msg, UNDISTORT_PUB2, MAPX2, MAPY2)

def img_cb(img_msg1, img_msg2, camera_info_msg1, camera_info_msg2):
    global DROP_IND
    check_none = (
        MAPX1, 
        MAPX2,
        MAPY1,
        MAPY2,
        UNDISTORT_PUB1,
        UNDISTORT_PUB2,
        CAMERA_INFO_PUB1,
        CAMERA_INFO_PUB2,
    )
    if any([x is None for x in check_none]):
        rospy.logwarn("Waiting for camera info")
        return
    # drop frames to reduce load
    DROP_IND += 1
    if DROP_IND < DROP_FRAMES:
        return
    DROP_IND = 0

    img_distorted1 = BRIDGE.imgmsg_to_cv2(img_msg1, desired_encoding="passthrough")
    img_undistorted1 = cv2.remap(
        img_distorted1,
        MAPX1,
        MAPY1,
        interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
    )
    img_distorted2 = BRIDGE.imgmsg_to_cv2(img_msg2, desired_encoding="passthrough")
    img_undistorted2 = cv2.remap(
        img_distorted2,
        MAPX2,
        MAPY2,
        interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
    )
    # crop top and bottom based on DOWNSCALE_H
    orig_height = img_undistorted1.shape[0]
    new_height = orig_height//DOWNSCALE_H

    # take center of image of new height
    img_undistorted1 = img_undistorted1[
        (orig_height - new_height)//2 : (orig_height + new_height)//2, :
    ]
    img_undistorted2 = img_undistorted2[
        (orig_height - new_height)//2 : (orig_height + new_height)//2, :
    ]
    
    # convert from mono8 to bgr8
    img_undistorted = cv2.cvtColor(img_undistorted, cv2.COLOR_GRAY2BGR)
    output_msg1 = BRIDGE.cv2_to_imgmsg(img_undistorted1, encoding="bgr8")
    output_msg1.header = img_msg1.header
    output_msg2 = BRIDGE.cv2_to_imgmsg(img_undistorted2, encoding="bgr8")
    output_msg2.header = img_msg2.header

    # update camera info
    CAM_INFO1_MODIFIED.header = camera_info_msg1.header
    CAM_INFO2_MODIFIED.header = camera_info_msg2.header

    # publish
    UNDISTORT_PUB1.publish(output_msg1)
    UNDISTORT_PUB2.publish(output_msg2)
    CAMERA_INFO_PUB1.publish(CAM_INFO1_MODIFIED)
    CAMERA_INFO_PUB2.publish(CAM_INFO2_MODIFIED)

def camera_info_cb1(msg):
    global CAM_INFO1_MODIFIED, CAM_INFO1_ORIGINAL
    if CAM_INFO1_MODIFIED is not None:
        return
    CAM_INFO1_ORIGINAL = copy.deepcopy(msg)
    msg = modify_camera_info(msg)
    CAM_INFO1_MODIFIED = copy.deepcopy(msg)

def camera_info_cb2(msg):
    global CAM_INFO2_MODIFIED, CAM_INFO2_ORIGINAL
    if CAM_INFO2_MODIFIED is not None:
        return
    CAM_INFO2_ORIGINAL = copy.deepcopy(msg)
    msg = modify_camera_info(msg)
    CAM_INFO2_MODIFIED = copy.deepcopy(msg)

def modify_camera_info(msg):
    msg.distortion_model = "plumb_bob"
    msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    # downscale K and P
    msg.K = list(msg.K)
    msg.K[5] = msg.K[5]/DOWNSCALE_H # optical center
    msg.P = list(msg.P)
    msg.P[6] = msg.P[6]/DOWNSCALE_H # optical center
    msg.height = msg.height//DOWNSCALE_H
    return msg

def init_maps():
    global MAPX1, MAPY1, MAPX2, MAPY2
    K1 = np.array(CAM_INFO1_ORIGINAL.K).reshape(3,3)
    D1 = np.array(CAM_INFO1_ORIGINAL.D)
    K2 = np.array(CAM_INFO2_ORIGINAL.K).reshape(3,3)
    D2 = np.array(CAM_INFO2_ORIGINAL.D)
    T = np.array([BASELINE, 0, 0]) # 64 mm baseline

    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        K1, D1, K2, D2, IMG_SIZE_WH, R=np.eye(3), T=T
    )
    print(K1, D1)
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
    sync = TimeSynchronizer(
        [
            Subscriber("/camera/fisheye1/image_raw", Image),
            Subscriber("/camera/fisheye2/image_raw", Image),
            Subscriber("/camera/fisheye1/camera_info", CameraInfo), # unused
            Subscriber("/camera/fisheye2/camera_info", CameraInfo), # unused
        ],
        10,
        0.05,
    )
    sync.registerCallback(img_cb)
    # rospy.Subscriber("/camera/fisheye1/image_raw", Image, img_cb1)
    # rospy.Subscriber("/camera/fisheye1/camera_info", CameraInfo, camera_info_cb1)
    # rospy.Subscriber("/camera/fisheye2/image_raw", Image, img_cb2)
    # rospy.Subscriber("/camera/fisheye2/camera_info", CameraInfo, camera_info_cb2)

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
        if CAM_INFO1_ORIGINAL is not None and CAM_INFO2_ORIGINAL is not None and MAPX1 is None:
            init_maps()
            print("initialiuzed maps")
        rate.sleep()


if __name__ == "__main__":
    main()
