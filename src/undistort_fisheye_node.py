#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Image, CameraInfo
from threading import Lock
import cv2
import numpy as np
from cv_bridge import CvBridge

from constants import *

UNDISTORT_PUB = None
CAMERA_INFO_PUB = None

MAPX = None
MAPY = None
K = None
D = None

BRIDGE = CvBridge()

WINDOW = cv2.namedWindow("undistorted", cv2.WINDOW_NORMAL)
WINDOW2 = cv2.namedWindow("distorted", cv2.WINDOW_NORMAL)

def img_cb(msg):
    if MAPX is None or MAPY is None or UNDISTORT_PUB is None or CAMERA_INFO_PUB is None:
        return
    img_distorted = BRIDGE.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    img_undistorted = cv2.remap(
        img_distorted,
        MAPX,
        MAPY,
        interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
    )
    UNDISTORT_PUB.publish(
        BRIDGE.cv2_to_imgmsg(img_undistorted, encoding="passthrough")
    )


def camera_info_cb(msg):
    global MAPX, MAPY, K, D
    if MAPX is None:
        # don't rectify
        K = np.array(msg.K).reshape(3,3)
        D = np.array(msg.D)
        R = np.array(msg.R).reshape(3,3)
        P = np.array(msg.P).reshape(3,4)
        MAPX, MAPY = cv2.fisheye.initUndistortRectifyMap(
            K, D, R, P, size=STEREO_SIZE_WH, m1type=cv2.CV_32FC1
        )
    msg.distortion_model = "plumb_bob"
    msg.D = [0, 0, 0, 0, 0]
    CAMERA_INFO_PUB.publish(msg)


def main():
    global UNDISTORT_PUB, CAMERA_INFO_PUB
    rospy.init_node("undistort_img_node")
    # subscribers
    rospy.Subscriber("fisheye/image_raw", Image, img_cb)
    rospy.Subscriber("fisheye/camera_info", CameraInfo, camera_info_cb)
    # publishers
    UNDISTORT_PUB = rospy.Publisher("undistorted/image_raw", Image, queue_size=1)
    CAMERA_INFO_PUB = rospy.Publisher(
        "undistorted/camera_info", CameraInfo, queue_size=1
    )

    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == "__main__":
    main()
