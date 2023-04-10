#!/usr/bin/env python
"""
Dedicated to Alex, who is the best.
"""
import rospy

from sensor_msgs.msg import Image, CameraInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber
from threading import Lock
import cv2
import numpy as np
from cv_bridge import CvBridge

from constants import *

STEREO_PUB = None

MAPX1 = None
MAPY1 = None
MAPX2 = None
MAPY2 = None

BRIDGE = CvBridge()

STEREO_BM = None

Q = None

VISUALIZE = True
STEREO_WINDOW = cv2.namedWindow("stereo", cv2.WINDOW_NORMAL)
IMG_RAW_WINDOW = cv2.namedWindow("raw", cv2.WINDOW_NORMAL)

def stereo_cb(msg1, msg2):
    if any([x is None for x in [MAPX1, MAPY1, MAPX2, MAPY2, STEREO_BM]]):
        return
    img_raw1 = BRIDGE.imgmsg_to_cv2(msg1, desired_encoding="passthrough")
    img_raw2 = BRIDGE.imgmsg_to_cv2(msg2, desired_encoding="passthrough")
    img_undistorted1 = undistort_rectify(img_raw1, MAPX1, MAPY1)
    img_undistorted2 = undistort_rectify(img_raw2, MAPX2, MAPY2)

    # crop image for faster computation
    orig_height = img_undistorted1.shape[0]
    new_height = orig_height // DOWNSCALE_H  # 800//8 = 100
    # take center of image of new height
    img_crop1 = img_undistorted1[
        (orig_height - new_height) // 2 : (orig_height + new_height) // 2, :
    ]
    img_crop2 = img_undistorted2[
        (orig_height - new_height) // 2 : (orig_height + new_height) // 2, :
    ]

    # compute disparity, divide by DISP_SCALE, not sure what that means
    disparity = STEREO_BM.compute(img_crop1, img_crop2).astype(np.float32) / 16.0
    if VISUALIZE:
        visualize_disparity(img_crop1, disparity)

    # # convert to disparity image message (untested)
    # disp_msg = DisparityImage()
    # disp_msg.header = msg1.header
    # disp_msg.image = BRIDGE.cv2_to_imgmsg(disparity, encoding="passthrough")
    # disp_msg.f = 1.0
    # disp_msg.T = BASELINE
    # disp_msg.min_disparity = 0.0  # see minDisparity in stereo_bm
    # disp_msg.max_disparity = 16  # see num_disparities in stereo_bm
    # disp_msg.delta_d = 1.0  # see disp12MaxDiff in stereo_bm

    # # publish disparity image
    # STEREO_PUB.publish(disp_msg)


def undistort_rectify(img, mapx, mapy):
    img_undistorted = cv2.remap(
        img,
        mapx,
        mapy,
        interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
    )
    return img_undistorted


def camera_infos_cb(msg1, msg2):
    global MAPX1, MAPY1, MAPX2, MAPY2, Q
    if not any([x is None for x in [MAPX1, MAPY1, MAPX2, MAPY2]]):
        return
    K1 = np.array(msg1.K).reshape(3, 3)
    K2 = np.array(msg2.K).reshape(3, 3)
    D1 = np.array(msg1.D)
    D2 = np.array(msg2.D)
    R1 = np.array(msg1.R).reshape(3, 3)
    R2 = np.array(msg2.R).reshape(3, 3)
    P1 = np.array(msg1.P).reshape(3, 4)
    P2 = np.array(msg2.P).reshape(3, 4)
    img_w, img_h = msg1.width, msg1.height  # (848, 800)
    img_wh = (img_w, img_h)

    # compute undistortion maps
    MAPX1, MAPY1 = cv2.fisheye.initUndistortRectifyMap(
        K1, D1, R1, P1, img_wh, cv2.CV_32FC1
    )
    MAPX2, MAPY2 = cv2.fisheye.initUndistortRectifyMap(
        K2, D2, R2, P2, img_wh, cv2.CV_32FC1
    )

    T = np.zeros((3, 1))
    # see https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/t265_stereo.py#L197
    # -18.2928466796875/286.1825866699219, computed from msg2.P[3]/msg2.P[0] ^^^
    T[0] = BASELINE
    # adjust K for cropped image
    K1[1, 2] /= DOWNSCALE_H
    K2[1, 2] /= DOWNSCALE_H
    R = np.eye(3)

    # compute Q for cropped image
    _, _, _, _, Q,  = cv2.fisheye.stereoRectify(K1, D1, K2, D2, img_wh, R, T, 0)


def visualize_disparity(img_crop1, disparity):
    # convert disparity to color image, divide by num disparities
    disparity_color = cv2.applyColorMap(
        cv2.convertScaleAbs(disparity, alpha=255 / 16.0), cv2.COLORMAP_JET
    )
    cv2.imshow("stereo", disparity_color)
    cv2.imshow("raw", img_crop1)
    cv2.waitKey(1)


def main():
    global STEREO_PUB, STEREO_BM
    rospy.init_node("undistort_img_node")
    num_disparities = rospy.get_param("~num_disparities", 16)
    block_size = rospy.get_param("~block_size", 15)
    STEREO_BM = cv2.StereoBM_create(
        numDisparities=num_disparities, blockSize=block_size
    )

    # subscribers
    tss_img = ApproximateTimeSynchronizer(
        [
            Subscriber("/camera/fisheye1/image_raw", Image),
            Subscriber("/camera/fisheye2/image_raw", Image),
        ],
        10,  # queue size
        0.05,  # expect 30 fps
    )
    tss_img.registerCallback(stereo_cb)
    tss_cam_info = ApproximateTimeSynchronizer(
        [
            Subscriber("/camera/fisheye1/camera_info", CameraInfo),
            Subscriber("/camera/fisheye2/camera_info", CameraInfo),
        ],
        10,  # queue size
        0.05,  # expect 30 fps
    )
    tss_cam_info.registerCallback(camera_infos_cb)

    # publishers
    # STEREO_PUB = rospy.Publisher(
    #     "/camera/stereo/image_raw", DisparityImage, queue_size=10
    # )
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    main()
