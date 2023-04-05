#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Image
from threading import Lock
import cv2
import numpy as np
from cv_bridge import CvBridge


H, W = 800, 848
img_size_WH = (W, H)
downsized_img_size_WH = (W // 2, H // 2)

# TODO initialize IMG1 and IMG2 to empty images
IMG1 = None
IMG2 = None

# TODO initialize the following params with the streamed camera info, support arbitrary hardware
# Camera intrinsics
K1 = np.array([
    286.1167907714844,  0.0,                421.62689208984375, 
    0.0,                286.27880859375,    399.5252990722656, 
    0.0,                0.0,                1.0
]).reshape(3, 3)
K2 = np.array([
    286.1825866699219,  0.0,                425.0535888671875, 
    0.0,                286.18560791015625, 398.7244873046875, 
    0.0,                0.0,                1.0
]).reshape(3, 3)
# Distortion params
D1 = np.array([-0.008292312733829021, 0.0453055202960968, -0.04263167083263397, 0.007736437954008579])
D2 = np.array([-0.00701235281303525, 0.04197357967495918, -0.03934945911169052, 0.006663409061729908])


# TODO put params in launch file
min_disp = 0
max_disp = 16
num_disp = max_disp - min_disp
block_size = 21
disp12_max_diff = 1
uniqueness_ratio = 10
speckle_window_size = 100
speckle_range = 1


stereo = cv2.StereoBM_create(numDisparities=num_disp, blockSize=block_size)

# From camera infos
R1 = np.eye(3)
R2 = np.eye(3)
P1 = np.array([
    286.1167907714844,  0.0,                421.62689208984375, 0.0,
    0.0,                286.27880859375,    399.5252990722656,  0.0, 
    0.0,                0.0,                1.0,                0.0
]).reshape(3, 4)
P2 = np.array([
    286.1825866699219,  0.0,                425.0535888671875,  -18.2928466796875, 
    0.0,                286.18560791015625, 398.7244873046875,  -0.01925480365753174, 
    0.0,                0.0,                1.0,                0.0
]).reshape(3, 4)

T = np.zeros(3)
T[0] = -P2[0, 3] / P2[0, 0] # 64 mm baseline

m1type = cv2.CV_32FC1
map1x, map1y = cv2.fisheye.initUndistortRectifyMap(
    K1, D1, R1, P1, img_size_WH, m1type
)
map2x, map2y = cv2.fisheye.initUndistortRectifyMap(
    K2, D2, R2, P2, img_size_WH, m1type
)

frame_mutex = Lock()

bridge = CvBridge()


def callback_fisheye1(msg):
    global IMG1    
    IMG1 = msg


def callback_fisheye2(msg):
    global IMG2
    IMG2 = msg


def main():
    print("entered")
    rospy.init_node("t265_stereo_node")
    # subscribers
    rospy.Subscriber("/camera/fisheye1/image_raw", Image, callback_fisheye1)
    rospy.Subscriber("/camera/fisheye2/image_raw", Image, callback_fisheye2)
    # assume that the fisheye cameras are synchronized

    # publishers
    stereo_pub = rospy.Publisher("/camera/stereo/image_raw", Image, queue_size=1)
    print("Publishing stereo image on /camera/stereo/image_raw")
    window_name = "DEBUG"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    while not rospy.is_shutdown():
        # obtain images
        frame_mutex.acquire()
        # TODO convert IMG1 and IMG2 to cv2 images from ROS
        if IMG1 is None or IMG2 is None:
            frame_mutex.release()
            continue
        img1 = bridge.imgmsg_to_cv2(IMG1, desired_encoding="passthrough").copy()
        img2 = bridge.imgmsg_to_cv2(IMG2, desired_encoding="passthrough").copy()
        header = IMG1.header
        frame_mutex.release()
        # undistort images, crop center of frames
        img1 = cv2.remap(img1, map1x, map1y, interpolation=cv2.INTER_LINEAR)
        img2 = cv2.remap(img2, map2x, map2y, interpolation=cv2.INTER_LINEAR)
        # downscale img1, img2 by half
        img1_small = cv2.resize(img1, downsized_img_size_WH)
        img2_small = cv2.resize(img2, downsized_img_size_WH)
        # compute disparity on the center of frames, convert to pixel disparity
        disparity = stereo.compute(img1_small, img2_small).astype(np.float32) / 16.0
        # scale back up disparity
        disparity = cv2.resize(disparity, img_size_WH)
        # recrop valid disparity
        disparity = disparity[:, max_disp:]
        # convert disparity to pixel intensitites
        disparity = 255 * (disparity - min_disp) / num_disp
        # convert to color
        disp_vis = cv2.applyColorMap(
            cv2.convertScaleAbs(disparity, 1), cv2.COLORMAP_JET
        )

        color_img = cv2.cvtColor(img1[:, max_disp:], cv2.COLOR_GRAY2BGR)

        # publish debug image
        cv2.imshow(window_name, np.hstack((color_img, disp_vis)))
        cv2.waitKey(1)
        
        # project disparity to 3D
        # points = cv2.reprojectImageTo3D(disparity, Q)
        
        # # convert to correct msg type
        # pub_msg = Image()
        # pub_msg = bridge.cv2_to_imgmsg(points, encoding="passthrough")
        # pub_msg.header = header
        # stereo_pub.publish(pub_msg)



if __name__ == "__main__":
    main()
