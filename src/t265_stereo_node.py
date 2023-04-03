import rospy

from sensor_msgs.msg import Image
from threading import Lock
import cv2
import numpy as np

def camera_matrix(intrinsics):
    return np.array(
        [
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy],
            [0, 0, 1],
        ]
    )


def fisheye_distortion(intrinsics):
    return np.array(intrinsics.coeffs[:4])

# TODO initialize IMG1 and IMG2 to empty images
IMG1 = None
IMG2 = None

INTRINSICS1 = None
INTRINSICS2 = None

K1 = camera_matrix(INTRINSICS1)
K2 = camera_matrix(INTRINSICS2)
D1 = fisheye_distortion(INTRINSICS1)
D2 = fisheye_distortion(INTRINSICS2)

# extrinsics
R = None
T = None
R1 = np.eye(3)
R2 = R

H, W = 480, 848

# TODO put params in launch file
min_disp = 0
max_disp = 112
num_disp = max_disp - min_disp
block_size = 5
disp12_max_diff = 1
uniqueness_ratio = 10
speckle_window_size = 100
speckle_range = 1

stereo = cv2.StereoBM_create(
    minDisparity=min_disp,
    numDisparities=num_disp,
    blockSize=block_size,
    P1=8 * block_size**2,
    P2=32 * block_size**2,
    disp12MaxDiff=disp12_max_diff,
    uniquenessRatio=uniqueness_ratio,
    speckleWindowSize=speckle_window_size,
    speckleRange=speckle_range,
)

stereo_fov_rad = np.pi / 2.0  # 90 degrees
stereo_height_px = 480
stereo_focal_px = stereo_height_px / (2.0 * np.tan(stereo_fov_rad / 2.0))
stereo_width_px = stereo_height_px + max_disp
stereo_size = (stereo_width_px, stereo_height_px)
stereo_cx = (stereo_height_px - 1) / 2.0 + max_disp
stereo_cy = (stereo_height_px - 1) / 2.0

P1 = np.array(
    [
        [stereo_focal_px, 0, stereo_cx, 0],
        [0, stereo_focal_px, stereo_cy, 0],
        [0, 0, 1, 0],
    ]
)
P2 = P1.copy()
P2[0, 3] = T[0] * stereo_focal_px

Q = np.array(
    [
        [1, 0, 0, -(stereo_cx - max_disp)],
        [0, 1, 0, -stereo_cy],
        [0, 0, 0, stereo_focal_px],
        [0, 0, 1 / T[0], 0],
    ]   
)

m1type = cv2.CV_32FC1
map1x, map1y = cv2.fisheye.initUndistortRectifyMap(
    K1, D1, R1, P1, stereo_size, m1type
)
map2x, map2y = cv2.fisheye.initUndistortRectifyMap(
    K2, D2, R2, P2, stereo_size, m1type
)


frame_mutex = Lock()

def callback_fisheye1(msg):
    global IMG1    
    IMG1 = msg


def callback_fisheye2(msg):
    global IMG2
    IMG2 = msg


def main():
    rospy.init_node("t265_stereo_node")
    # subscribers
    rospy.Subscriber("/camera/fisheye1/image_raw", Image, callback_fisheye1)
    rospy.Subscriber("/camera/fisheye2/image_raw", Image, callback_fisheye2)
    # assume that the fisheye cameras are synchronized

    # publishers
    stereo_pub = rospy.Publisher("/camera/stereo/image_raw", Image, queue_size=1)
    window_name = "DEBUG"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    while not rospy.is_shutdown():
        # obtain images
        frame_mutex.acquire()
        # TODO convert IMG1 and IMG2 to cv2 images from ROS
        img1 = IMG1.copy()
        img2 = IMG2.copy()
        header = IMG1.header
        frame_mutex.release()
        # undistort images, crop center of frames
        img1 = cv2.remap(img1, map1x, map1y, interpolation=cv2.INTER_LINEAR)
        img2 = cv2.remap(img2, map2x, map2y, interpolation=cv2.INTER_LINEAR)
        # compute disparity on the center of frames, convert to pixel disparity
        disparity = stereo.compute(img1, img2).astype(np.float32) / 16.0
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
        points = cv2.reprojectImageTo3D(disparity, Q)
        
        # convert to correct msg type
        pub_msg = Image()
        pub_msg.header = header
        pub_msg.height = points.shape[0]
        pub_msg.width = points.shape[1]
        pub_msg.encoding = "32FC3"
        pub_msg.is_bigendian = False
        pub_msg.step = points.shape[1] * 3 * 4
        pub_msg.data = points.tostring()
        stereo_pub.publish(pub_msg)



if __name__ == "__main__":
    main()
