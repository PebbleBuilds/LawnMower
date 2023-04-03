import cv2
import numpy as np
from img_utils import *

class ObstacleDetectorColour:
    def __init__(self):
        self.image_path = "a_cylinder.jpg"
        self.img_dict = {}
        self.img_dict["colour_bgr"] = cv2.imread(self.image_path, cv2.IMREAD_COLOR)

        # Yellow mask
        self.yellow_min = np.array([22,50,50])
        self.yellow_max = np.array([30,255,255])

        # Blob detector
        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()
        # Change thresholds
        params.minThreshold = 10
        params.maxThreshold = 200
        # Filter by Area.
        params.filterByArea = True
        params.minArea = 1500
        params.maxArea = 10000000
        # Filter by Circularity
        params.filterByCircularity = False
        params.minCircularity = 0.1
        params.maxCircularity = 0.765
        # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.87
        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.01
        self.blob_detector = cv2.SimpleBlobDetector_create(params)

    def detect_obstacles(self):
        self.img_dict["colour_hsv"] = cv2.cvtColor(self.img_dict["colour_bgr"], cv2.COLOR_BGR2HSV)
        self.img_dict["mask_yellow"] = cv2.inRange(self.img_dict["colour_hsv"], self.yellow_min, self.yellow_max)
        
        self.blobs = self.blob_detector.detect(cv2.bitwise_not(self.img_dict["mask_yellow"]))
        self.img_dict["blobs_yellow"] = cv2.drawKeypoints(self.img_dict["mask_yellow"], self.blobs, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

if __name__ == "__main__":
    ODC = ObstacleDetectorColour()
    ODC.detect_obstacles()
    
    show_img(ODC.img_dict["blobs_yellow"])