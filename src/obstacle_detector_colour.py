import cv2
import numpy as np
from img_utils import *

class ObstacleDetectorColour:
    def __init__(self):
        self.image_path = "bcd_cylinder.jpg"
        self.img_dict = {}
        self.img_dict["colour_bgr"] = cv2.pyrDown(cv2.imread(self.image_path, cv2.IMREAD_COLOR))
        print(self.img_dict["colour_bgr"].shape)

        # Yellow mask
        self.yellow_min = np.array([22,50,50])
        self.yellow_max = np.array([30,255,255])

        # Rectangle detector

    def detect_obstacles(self):
        self.img_dict["colour_hsv"] = cv2.cvtColor(self.img_dict["colour_bgr"], cv2.COLOR_BGR2HSV)
        self.img_dict["mask_yellow"] = cv2.inRange(self.img_dict["colour_hsv"], self.yellow_min, self.yellow_max)
        
        contours,hierarchy = cv2.findContours(self.img_dict["mask_yellow"], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            x1,y1 = cnt[0][0]
            approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(cnt)
                ratio = float(w)/h
                self.img_dict["colour_bgr_rects"] = self.img_dict["colour_bgr"].copy()
                if ratio >= 0.9 and ratio <= 1.1:
                    self.img_dict["colour_bgr_rects"] = cv2.drawContours(self.img_dict["colour_bgr_rects"], [cnt], -1, (0,255,255), 3)
                    cv2.putText(self.img_dict["colour_bgr_rects"], 'Square', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                else:
                    cv2.putText(self.img_dict["colour_bgr_rects"], 'Rectangle', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    self.img_dict["colour_bgr_rects"] = cv2.drawContours(self.img_dict["colour_bgr_rects"], [cnt], -1, (0,255,0), 3)

if __name__ == "__main__":
    ODC = ObstacleDetectorColour()
    ODC.detect_obstacles()
    
    show_img(ODC.img_dict["mask_yellow"])