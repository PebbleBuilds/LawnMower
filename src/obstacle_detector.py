import cv2
import numpy as np
from img_utils import *

class ObstacleDetector:
    def __init__(self):
        self.image_path = "ab_cylinder.jpg"
        self.img_dict = {}
        self.img_dict["gray"] = cv2.imread(self.image_path, cv2.IMREAD_GRAYSCALE)
        self.img_dict["blur"] = cv2.GaussianBlur(self.img_dict["gray"], (5,5), 0) # Gaussian blur
        self.img_canny = None

        # vertical line detection (Canny edges + Hough lines)
        self.rho = 1  # distance resolution in pixels of the Hough grid
        self.theta = np.pi / 180  # angular resolution in radians of the Hough grid
        self.threshold = 15  # minimum number of votes (intersections in Hough grid cell)
        self.min_line_length = 300  # minimum number of pixels making up a line
        self.max_line_gap = 20  # maximum gap in pixels between connectable line segments
        self.img_dict["vert_lines"] = np.copy(self.img_dict["gray"]) * 0  # creating a blank to draw lines on
        self.vert_constraint = 30 # max distance between x points for a vertical line
        self.vert_lines = np.empty((0,4))

        # segmenting based on vertical lines
        self.max_width = 800
        self.min_width = 50
        self.obstacle_imgs = []

    def detect_vert_lines(self):
        # Canny edge detection
        self.img_canny = cv2.Canny(image=self.img_dict["blur"], threshold1=0, threshold2=200)
        lines = cv2.HoughLinesP(self.img_canny, self.rho, self.theta, self.threshold, np.array([]),
                    self.min_line_length, self.max_line_gap)

        for line in lines:
            for x1,y1,x2,y2 in line:
                if np.abs(x1-x2) < self.vert_constraint:
                    if self.vert_lines.size > 0 and np.isclose(self.vert_lines[:,0],line[:,0],atol=50.0).any():
                        continue
                    self.vert_lines = np.vstack([self.vert_lines,line])
                    cv2.line(self.img_dict["vert_lines"],(x1,y1),(x2,y2),(255,0,0),5)

        # Draw the lines on the image
        self.img_dict["gray_vert_lines"] = cv2.addWeighted(self.img_dict["gray"], 0.8, self.img_dict["vert_lines"], 1, 0)

        # Segment images
        self.vert_lines = self.vert_lines[self.vert_lines[:, 0].argsort()] #so xj is always bigger than xi
        for i in range(0,self.vert_lines.shape[0]):
            for j in range(i+1,self.vert_lines.shape[0]):
                xi = int(self.vert_lines[i,0])
                xj = int(self.vert_lines[j,0])
                width =  xj - xi 
                if width > self.min_width and width < self.max_width:
                    self.obstacle_imgs.append(self.img_dict["gray"][:,xi:xj])

if __name__ == "__main__":
    OD = ObstacleDetector()
    OD.detect_vert_lines()
    show_img(OD.img_dict["gray_vert_lines"])
    #print(OD.vert_lines)
    #print(OD.img_dict["gray"].shape)
    for img in OD.obstacle_imgs:
        show_img(img)