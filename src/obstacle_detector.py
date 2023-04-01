import cv2
import numpy as np

def ResizeWithAspectRatio(image, width=None, height=None, inter=cv2.INTER_AREA):
    dim = None
    (h, w) = image.shape[:2]

    if width is None and height is None:
        return image
    if width is None:
        r = height / float(h)
        dim = (int(w * r), height)
    else:
        r = width / float(w)
        dim = (width, int(h * r))

    return cv2.resize(image, dim, interpolation=inter)

def show_img(img):
    img_scaled = ResizeWithAspectRatio(img, height=800)
    cv2.imshow("image",img_scaled)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

class ObstacleDetector:
    def __init__(self):
        self.image_path = "a_cylinder.jpg"
        self.img_gray = cv2.imread(self.image_path, cv2.IMREAD_GRAYSCALE)
        self.img_blur = cv2.GaussianBlur(self.img_gray, (5,5), 0) # Gaussian blur
        self.img_canny = None

        self.rho = 1  # distance resolution in pixels of the Hough grid
        self.theta = np.pi / 180  # angular resolution in radians of the Hough grid
        self.threshold = 15  # minimum number of votes (intersections in Hough grid cell)
        self.min_line_length = 200  # minimum number of pixels making up a line
        self.max_line_gap = 20  # maximum gap in pixels between connectable line segments
        self.img_vert_lines = np.copy(self.img_gray) * 0  # creating a blank to draw lines on

        self.vert_constraint = 30 # max distance between x points for a vertical line
        self.vert_lines = []

    def detect_obstacles(self):
        # Canny edge detection
        self.img_canny = cv2.Canny(image=self.img_blur, threshold1=0, threshold2=200)
        lines = cv2.HoughLinesP(self.img_canny, self.rho, self.theta, self.threshold, np.array([]),
                    self.min_line_length, self.max_line_gap)

        for line in lines:
            for x1,y1,x2,y2 in line:
                if np.abs(x1-x2) < self.vert_constraint:
                    self.vert_lines.append(line)
                    cv2.line(self.img_vert_lines,(x1,y1),(x2,y2),(255,0,0),5)

        # Draw the lines on the  image
        self.img_gray_vert_lines = cv2.addWeighted(self.img_gray, 0.8, self.img_vert_lines, 1, 0)

if __name__ == "__main__":
    OD = ObstacleDetector()
    OD.detect_obstacles()
    show_img(OD.img_gray_vert_lines)