import cv2

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