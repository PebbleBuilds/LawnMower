import cv2
import numpy as np

DEBUG = False

def get_whitespace(img):
    x_pad = 50
    y_pad = 100
    _, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)

    # Find contours
    contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Find largest gray contour
    max_contour = None
    max_area = 0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > max_area: # and cv2.mean(gray, mask=contour)[0] <= 200:
            max_contour = contour
            max_area = area

    # Draw bounding box around largest gray contour with green color
    x,y,w,h = cv2.boundingRect(max_contour)
    cv2.rectangle(img, (x,y), (x+w,y+h), (0,255,0), 2)


    # Crop image using bounding box of largest gray contour
    cropped = img[y+y_pad:y+h-y_pad, x+x_pad:x+w-x_pad]

    # x_min = contours[9][:,0][:,0].min()
    # x_max = contours[9][:,0][:,0].max()
    # y_min = contours[9][:,0][:,1].min()
    # y_max = contours[9][:,0][:,1].max()
    
    return cropped

def bound_character(img):
    # Define threshold for black color
    black_threshold = 30
    padding = 0

    # Initialize list to store coordinates of black pixels
    black_pixels = []

    # Loop through each pixel in the img
    for i in range(img.shape[0]):
        for j in range(img.shape[1]):
            # Get color value of pixel
            color = img[i,j]

            # Check if color is close to black
            if (color < black_threshold):
                # Pixel is black, store its coordinates
                black_pixels.append([i,j])


    black_pixels = np.array(black_pixels)
    x_min = black_pixels[:,0].min()
    x_max = black_pixels[:,0].max()
    y_min = black_pixels[:,1].min()
    y_max = black_pixels[:,1].max()
    return img[x_min-padding:x_max+padding, y_min-padding:y_max+padding]

def display(img):
    cv2.imshow('Image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Load img
img = cv2.imread('test4.jpg')
if DEBUG:
    display(img)

img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
img = get_whitespace(img)
chunks = np.array_split(img, 4, axis=1)

if DEBUG:
    display(img)
    for chunk in chunks:
        display(chunk)
    
templates = {i: cv2.imread(f'{i}.png', cv2.IMREAD_GRAYSCALE) for i in range(1,7)}

threshold = 0.2
best_match_keys = []
for character in chunks:
    match_scores = []
    character = bound_character(character)
    for key, template in templates.items():
        template = cv2.resize(template, (character.shape[1], character.shape[0]))

        if DEBUG:
            display(np.hstack((template,character)))

        res = cv2.matchTemplate(character, template, cv2.TM_CCOEFF_NORMED)
        match_score = np.max(res)
        match_scores.append(match_score)
        
    best_match_idx = np.argmax(match_scores)
    best_match_score = match_scores[best_match_idx]
    
    best_match_key = list(templates.keys())[best_match_idx]
    best_match_keys.append(best_match_key)
    print("Detected character: ", best_match_key)

print('Detected characters in order:', best_match_keys)
with open("detected_nums.txt", "w") as output:
    output.write(str(best_match_keys))
