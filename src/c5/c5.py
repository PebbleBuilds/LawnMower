# import pytesseract
# import cv2
# from pytesseract import Output

# img = cv2.imread('characters.png')
# d = pytesseract.img_to_data(img, output_type=Output.DICT)
# n_boxes = len(d['level'])
# for i in range(n_boxes):
#     (text,x,y,w,h) = (d['text'][i],d['left'][i],d['top'][i],d['width'][i],d['height'][i])
#     cv2.rectangle(img, (x,y), (x+w,y+h) , (0,255,0), 2)
# cv2.imshow('img',img)
# cv2.waitKey(0)


import cv2
import numpy as np

def get_whitespace(img):
    # Convert to HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define lower and upper thresholds for red color
    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)

    lower_red = np.array([170, 50, 50])
    upper_red = np.array([180, 255, 255])
    mask2 = cv2.inRange(hsv, lower_red, upper_red)

    # Combine masks
    mask = cv2.bitwise_or(mask1, mask2)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    print(len(contours))
    # Draw bounding boxes around contours with red color
    for contour in contours:
        x,y,w,h = cv2.boundingRect(contour)
        if w > 10 and h > 10:
            cv2.rectangle(img, (x,y), (x+w,y+h), (0,0,255), 2)

    x_min = contours[9][:,0][:,0].min()
    x_max = contours[9][:,0][:,0].max()
    y_min = contours[9][:,0][:,1].min()
    y_max = contours[9][:,0][:,1].max()
    x_pad = 50
    y_pad = 100
    return img[y_min+y_pad:y_max-y_pad, x_min+x_pad:x_max-x_pad]

def bound_character(img):
    # Define threshold for black color
    black_threshold = 30
    padding = 10

    # Initialize list to store coordinates of black pixels
    black_pixels = []

    # Loop through each pixel in the img
    for i in range(img.shape[0]):
        for j in range(img.shape[1]):
            # Get color value of pixel
            color = img[i,j]

            # Check if color is close to black
            if (color[0] < black_threshold) and (color[1] < black_threshold) and (color[2] < black_threshold):
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
img = cv2.imread('alien_letters.png')
img = get_whitespace(img)
# img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

chunks = np.array_split(img, 4, axis=1)


for i in range(len(chunks)):
    print(chunks[i].shape)


    
templates = {i: cv2.imread(f'{i}.png', cv2.IMREAD_GRAYSCALE) for i in range(1,7)}

threshold = 0.3

# Loop through each segmented character and perform template matching
for character in chunks:
    match_scores = []
    character = cv2.cvtColor(bound_character(character), cv2.COLOR_BGR2GRAY)
    # Loop through each template and calculate the matching score
    for key, template in templates.items():
        template = cv2.resize(template, (character.shape[1], character.shape[0]))
        display(np.hstack((template,character)))

        res = cv2.matchTemplate(character, template, cv2.TM_CCOEFF_NORMED)
        match_score = np.max(res)
        match_scores.append(match_score)
        
    # Find the best matching template based on the highest score
    best_match_idx = np.argmax(match_scores)
    best_match_score = match_scores[best_match_idx]
    
    # Check if the best match score is above the threshold
    if best_match_score >= threshold:
        best_match_key = list(templates.keys())[best_match_idx]
        print('Detected character:', best_match_key)
    else:
        print('No match found')
