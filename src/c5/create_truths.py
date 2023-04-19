import cv2
import numpy as np

def isolate(img):
    # Define threshold for black color
    black_threshold = 30
    padding = 0

    # Initialize list to store coordinates of black pixels
    black_pixels = []

    # Loop through each pixel in the image
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

image = cv2.imread('characters.png')

chunks = np.array_split(image, 6, axis=1)

for i in range(len(chunks)):
    cv2.imwrite(f'{i+1}.png', isolate(chunks[i]))
