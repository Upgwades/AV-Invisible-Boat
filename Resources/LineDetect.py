import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import math
import cv2
import os

# ------ Region of Interest -----
def regionOfInterest(image):

	# Top Left, Bottom Left, Bottom Right, Top Right
	vertices = np.array([[(250,400),(150, image.shape[0]), (image.shape[1]-150,image.shape[0]), (450,400)]], dtype=np.int32)

	mask = np.zeros_like(image)

	if len (image.shape) > 2:
		channel_count = image.shape[2]
		ignore_color = (255,) * channel_count
	else:
		ignore_color = 255

	cv2.fillPoly(mask, vertices, ignore_color)
	maskedImage = cv2.bitwise_and(image, mask)

	#displayImage(maskedImage)

	return maskedImage

# --------------- Color Detection ---------------
def colorFilter(image):
	hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

	# Blues
	blue_lower_range = np.array([85, 35, 0], dtype=np.uint8)
	blue_upper_range = np.array([150, 255, 255], dtype=np.uint8)

	blueMask = cv2.inRange(hsv_img, blue_lower_range, blue_upper_range)
	blueFilteredImage = cv2.bitwise_and(image, image, mask = blueMask)

	return blueFilteredImage

# ------ Image Processing -----
def cleanImage(rawImage):

	# Gaussian Blur to Remove Noise
	blurredImage = blur(rawImage, 13)

	# Crop out all the crap
	croppedImage = regionOfInterest(blurredImage)

	# Filter out unnecessary colors
	filteredImage = colorFilter(croppedImage)

	#displayImage(filteredImage)

	# Convert to Greyscale
	cleanedImage = convertGreyscale(filteredImage)

	return cleanedImage

# ------ Helper Functions -----
def convertGreyscale(image):
	return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

def canny(image, low, high):
	return cv2.Canny(image, low, high)

def blur(image, size):
	return cv2.GaussianBlur(image, (size, size), 0)

def genColorMask(image):
	color_thresholds = (image[:,:,0] < white_threshold[0]) | (image[:,:,1] < white_threshold[1]) | (image[:,:,2] < white_threshold[2])
	return color_thresholds

def displayImage(image):
	cv2.namedWindow("Color Detection", cv2.WINDOW_NORMAL)
	cv2.resizeWindow('Color Detection', 1920,1080)
	cv2.imshow("Color Detection", image)
	cv2.waitKey(0)

# ------ Main -----
rawImage = cv2.imread('Blue Line.jpg', 1)
overlayImage = rawImage.copy()

# Set contour threashold. 0 for everything, since the image has already been cleaned
ret, threshold = cv2.threshold(cleanImage(rawImage), 0, 255, 0)

# Find the contours
contourMask, contours, hierarchy = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# Find the center of the contour region, no idea wtf this does, but it works
lineRegion = contours[0]
imageMoment = cv2.moments(lineRegion)
centerX = int(imageMoment['m10'] / imageMoment['m00'])
centerY = int(imageMoment['m01'] / imageMoment['m00'])

# Draw the contours on the image
cv2.drawContours(overlayImage, contours, -1, (0,255,0), 5)

# Draw circle at the center of the contour
cv2.circle(overlayImage, (centerX, centerY), 7, (0, 0, 255), -1)

# Display the image
displayImage(overlayImage)
