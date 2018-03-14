import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import math
import cv2
import os
import imageio

imageio.plugins.ffmpeg.download()

from moviepy.editor import VideoFileClip

frameCounter = 0

# ------ Region of Interest -----
def regionOfInterest(image):

	# Top Left, Bottom Left, Bottom Right, Top Right
	vertices = np.array([[(440,325),(0, image.shape[0]), (image.shape[1],image.shape[0]), (530,325)]], dtype=np.int32)

	mask = np.zeros_like(image)

	if len (image.shape) > 2:
		channel_count = image.shape[2]
		ignore_color = (255,) * channel_count
	else:
		ignore_color = 255

	cv2.fillPoly(mask, vertices, ignore_color)
	masked_image = cv2.bitwise_and(image, mask)

	return masked_image

# ------ Color Thresholds -----
def genColorMask(image):
	color_thresholds = (image[:,:,0] < white_threshold[0]) | (image[:,:,1] < white_threshold[1]) | (image[:,:,2] < white_threshold[2])
	return color_thresholds

# ------ Hough Lines -----
def hough(image, linesOverlay):
	rho = 1
	theta = np.pi / 180
	threshold = 20
	min_length = 15
	max_gap = 200

	lines = cv2.HoughLinesP(image, rho, theta, threshold, np.array([]), min_length, max_gap)

	return lines

# ------ Draw Lines -----
def findEndPoints(y1, y2, line):

	slope, intercept = line

	x1 = int((y1 - intercept)/slope)
	x2 = int((y2 - intercept)/slope)
	y1 = int(y1)
	y2 = int(y2)

	return ((x1, y1), (x2, y2))

# ------ Hough Average -----
def averageLines(lines):

	leftSideLines = []
	leftWeights = []
	rightSideLines = []
	rightWeights = []

	for line in lines:
		for x1, y1, x2, y2 in line:
			if x2 == x1:
				continue # Ignore Vertical Line
			slope = (float(y2) - float(y1))/(float(x2) - float(x1))
			yInterecpt = y1 - slope * x1
			length = np.sqrt((y2-y1)**2+(x2-x1)**2)
			if slope < 0:
				leftSideLines.append((slope, yInterecpt))
				leftWeights.append(length)
				#print "Line found at (", x1, ",", x2, "),(", y1, ",", y2, ") Slope: ", format(slope, '.2f'), " Y-Intercept: ", format(yInterecpt, '.2f'), " Length: ", length, " LEFT"
			else:
				#print "Line found at (", x1, ",", x2, "),(", y1, ",", y2, ") Slope: ", format(slope, '.2f'), " Y-Intercept: ", format(yInterecpt, '.2f'), " Length: ", length, " RIGHT"
				rightSideLines.append((slope, yInterecpt))
				rightWeights.append(length)

	# Calculate weight based on length of the line
	if len(leftSideLines) == 0:
		leftPoints = None
	else:
		leftPoints  = np.dot(leftWeights,  leftSideLines) / np.sum(leftWeights)
	
	if len(rightSideLines) == 0:
		rightPoints = None
	else:
		rightPoints = np.dot(rightWeights, rightSideLines) / np.sum(rightWeights)

	return leftPoints, rightPoints

# ------ Draw Lines -----
def drawLines(image, lines):
	leftSide, rightSide = averageLines(lines)

	y1 = image.shape[0]
	y2 = y1 * 0.6

	laneOverlay = np.copy(image)

	#print "Left Side Lane Line at: ", leftSide, " Right Side Lane Line at: ", rightSide

	# Find Left Side
	if leftSide is not None:
		leftLine = findEndPoints(y1, y2, leftSide
		cv2.line(laneOverlay, leftLine[0], leftLine[1], [255, 0, 0], 20)
	# Find Right Side
	if rightSide is not None:	
		rightLine = findEndPoints(y1, y2, rightSide)
		cv2.line(laneOverlay, rightLine[0], rightLine[1], [255, 0, 0], 20)

	return laneOverlay

# ------ Image Processing -----
def processImage(rawImage):
	global frameCounter
	#print
	#print "Processing Frame ", frameCounter, ":"
	# Get the Edgy Image
	EdgyImage = canny(blur(convertGreyscale(rawImage), 9), 60, 60)
	# Crop out all the crap
	areaToProcess = regionOfInterest(EdgyImage)

	# Show Region of Interest for Each Frame for Debugging
	#plt.imshow(regionOfInterest(rawImage))
	#plt.show()

	# Find the lines
	processedImage = drawLines(rawImage, hough(areaToProcess, rawImage))
	frameCounter += 1
	return processedImage

# ------ Helper Functions -----
def convertGreyscale(image):
	return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

def canny(image, low, high):
	return cv2.Canny(image, low, high)

def blur(image, size):
	return cv2.GaussianBlur(image, (size, size), 0)

# ------ Main -----

# I know how to do the challenge, but I ran out of time

videoOutput = 'test_videos_output/solidWhiteRight.mp4'
clip1 = VideoFileClip("test_videos/solidWhiteRight.mp4")
white_clip = clip1.fl_image(processImage)
white_clip.write_videofile(videoOutput, audio=False)
