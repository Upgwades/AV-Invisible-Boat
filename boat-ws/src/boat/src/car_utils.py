import numpy as np
import rospy as rp
from sensor_msgs.msg import Image
import cv2

from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
class imgproc:
    display = True
    image = np.zeros((1,1,3))
    height,width = image.shape[:2]
    ROI = np.array([[(0,0),(width,0),(width,height),(height,0)]],dtype=np.int32)
    ROI_BOUNDS = (0,0,width, height)
    web_view = None
    rate = 0
    def set(self,image):
        self.image = image
        self.height,self.width = self.image.shape[:2]
    def set_display(self,boolean):
        self.display = boolean
        if boolean:
            self.web_view = rp.Publisher('show_lines',Image,queue_size=1)
            self.rate = rp.Rate(21)#Hz
    def set_ROI(self, ROI):
        self.ROI = np.array([ROI],dtype=np.int32)
        self.ROI_BOUNDS = cv2.boundingRect(self.ROI)
    def get_ROI_mask(self,image,crop=True):
        mask = np.zeros_like(image)
        if len(image.shape) > 2:
            channel_count = image.shape[2]
            ignore_color = (255,)*channel_count
        else:
            ignore_color = 255
        cv2.fillPoly(mask,self.ROI,ignore_color)
	if crop:
            return cv2.bitwise_and(image,mask)[self.ROI_BOUNDS[1]:self.ROI_BOUNDS[1]+self.ROI_BOUNDS[3],self.ROI_BOUNDS[0]:self.ROI_BOUNDS[0]+self.ROI_BOUNDS[2]]
	else:
	    return cv2.bitwise_and(image,mask)
    def get_color_mask(self,image,color_range):
        lower = np.array(color_range[0],dtype="uint8")
        upper = np.array(color_range[1],dtype="uint8")
        return cv2.bitwise_and(image,image,mask=cv2.inRange(cv2.cvtColor(image,cv2.COLOR_BGR2HSV), lower, upper))
    def get_blur(self, image, blur_size):
	    return cv2.GaussianBlur(image, (blur_size, blur_size), 0)
    def get_grayscale(self,image):
        return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    def clean_image(self,blur_size,color_range):
        return self.get_grayscale(self.get_color_mask(self.get_ROI_mask(self.image),color_range))
    def slice_image(self,image,slices):
        h,w = image.shape[:2]
        sliceHeight = (h/slices)
        for i in range(slices):
            cv2.line(image, (0, sliceHeight*i), (w,sliceHeight*i), (0,0,0), 1, 0)
        return image,sliceHeight
    def get_impulse(self,slices,blur_size,color_range):
        clean = self.clean_image(blur_size,color_range)
        if self.display:
            overlay = self.image.copy()
        L_Centers = {}
        R_Centers = {}
        L_Weights = {}
        R_Weights = {}
        for x in range(slices):
            L_Centers[x] = []
            R_Centers[x] = []
            L_Weights[x] = []
            R_Weights[x] = []
        sliced,slice_size = self.slice_image(clean,slices)
        threshold = cv2.threshold(sliced, 0, 255, 0)[1]
        contours = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[1]
        for c in contours:
            imageMoment = cv2.moments(c)
            if cv2.contourArea(c) >= 500:
                x,y,rect_w,rect_h = cv2.boundingRect(c)
                if self.display:
                    cv2.drawContours(overlay[self.ROI_BOUNDS[1]:self.ROI_BOUNDS[1]+self.ROI_BOUNDS[3],self.ROI_BOUNDS[0]:self.ROI_BOUNDS[0]+self.ROI_BOUNDS[2]], c, -1, (0,255,0), 2)
                centerX = int(imageMoment['m10'] / imageMoment['m00'])+self.ROI_BOUNDS[0]
                centerY = int(imageMoment['m01'] / imageMoment['m00'])+self.ROI_BOUNDS[1]
                if self.display:
                    cv2.circle(overlay, (centerX, centerY), 5, (0, 0, 255), -1)
                    self.web_view.publish(bridge.cv2_to_imgmsg(self.get_ROI_mask(overlay,crop=False), "bgr8"))
                if centerX < self.width/2:
                    L_Centers[(self.height-centerY)//slice_size] += [centerX]
                    L_Weights[(self.height-centerY)//slice_size] += [(cv2.contourArea(c))/(rect_w*rect_h)]
                else:
                    R_Centers[(self.height-centerY)//slice_size] += [centerX]
                    R_Weights[(self.height-centerY)//slice_size] += [(cv2.contourArea(c))/(rect_w*rect_h)]
                if (len(L_Centers[0]) == len(R_Centers[0])) and (centerY < centerY):
                    pass    # Eventual code for full stop when end of line detected, conditional imperfect
        common = 0
        center = 0
        weight = 0
        impulses = []
        for x in range(slices):
            Len_L = len(L_Centers[x])
            Len_R = len(R_Centers[x])
            if Len_L > 0 and Len_R > 0:
                common = min([Len_L,Len_R])
                center = ((sum(sorted(L_Centers[x],key=int,reverse=True)[:common])/common)+(sum(sorted(R_Centers[x],key=int,reverse=False)[:common])/common))-self.width
                weight = ((sum(sorted(L_Weights[x],key=int,reverse=True)[:common])/common)+(sum(sorted(R_Weights[x],key=int,reverse=False)[:common])/common))/2
                impulses += [center*weight]
        if self.display:
            try:
                self.web_view.publish(bridge.cv2_to_imgmsg(self.get_ROI_mask(overlay,crop=False), "bgr8"))
            except CvBridgeError as e:
                print(e)
        if len(impulses) == 0:
            return None
        else:
            return sum(impulses)
    def displayFeed(self,feed):
	    cv2.imshow('Feed Display', feed)
	    cv2.waitKey(1)

    def displayImage(self,image):
        cv2.imshow('Image Display',image)
        cv2.waitKey(0)
