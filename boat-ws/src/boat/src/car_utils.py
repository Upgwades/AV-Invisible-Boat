import numpy as np
import rospy as rp
from math import pi,sqrt
from sensor_msgs.msg import Image
import cv2

from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()

def clamp(n,a,b):
	return max(min(b,n),a)

def rad(degree):
	return (degree*pi)/180

def deg(radian):
    return (radian*180)/pi

class orbiter:
	def __init__(self):
		self.direction = ""
		self.origin = 0
		self.radius = 0.0
		self.data = {}
		self.ref_ind = 0

	def set_dir(self,dir):
		self.direction = dir
		if dir == "left":
			self.ref_ind = 180
		elif dir == "right":
			self.ref_ind = 900

	def set_origin(self,o):
		self.origin = o

	def set_radius(self,r):
		self.radius = r

	def set_data(self,data):
		self.data = data
		self.clean_data()

	def update_all(self,dir,o,r,data):
		self.direction = dir
		self.radius = r
		self.data = data

	def clean_data(self):
		previous_length = -1
		while len(self.data) != previous_length:
			previous_length = len(self.data)
			for key in range(0,len(self.data)):
				if key in self.data && key+1 in self.data:
					self.data[key] = max(self.data[key],self.data[key+1])
					del self.data[key+1]


	def orbit_archspiral(self):
		steering_correction = 0.0
		return steering_correction

	def orbit_circular(self):
		steering_correction = 0.0
		closes = min(self.data.keys(), key=lambda x:abs(x-self.ref_ind))
		reff_diff = abs(closes - self.ref_ind)
		rad_diff = abs(data[closes] - self.radius)
		return steering_correction

class pid_steering:
    def __init__(self):
        self.old_errors = [0]
        self.kp = 0.065
        self.ki = 0.0
        self.kd = 0.0

    def set_pid(self,kp=0,ki=0,kd=0):
        self.kp,self.ki,self.kd = kp,ki,kd

    def add_error(self,error):
        self.old_errors += [error]
        if len(self.old_errors) > 100:
            self.old_errors.pop(0)

    def control(self,error):
        proportional = error
        integral = sum(self.old_errors)
        derivative = self.old_errors[-2]+error

        output = ((self.kp * proportional) + (self.ki*integral) + (self.kd*derivative))
        if(output<-1):
            output = -1
        elif(output>1):
            output = 1
        return -output

    def steer(self,impulse):
        try:
            if impulse != 0:
                steering_angle = self.control(impulse)
                speed = ((-1*sqrt(abs(0.3*steering_angle)))+1)-0.15
                acceleration = ((0.273861*steering_angle)/(abs(steering_angle)**(3/2)))
                jerk = ((0.136931*(steering_angle**2))/(abs(steering_angle)**(7/2)))
                steering_angle_velocity = 0
	        #steering_angle_velocity = ((-1*sqrt(abs(0.3*steering_angle)))+1)-0.15
                #print "Steering: %.2f, Speed: %.2f, Acceleration: %.2f, Jerk: %.2f, Angle: %.2f"%(steering_angle,speed,acceleration,jerk,steering_angle_velocity)
            else:
                steering_angle = 0
                speed = 1
                acceleration = 0
                jerk = 0
                steering_angle_velocity = 0
            return speed,steering_angle,acceleration,jerk,steering_angle_velocity
        except:
            print("error")

class timer:
    def __init__(self):
        self.current_time = cv2.getTickCount()
        self.scale = 0
        self.name = "Timer"
        self.times = {}
        self.max_label_size = 30

    def set_max_label_size(self,max_label_size):
        self.max_label_size = max_label_size

    def set_scale(self,scale):
        self.scale = scale

    def set_name(self,name):
        self.name = name

    def update(self):   # Good practice to call before first get_time in function, any time it's been "awhile" since relevant code has executed
        if rp.get_param('timer'):
            self.current_time = cv2.getTickCount()

    def get_label_avg(self,label):
        if rp.get_param('timer'):
            if label in self.times:
                print "[%s] %s average: %s%.2f x 10^%d ticks"%(self.name,label,' '*(self.max_label_size-len(label)),(sum(self.times[label])/len(self.times[label]))*(10**self.scale),(-1*self.scale))
            else:
                print "No such label."

    def get_all_avg(self):
        if rp.get_param('timer'):
            print "[%s] Timer averages:"%self.name
            labels = sorted(self.times.keys())
            for label in labels:
                print "\t%s: %s%.2f x 10^%d ticks"%(label,' '*(self.max_label_size-len(label)),(sum(self.times[label])/len(self.times[label]))*(10**self.scale),(-1*self.scale))

    def get_time(self,label):   # Gets the time (in ticks) since this same function was last run in this instance (including the overhead of the function itself)
        if rp.get_param('timer'):
            if len(label) <= self.max_label_size:
                if label not in self.times:
                    self.times[label] = []
                current_time,self.current_time = cv2.getTickCount() - self.current_time,cv2.getTickCount()
                if len(self.times[label])<100:
                    self.times[label] += [current_time]
                else:
                    self.times[label] = self.times[label][1:]+[current_time]
                print "[%s] %s: %s%.2f x 10^%d ticks"%(self.name,label,' '*(self.max_label_size-len(label)),current_time*(10**self.scale),(-1*self.scale))
            else:
                print "Label too long! (%d characters max)"%self.max_label_size

class laserproc:
    def __init__(self):
        self.display = False
        self.image = np.zeros((1,1,3))
        self.height,self.width = self.image.shape[:2]
        self.web_view = None
        self.rate = 0
    def set_display(self,boolean):
        self.display(boolean)
        if boolean:
            self.web_view = rp.Publisher('show_lines',Image,queue_size=1)
            self.rate(21)
    def show_lasers(self,data):
        size = int(round(max(data)))
        size += 50
        size *= 2
        self.image = np.zeros((size,size,3))
        self.height,self.width = self.image.shape[:2]
        cv2.circle(self.image,(size,size), 5, (0,0,255), -1)



class imgproc:
    def __init__(self):
        self.display = False
        self.image = np.zeros((1,1,3))
        self.height,self.width = self.image.shape[:2]
        self.ROI = np.array([[(0,0),(self.width,0),(self.width,self.height),(self.height,0)]],dtype=np.int32)
        self.ROI_BOUNDS = (0,0,self.width, self.height)
        self.web_view = None
        self.rate = 0
        self.clock = timer()
        self.clock.set_scale(-6)
        self.clock.set_name("imgproc")

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

    def clean_image(self,color_range):
        return self.get_grayscale(self.get_color_mask(self.get_ROI_mask(self.image),color_range))

    def slice_image(self,image,slices):
        h,w = image.shape[:2]
        sliceHeight = (h/slices)
        for i in range(slices):
            cv2.line(image, (0, sliceHeight*i), (w,sliceHeight*i), (0,0,0), 1, 0)
        return image,sliceHeight

    def get_impulse(self,slices,color_range):

        self.clock.update()

        clean = self.clean_image(color_range)

        self.clock.get_time("Clean image")

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

        self.clock.get_time("Slice image")

        threshold = cv2.threshold(sliced, 0, 255, 0)[1]
        contours = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[1]

        self.clock.get_time("Get contours")

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

        self.clock.get_time("Find centers & weights")

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

        self.clock.get_time("Calculate impulse")

        if self.display:
            try:
                self.web_view.publish(bridge.cv2_to_imgmsg(self.get_ROI_mask(overlay,crop=False), "bgr8"))

                self.clock.get_time("Publish visual")

                self.clock.get_all_avg()

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
