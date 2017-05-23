#!/usr/bin/env python

import cv2
from sklearn.externals import joblib
from skimage.feature import hog
import numpy as np
import time
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import threading
import csv
import datetime
import rosbag
from std_msgs.msg import Int32, String

class video(object):

    def __init__(self):
        #event that will block until the info is received
        self._event = threading.Event()
        #attribute for storing the rx'd message
        self._video = None
	self._time = None
	self._time2 = None
	self._frame = None


    def __call__(self, msg):
        #Uses __call__ so the object itself acts as the callback
        #save the data, trigger the event
        self._video = msg
	self._time = msg.header.stamp.nsecs
	self._time2 = msg.header.stamp.secs
	self._frame = msg.header.seq
	self._event.set()

    def get_video(self, timeout=None):
        """Blocks until the data is rx'd with optional timeout
        Returns the received message
        """
        self._event.wait(timeout)
        return self._video

    def get_time(self, timeout=None):
        """Blocks until the data is rx'd with optional timeout
        Returns the received message
        """
        self._event.wait(timeout)
        return self._time

    def get_time2(self, timeout=None):
        """Blocks until the data is rx'd with optional timeout
        Returns the received message
        """
        self._event.wait(timeout)
        return self._time2

    def get_frame(self, timeout=None):
        """Blocks until the data is rx'd with optional timeout
        Returns the received message
        """
        self._event.wait(timeout)
        return self._frame




# RECEBER O CROP DO FRAME A LER

rospy.init_node('video_receiver', anonymous=True)

vid_callback = video()

rospy.Subscriber('video_msg', Image, vid_callback)
#delay_pub = rospy.Publisher('delay_msg', String, queue_size=30)

# Load the classifier
clf = joblib.load("digits_cls.pkl")


#bag = rosbag.Bag('delay.bag', 'w')

def sort_contours(ctrs, method="left-to-right"):
	reverse = False
	i = 0
 
	# handle if we need to sort in reverse
	if method == "right-to-left" or method == "bottom-to-top":
		reverse = True
 
	# handle if we are sorting against the y-coordinate rather than
	# the x-coordinate of the bounding box
	if method == "top-to-bottom" or method == "bottom-to-top":
		i = 1
 
	# construct the list of bounding boxes and sort them from top to
	# bottom
	rects = [cv2.boundingRect(ctr) for ctr in ctrs]
	(ctrs, rects) = zip(*sorted(zip(ctrs, rects),
		key=lambda b:b[1][i], reverse=reverse))
	return rects

lis = []
lis_final = []
kernel = np.ones((5,5),np.uint8)

#count = 0
def video():
	#print rospy.Time.now()
	
	v = vid_callback.get_video()
	t = str(vid_callback.get_time())[0:2]
	t2 = str(vid_callback.get_time2())[8:12]
	
	frames = vid_callback.get_frame()
	timestamp = t2+t
	#print timestamp, frames
	vid_final = CvBridge().imgmsg_to_cv2(v, "bgr8")
	flip = cv2.flip(vid_final,-1)
	vid = cv2.cvtColor(flip, cv2.COLOR_BGR2GRAY)
	vid_gau = cv2.GaussianBlur(vid, (5, 5), 0)
	crop = vid_gau[100:340, 20:620]
	#e = cv2.morphologyEx(crop, cv2.MORPH_OPEN, kernel)
	erode = cv2.erode(crop,kernel,iterations = 1)
	# Threshold the image
	ret, video = cv2.threshold(erode, 150, 255, cv2.THRESH_BINARY)
	video_rgb = np.zeros((video.shape[0]*2, video.shape[1], 3))
	video_rgb[:video.shape[0],:,0] = video
	video_rgb[:video.shape[0],:,1] = video
	video_rgb[:video.shape[0],:,2] = video

	_, ctrs, hier = cv2.findContours(video.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	rects = [cv2.boundingRect(ctr) for ctr in ctrs]
	
	sort = sort_contours(ctrs, method="left-to-right")
	#print vid_callback.get_time()
	a = len(sort)
	#print a
	if a != 4:
		lis_final.append(-1)
		#print ""
	for rect in sort:
	    if len(sort) != 4:
		pass 
		
	    else:
		
		    #print lis_final
		    # Draw the rectangles
		    cv2.rectangle(video_rgb, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 255, ), 2) 
		    # Make the rectangular region around the digit
		    leng = int(rect[3] * 1.6)
		    pt1 = int(rect[1] + rect[3] // 2 - leng // 2)
		    pt2 = int(rect[0] + rect[2] // 2 - leng // 2)
		    roi = video[pt1:pt1+leng, pt2:pt2+leng]
		    #print pt1, pt2, leng
		    # Resize the image
		    roi = cv2.resize(roi, (28, 28), interpolation=cv2.INTER_AREA)
		    roi = cv2.dilate(roi, (3, 3))
		    # Calculate the HOG features
		    roi_hog_fd = hog(roi, orientations=9, pixels_per_cell=(14, 14), cells_per_block=(1, 1), visualise=False)
		    nbr = clf.predict(np.array([roi_hog_fd], 'float64'))
		    # cv2.putText(video_rgb, str(int(nbr[0])), (rect[0]-20, rect[1]),cv2.FONT_HERSHEY_DUPLEX, 3, (0, 0, 255), 3)
		    cv2.putText(video_rgb, str(int(nbr[0])), (rect[0]	, rect[1] + video.shape[0]),cv2.FONT_HERSHEY_DUPLEX, 3, (0, 0, 255), 3)


		    text_hour = str(nbr[0])
		    
		    
		    for val in text_hour:
		    	lis.append(val)
			if len(lis)>3:
				#print lis
				valor_b = str(lis[0])+str(lis[1])+str(lis[2])+str(lis[3])
				#valor_b = str(lis[0])+str(lis[1])+str(lis[2])+str(lis[3])+str(lis[4])+str(lis[5])
				#print valor_b
				#bag.write('video_msg', valor_bonito)
				#delay_pub.publish(valor_bonito)
				#print "enchi, vou limpar a lista"
				lis[:] = []
				#print valor_b
				#now = rospy.get_time()
				#print rospy.loginfo("I heard %s"%valor_bonito)
				delay = int(timestamp)-int(valor_b)
				with open('/home/lourenco/jumpstart/sharpeye15_ws/src/seagull_autopilot/scripts/data.csv', 'wb') as f:
					lis_final.append(delay)
					print delay
					writer = csv.writer(f)
					writer.writerow(lis_final)
				#print delay
				#print lis_final	
	#msg = str(rospy.get_time())
	#print msg
	    #print rect[0], rect[1], (rect[0] + rect[2], rect[1] + rect[3])
	    #print text_hour
	    #list.append(nbr[0])
	#cv2.imwrite("/home/lourenco/jumpstart/sharpeye15_ws/src/seagull_autopilot/scripts/flip_img/%s.jpg"%str(rospy.Time.now()),flip)
	#cv2.imwrite("/home/lourenco/jumpstart/sharpeye15_ws/src/seagull_autopilot/scripts/vid_img/%s.jpg"%str(rospy.Time.now()),vid)
	#cv2.imwrite("/home/lourenco/jumpstart/sharpeye15_ws/src/seagull_autopilot/scripts/vidgau_img/%s.jpg"%str(rospy.Time.now()),vid_gau)
	#cv2.imwrite("/home/lourenco/jumpstart/sharpeye15_ws/src/seagull_autopilot/scripts/crop_img/%s.jpg"%str(rospy.Time.now()),crop)
	#cv2.imwrite("/home/lourenco/jumpstart/sharpeye15_ws/src/seagull_autopilot/scripts/erode_img/%s.jpg"%str(rospy.Time.now()),erode)
	#cv2.imwrite("/home/lourenco/jumpstart/sharpeye15_ws/src/seagull_autopilot/scripts/video_img/%s.jpg"%str(rospy.Time.now()),video)
	#cv2.imwrite("/home/lourenco/jumpstart/sharpeye15_ws/src/seagull_autopilot/scripts/videorec_img/%s.jpg"%str(rospy.Time.now()),video_rgb)
	#cv2.imwrite("/home/lourenco/jumpstart/sharpeye15_ws/src/seagull_autopilot/scripts/frames_final/%s.jpg"%str(rospy.Time.now()),)
	
	cv2.imshow('final',video_rgb)
	#cv2.imshow('crop',video)
	#cv2.imshow('d',video)
	cv2.waitKey(1000)

	

while True:
	video()
