#!/usr/bin/env python


import rospy
import numpy as np
import cv2
import time
import threading
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import std_msgs.msg


cap = cv2.VideoCapture(0)


rospy.init_node('video_sender', anonymous=True)


def video_out():

		
	video_pub = rospy.Publisher('video_msg', Image, queue_size=1)
	

	rate = rospy.Rate(30)
	bridge = CvBridge()
	
	
	while not rospy.is_shutdown():

		#print msg
		ret,frame = cap.read()
		#msg.data = frame
		#msg.header.stamp = rospy.Time.now()
		#print msg
		
		#cv2.imshow('frame',frame)
		
		#video_pub.publish(msg)
		rate.sleep()
		k = cv2.waitKey(30)
		if k==27:
			break
		
		
		
		img_data = bridge.cv2_to_imgmsg(frame, "bgr8")
		img_data.header.stamp = rospy.Time.now()
		video_pub.publish(img_data)
		#h = std_msgs.msg.Header()
		#h.stamp = rospy.Time.now()
		#msg.data = img_data	
		#print msg
					
		
		
        	
video_out()
