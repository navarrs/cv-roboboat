#!/usr/bin/env python
"""
	@modified: Tue Feb 12, 2019
	@author: fitocuan
	@file: detection.py
	@version: 1.0
	@brief: 
		This code implements a service to find the color of an 
		object within specified coordinates. 
	@requirements:
		Tested on python2.7 and python3.6. 
        OpenCV version 3
        Tested on ROS Kinetic. 
        Tested on Ubuntu 16.04 LTS
"""

from detect.srv import ObjectColor
from cv_bridge import CvBridge, CvBridgeError

import rospy
import cv2

bridge = CvBridge()

def callback_color(img):
	"""  Gets color within specified image coordinates from ObjectColor. """

	global bridge

	image = img.imagen
	x = img.x 
	y = img.y
	h = img.h
	w = img.w
	image = bridge.imgmsg_to_cv2(image, "bgr8")
	image = image[y:y+h, x:x+w]	

	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	hist = cv2.calcHist([hsv], [0], None, [6], [0, 180])
	
	# colors = ["red", "yellow-green", "green", "blue", "blue-mag", "magenta-red"]
	# TODO: detectar blanco
	colors = ["r", "g", "g", "b", "b", "r"]
	hist_list = [ hist[i][0] for i in range(len(hist)) ]
	hist_dic = dict(zip(hist_list, colors))
	ret = str(hist_dic[max(hist_dic)])
	
	return ret
	
if __name__ == '__main__':
	try:
		rospy.init_node('color_detector_srv')
		service = rospy.Service("/get_object_color", ObjectColor, callback_color)
		rate = rospy.Rate(10)

		while not rospy.is_shutdown():
			rate.sleep()

	except rospy.ROSInitException:
		print "[ERROR] Can't initialize ros service."