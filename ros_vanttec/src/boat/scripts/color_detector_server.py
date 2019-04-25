#!/usr/bin/env python


import rospy
import cv2
import numpy as np
from custom_msgs.srv import ColorDeImagen
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def callback_color(img):
	global bridge
	
	image = img.imagen
	x = img.x 
	y = img.y
	h = img.h
	w = img.w
	image = bridge.imgmsg_to_cv2(image, "bgr8")
	print(x)
	print(y)
	print(h)
	print(w)
	image = image[y:y+h,x:x+w]
	
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	
	# split each channel from hsv tensor
	#(h, s, v) = cv2.split(image)

	#if np.median(s) < 50:
	#	predicted_color = "white"
	#else:
		# use histogram obtain frequency of hue regions of interest
		# in this case its useful to divide it in 6 regions of equal length
		# then map color code to frequency using the histogram
	#	hist = cv2.calcHist([hsv], [0], None, [6], [0, 180])
	#	colors = ["red", "green", "green", "blue", "blue", "red"]

		# uncomment in case 6 bins case is detecting false positives use 12 and adjust color mapping
		# hist = cv2.calcHist([hsv], [0], None, [12], [0, 180])
		# colors = ["red", "-", "-", "green", "green", "-", "-", "blue", "blue", "-", "-", "red"]

	#	hist_list = [hist[i][0] for i in range(len(hist))]
	#	hist_dic = dict(zip(hist_list, colors))

		# uncomment to see the frequencies of each color region
		# print(hist_dic)

		# predicted_color contains the region of maximum frequency
	#	predicted_color = hist_dic[max(hist_dic)]

	#ret = str(predicted_color)	
	
	hist = cv2.calcHist([hsv], [0], None, [6], [0, 180])
	
	colors = ["red", "green", "green", "blue", "blue", "red"]
	hist_list = [hist[i][0] for i in range(len(hist))]
	hist_dic = dict(zip(hist_list, colors))
	
	ret = str(hist_dic[max(hist_dic)])
	
	return ret
	


if __name__ == '__main__':

	rospy.init_node('color_detector_server')
	rospy.loginfo("Node created!")

	service = rospy.Service("/get_color", ColorDeImagen, callback_color)

	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		rate.sleep()
