#!/usr/bin/env python

import rospy
import cv2
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
	image = image[y:y+h,x:x+w]
	
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	
	hist = cv2.calcHist([hsv], [0], None, [6], [0, 180])
	
	colors = ["red", "yellow-green", "green", "blue", "blue-magenta", "magenta-red"]
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
