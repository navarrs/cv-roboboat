#!/usr/bin/env python


import rospy
import cv2
from custom_msgs.srv import DistanceCal
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import numpy as np
import statistics

bridge = CvBridge()

def callback_dist(img):

	global bridge
	
	image = img.imagen
	x = img.x 
	y = img.y
	h = img.h
	w = img.w
	image = bridge.imgmsg_to_cv2(image)
	
	z_vect = []
	
	for i in range(y+(h/4),y+(h*3/4)):
		for j in range(x+(w/4),x+(w*3/4)):
			z = image[i, j]
			if not np.isnan(z) and not np.isinf(z):
				z_vect.append(z)			
	try:
	
		z = statistics.median(z_vect)	
		
	except Exception:
		z = -1
		pass	
	
	return z
	

	


if __name__ == '__main__':

	rospy.init_node('distance_server')
	rospy.loginfo("Node created!")

	service = rospy.Service("/get_distance", DistanceCal, callback_dist)
	
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		rate.sleep()
