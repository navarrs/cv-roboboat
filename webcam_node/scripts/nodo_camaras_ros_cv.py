#!/usr/bin/env python 


import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys

#Variables globales
bridge = CvBridge()
img2 = Image()
img1 = Image()


#Funcion que modifica las imagenes de ambas camaras en formato cv
def modImg():

	global img1
	global img2
	
	cv2.putText(img2,'Camara 1',(10,50),cv2.FONT_HERSHEY_SIMPLEX ,1, (255,255,255), 2)
	cv2.putText(img1,'Camara 2',(10,50),cv2.FONT_HERSHEY_SIMPLEX ,1, (255,255,255), 2)
	


def image_callback2(ros_image):

	global img2
	global bridge

	try:
		print("got image2")

		#se lee la imagen recibida y se cambia a formato cv
		img2 = bridge.imgmsg_to_cv2(ros_image, "bgr8")
		
		#se modifica la imagen
		modImg()
		
		#se vuelve a cambiar a formato Image para ser publicada 
		ros_image = bridge.cv2_to_imgmsg(img2, encoding = "bgr8")
		
		msg = ros_image		
		pub2.publish(msg)
		
	except CvBridgeError as e:
		print(e)
			




def image_callback(ros_image):
	global img1
	global bridge

	try:
		print("got image1")
		
		#se lee la imagen recibida y se cambia a formato cv
		img1 = bridge.imgmsg_to_cv2(ros_image, "bgr8")

		#se modifica la imagen		
		modImg()

		#se vuelve a cambiar a formato Image para ser publicada 		
		ros_image = bridge.cv2_to_imgmsg(img1, encoding = "bgr8")
		
		msg = ros_image		
		pub.publish(msg)

	except CvBridgeError as e:
		print(e)
			

	

if __name__ == '__main__':
	
	
	rospy.init_node('image_converter', anonymous = True)
	
	#se declaran los subscribers a los nodos de las camaras
	image_sub1 = rospy.Subscriber("/usb_cam1/image_raw", Image, image_callback)
	image_sub2 = rospy.Subscriber("/usb_cam2/image_raw", Image, image_callback2)
	
	#se declaran los publishers donde se mandaran las imagenes modificadas	
	pub = rospy.Publisher("/imagen_modificada", Image, queue_size = 10)
	pub2 = rospy.Publisher("/imagen_modificada2", Image, queue_size = 10)
				
	rate = rospy.Rate(5)	
	
	rate.sleep()
		
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("shutting down")
	cv2.destroyAllWindows()
