#!/usr/bin/env python

"""
    @modified: Tue Feb 12, 2019
    @author: IngridNavarroA and fitocuan
    @file: colors.py
    @version: 1.0
    @brief:    
        Implements helper functions to obtain object color and change 
        the brightness of an image. 
    @requirements: 
        Tested on python2.7 and python3.6. 
        OpenCV version 3
        Tested on ROS Kinetic. 
"""
from cv_bridge import CvBridge, CvBridgeError
from detect.srv import ObjectColor

import rospy
import cv2
import numpy as np

bridge = CvBridge()

class Color():
    """ Class to add color to info strings. """ 
    BLUE  = '\033[94m'
    GREEN = '\033[92m'
    RED   = '\033[91m'
    DONE  = '\033[0m'

def get_object_color(image, x, y, w, h):
    """ Given a bounding box, obtains the color of the object within it. """
    global bridge
    
    image = bridge.cv2_to_imgmsg(image, encoding = "bgr8")

    try:
        rospy.wait_for_service("/get_object_color", timeout=5.0)
        service = rospy.ServiceProxy("/get_object_color", ObjectColor)
        color = service(image,x,y,w,h)
        #rospy.loginfo(color)
        return str(color.color)
    
    except (rospy.ServicesException, rospy.ROSException) as e:
        rospy.logerr("Service call failed: {}".format(e))
        exit()

def change_brightness(image, brightness_coeff=1.5): 
    """ Increases brightness of an image. """
    
    # Convert to HSL image
    image = cv2.cvtColor(image,cv2.COLOR_RGB2HLS) 
    image = np.array(image, dtype = np.float64) 

    # Scale pixel values up or down for channel 1(Lightness)
    image[:, :, 1] = image[:, :, 1] * brightness_coeff 
    image[:,:,1][image[:,:,1] > 255]  = 255 
    image = np.array(image, dtype = np.uint8)

    # Conversion to RGB
    return cv2.cvtColor(image, cv2.COLOR_HLS2RGB)