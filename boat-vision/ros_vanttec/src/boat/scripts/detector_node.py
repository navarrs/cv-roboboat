#!/usr/bin/env python

"""
    @modified: Wed Jan 30, 2019
    @author: Ingrid Navarro 
    @file: detector_node.py
    @version: 1.0
    @brief:    
        This code implements a ROS node to perform object 
        detection and classification using Tiny YOLOv3. 
        Node receives frames from another node and publishes 
        the coordinates of the detected objects. 
    @requirements: 
        Tested on python2.7 and python3.6. 
        OpenCV version 3.4+ (because it uses the "dnn" module).
        Cuda version 8.0
        Tested on ROS Kinetic. 
        Tested on Ubuntu 16.04 LTS
"""
from detection.detector import Detector 
from std_msgs.msg import String

import argparse
import numpy as np 
import time
import rospy
import cv2

ap = argparse.ArgumentParser()
ap.add_argument('--config', required=True, help = 'Path to yolo config file')
ap.add_argument('--weights', required=True, help = 'Path to yolo pre-trained weights')
ap.add_argument('--classes', required=True, help = 'Path to text file containing class names')
ap.add_argument('--video', required=True, help = 'Path to the video' )
args = ap.parse_args()

cv2.dnn.readNet(args.config, args.weights)

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "__hello world %s " % rospy.get_time()
        hello_str += " %s " % cv2.__version__ 
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
