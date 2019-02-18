#!/usr/bin/env python

"""
    @modified: Web Feb 13, 2019
    @author: IngridNavarroA
    @file: image_node.py
    @version: 1.0
    @brief:    
        This code implements a ROS node to read frames from video and 
        publish them.  
    @requirements: 
        Tested on python2.7 and python3.6. 
        OpenCV version 3
        Tested on ROS Kinetic. 
        Tested on Ubuntu 16.04 LTS
"""
import cv2
import roslib
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    import argparse

    # Parse arguments
    ap = argparse.ArgumentParser()
    ap.add_argument('--video', required=True, help = 'Path to the video' )
    args = ap.parse_args()

    try:
        # Create publisher 
        image_pub = rospy.Publisher('frames', String, queue_size=10)
        rospy.init_node('video')
        rate = rospy.Rate(10) # 10Hz

        while True:
            image_pub.publish('Hello')
    
    except rospy.ROSInterruptException:
        print "[ERROR] Interrupted. "
        exit()