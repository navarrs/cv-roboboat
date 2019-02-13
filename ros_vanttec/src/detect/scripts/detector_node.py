#!/usr/bin/env python

"""
    @modified: Tue Feb 12, 2019
    @author: IngridNavarroA and fitocuan
    @file: detector_node.py
    @version: 1.0
    @brief:    
        This code implements a ROS node to perform object detection and classification 
        using YOLO detection framework. 
        Node receives frames from 'camera' node and publishes the class, color and 
        coordinates of the detected objects. 
    @requirements: 
        Tested on python2.7 and python3.6. 
        OpenCV version 3.4+ (because it uses the "dnn" module).
        Cuda version 8.0
        Tested on ROS Kinetic. 
        Tested on Ubuntu 16.04 LTS
"""
from detection.detector import Detector 
from color.colors import *
from std_msgs.msg import String
from imutils.video import VideoStream
from imutils.video import FPS

import imutils
import numpy as np 
import time
import rospy
import cv2

def send_message(color, msg, det=True):
    """ Publish message to ros node. """
    msg = color + msg + Color.DONE
    rospy.loginfo(msg)
    if det:
        detector_pub.publish(msg)
    else:
        infomsg_pub.publish(msg)

def detect(config, weights, classes, video):
    """ 
        Performs object detection and publishes a data structure that contains, 
        instance coordinates, instance class and sets a counter that will be used 
        by the tracker. 
    """
    # Initialize detector 
    send_message(Color.GREEN, "[INFO] Initializing TinyYOLOv3 detector.", False)
    det = Detector(config, weights, classes)
    (H, W) = (None, None)

    # Load model 
    send_message(Color.GREEN, "[INFO] Loading network model.", False)
    net = det.load_model()

    # Initilialize Video Stream
    send_message(Color.GREEN, "[INFO] Starting video stream.", False)
    if video == "0":
        video = cv2.VideoCapture(0)
    else:
        video = cv2.VideoCapture(args.video)

    counter = 0
    dets = 0
    ID = 0
    detect = True
    fps = FPS().start()
    boxes, indices, cls_ids = [], [], []
    
    while not rospy.is_shutdown() or video.isOpened():
        # Grab next frame
        ret, frame = video.read()
        if not ret:
            send_message(Color.RED, "[DONE] Finished processing.", False)
            cv2.waitKey(2000)
            break
        elif cv2.waitKey(1) & 0xFF == ord ('q'):
            send_message(Color.RED, "[DONE] Quitting program.", False)
            break

        frame = imutils.resize(frame, width=1000)
        (H, W) = frame.shape[:2]
        if det.get_w() is None or det.get_h() is None:
            det.set_h(H)
            det.set_w(W)

        # Modify frame brightness if necessary
        frame = change_brightness(frame, 0.8) # Decrease
        # frame = change_brightness(frame, 1.5) # Increase
        
        # Perform detection
        color = None
        if detect:
            detect = False
            dets += 1
            
            # Data structure to send over topic
            objects = dict()
            objects['buoy'] = []   # Class 0
            objects['marker'] = [] # Class 1

            # Get bounding boxes, condifences, indices and class IDs
            boxes, indices, cls_ids = det.get_detections(net, frame)

            for i in range(len(cls_ids)):
                x, y, w, h = boxes[i]
                color = get_object_color(frame, x, y, h, w)
                new_obj = {'bbox': boxes[i], 'color': color, 'lives' : 40, 'id' : ID}
                ID += 1
                if cls_ids[i] == 1:
                    objects['marker'].append(new_obj)
                elif cls_ids[i] == 0:
                    objects['buoy'].append(new_obj)
                else:
                    print "Error."

            # Publish detections
            det_str = "Counter: {}, Detections: {}".format(dets, objects)
            send_message(Color.BLUE, det_str)
        else:
            counter += 1
            if counter == 24:
                detect = True
                ID = 0
                counter = 0
        
        # If there were any previous detections, draw them
        obj = 0
        for obj_cls, detections in objects.items():
            for params in detections:
                x, y, w, h = params['bbox']
                color = get_object_color(frame, x, y, h, w)
                obj_id = params['id']
                det.draw_prediction(frame, obj, color, str(obj_id), x, y, x+w, y+h)
            obj += 1

        fps.update()
        fps.stop()

        info = [
            ("FPS", "{:.2F}".format(fps.fps())),
            ("OUT", "class, color, id")
        ]
        for (i, (k, v)) in enumerate(info):
            text = "{}: {}".format(k, v)
            cv2.putText(frame, text, (10, det.get_h() - ((i * 20) + 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # Show current frame
        cv2.imshow("Frame", frame)
        # rate.sleep()

if __name__ == '__main__':
    import argparse

    # Parse arguments
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', required=True, help = 'Path to yolo config file')
    ap.add_argument('--weights', required=True, help = 'Path to yolo pre-trained weights')
    ap.add_argument('--classes', required=True, help = 'Path to text file containing class names')
    ap.add_argument('--video', required=True, help = 'Path to the video' )
    args = ap.parse_args()

    try:
        # Create publisher 
        infomsg_pub = rospy.Publisher('infomsg', String, queue_size=10)
        detector_pub = rospy.Publisher('detections', String, queue_size=10)
        rospy.init_node('detector')
        rate = rospy.Rate(10) # 10Hz
        detect(args.config, args.weights, args.classes, args.video)
    
    except rospy.ROSInterruptException:
        print "[ERROR] Interrupted. "
        exit()