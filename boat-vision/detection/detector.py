"""
	@modified: Wed Dec 19, 2018
	@author: Ingrid Navarro 
	@brief: Perform object detection using YOLOv3 framework. 
	@file: detection.py
	@version: 1.0
"""

import cv2
import numpy as np 
import imutils
from imutils.video import FPS, VideoStream
import time

class Detector():
	def __init__( self, cfg, weights, class_file, conf_thresh=0.5, nms_thresh=0.4 ):
		"""
			Constructor
		"""
		self.config  = cfg
		self.weights = weights
		with open(class_file, 'r') as f:
			self.classes = [line.strip() for line in f.readlines()]
		self.conf_thresh = conf_thresh
		self.nms_thresh = nms_thresh

	def load_model( self ):
		return cv2.dnn.readNet(self.config, self.weights)

	def get_blob( self, scale ):
		return cv2.dnn.blobFromImage(image, scale, (416,416), (0,0,0), True, crop=False)

