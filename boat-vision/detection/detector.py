"""
	@modified: Wed Dec 19, 2018
	@author: Ingrid Navarro 
	@brief: Perform object detection using YOLO framework. 
	@file: detection.py
	@version: 1.0
"""

import cv2
import numpy as np 
import imutils
from imutils.video import FPS, VideoStream
import time

def get_output_layers(net):
	"""
		Gets layers that make detections.
	"""
	layer_names = net.getLayerNames()
	output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
	return output_layers

class Detector():
	def __init__( self, cfg, weights, class_file, conf_thresh=0.5, nms_thresh=0.4 ):
		"""
			Constructor.
		"""
		self.config  = cfg
		self.weights = weights
		with open(class_file, 'r') as f:
			self.classes = [line.strip() for line in f.readlines()]
		self.conf_thresh = conf_thresh
		self.nms_thresh = nms_thresh
		self.W = None
		self.H = None
		self.COLORS = np.random.uniform(0, 255, size=(len(self.classes), 3))

	def get_w( self ):
		""" Gets current frame width. """
		return self.W

	def set_w( self, w ):
		""" Sets frame width. """
		self.W = w

	def get_h( self ):
		""" Gets current frame height. """
		return self.H

	def set_h( self, h ):
		""" Sets frame height. """
		self.H = h

	def load_model( self ):
		""" Loads DNN model using the configuration and weights file. """
		return cv2.dnn.readNet(self.config, self.weights)

	def get_blob( self, scale, image ):
		""" Gets image blob. """
		return cv2.dnn.blobFromImage(image, scale, (416,416), (0,0,0), True, crop=False)

	def get_detections( self, net, image ):
		""" Computes detections and returns a list of bounding boxes, 
			confidences, indices and class ids. """

		# Get image blob
		scale = 0.00392 # ?
		blob = self.get_blob( scale, image )
		net.setInput(blob)

		# Detections
		class_ids = []
		confidences = []
		boxes = []
		det = []
		outs = net.forward( get_output_layers(net) )
		for out in outs:
			for detection in out:
				scores = detection[5:]
				class_id = np.argmax( scores )
				confidence = scores[class_id]
				if confidence > 0.5:
					center_x = int( detection[0] * self.W )
					center_y = int( detection[1] * self.H )
					w = int( detection[2] * self.W )
					h = int( detection[3] * self.H )
					x = center_x - w / 2
					y = center_y - h / 2
					class_ids.append( class_id )
					confidences.append( float( confidence ) )
					boxes.append( [x, y, w, h] )

		indices = cv2.dnn.NMSBoxes(boxes, confidences, self.conf_thresh, self.nms_thresh)
		
		return boxes, confidences, indices, class_ids

	def draw_prediction( self, img, class_id, confidence, x, y, x_plus_w, y_plus_h ):
		""" Draws bounding boxes to image. """
		label = str( self.classes[class_id] )
		color = self.COLORS[class_id] 
		cv2.rectangle(img, (x,y), (x_plus_w,y_plus_h), color, 2)
		cv2.putText(img, label, (x-10,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)




		
