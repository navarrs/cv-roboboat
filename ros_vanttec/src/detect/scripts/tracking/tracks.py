"""
	@modified: Wed Mar 6th, 2019
	@author: IngridNavarroA
	@file: tracks.py
	@version: 1.0
	@brief: 
		This code implements:
		  - A class Object() to create objects whose characteristics are: 
		  	color, class, bounding box, tracker object, and remaining lives.
		  - A class Tracks() that keeps track of the current objects being 
		  	detected. 
	@requirements:
		Tested on python2.7 and python3.6. 
        OpenCV version 3.4+ (because it uses the "dnn" module).
        Cuda version 8.0
        Tested on ROS Kinetic. 
        Tested on Ubuntu 16.04 LTS
"""
from scipy.spatial import distance as dist
import numpy as np
from collections import OrderedDict
import cv2

class Object():
	def __init__(self, clss, color, bbox, tracker, lives=15):
		self.id      = None
		self.clss    = clss
		self.color   = color
		self.bbox    = bbox
		self.tracker = tracker
		self.lives   = lives

	def update_obj_bbox(self, bbox):
		self.bbox = bbox

	def print_object(self):
		obj = {
			'id'   : self.id, 
			'class': self.clss,
			'color': self.color,
			'bbox' : self.bbox,
			'lives': self.lives
		}
		print obj

class Tracks():
	def __init__(self):
		self.next_obj_ID = 0
		self.objects = OrderedDict()
		self.first = True

	def get_object(self, obj):
		return self.objects[obj]

	def get_next_id(self):
		return self.next_obj_ID

	def add_object(self, obj):
		obj.id = self.next_obj_ID
		self.objects[self.next_obj_ID] = obj
		self.next_obj_ID += 1

	def delete_object(self, obj_id):
		if len(self.objects) == 1:
			self.objects = OrderedDict()
			self.next_obj_ID = 0 
		else:
			del self.objects[obj_id]

	def update(self, detected_objs):
		# Tracks will be added for the first time
		if self.first:
			self.objects = OrderedDict()
			self.next_obj_ID = 0 
			for o in detected_objs:
				self.add_object(o)
			#self.first = False
		# Need to do data association to update, 
		# add or delete objects 
		# elif detect and self.next_obj_ID > 0:
		#	pass
		
		# elif not self.first:
		# 	self.objects = OrderedDict()
		# 	self.next_obj_ID = 0 
		# 	self.first = True

		# Only update based on KCF tracker
		# elif not detect:
		#	pass

