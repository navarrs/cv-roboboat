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
		""" Intitializes Object() instance, with attrinutes: ID, 
			class, color, bounding box, tracker, lives and centroid. 
		"""
		self.id       = None
		self.clss     = clss
		self.color    = color
		self.bbox     = bbox
		self.tracker  = tracker
		self.lives    = lives
		x, y, w, h    = bbox
		self.centroid = (int(w / 2.0 + x), int(h / 2.0 + y))

		# TODO: define a possible distance

	def update_tracker(self, frame):
		""" Updates tracker using KCF filters. """
		return self.tracker.update(frame)

	def update_object_bbox(self, bbox):
		""" Updates bounding box obtained from new frame. """
		self.bbox = bbox
		x, y, w, h = bbox
		self.centroid = (int(w / 2.0 + x), int(h / 2.0 + y))

	def get_object_bbox(self):
		""" Returns current bounding box. """
		return self.bbox

	def reset_lives(self, lives=8):
		""" When the object is found, its lives are reset. """
		self.lives = lives

	def reduce_lives(self):
		""" Reduces life when object is not found in current frame. """
		self.lives -= 1

	def get_lives(self):
		""" Returns current lives. """
		return self.lives

	def get_color(self):
		""" Returns object color. """
		return self.color

	def get_class(self):
		""" Returns object class. """
		return self.clss

	def get_id(self):
		""" Returns object id. """
		return self.id

	def print_object(self):
		""" Prints object atributes. """
		obj = {
			'id'      : self.id, 
			'class'   : self.clss,
			'color'   : self.color,
			'bbox'    : self.bbox,
			'centroid': self.centroid,
			'lives'   : self.lives
		}
		print(obj)

class Tracks():
	def __init__(self, max_lives=8):
		""" Initializes Tracks instance that keeps track of current
		    detected object. Atributes are, next id, objects, max lives.
		"""
		self.next_obj_ID = 0
		self.objects = OrderedDict()
		self.first = True
		self.max_lives = max_lives

	def get_object(self, obj):
		""" Returns object from specified ID. """
		return self.objects[obj]

	def set_object(self, obj, o):
		""" Sets object from specified ID, to specified object. """
		self.objects[o] = obj

	def get_next_id(self):
		""" Returns next ID. """
		return self.next_obj_ID

	def add_object(self, obj):
		""" Adds new object. """
		obj.id = self.next_obj_ID
		self.objects[self.next_obj_ID] = obj
		self.next_obj_ID += 1

	def delete_object(self, obj_id):
		""" Deletes object instance. """
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
			self.first = False
		# Compare centroids
		else:
			# If there are no objects, add them
			if len(self.objects) == 0:
				for o in detected_objs:
					self.add_object(o)
			# If there are objects, compare their centroids
			else:
				# TODO: vectorize this. 
				input_centroids = []
				for obj in detected_objs:
					input_centroids.append(obj.centroid)

				current_centroids, object_ids = [], []
				for obj in self.objects:
					object_ids.append(obj)
					current_centroids.append(self.get_object(obj).centroid)

				# Compute distances between each pair of object centroids
				D = dist.cdist(np.array(current_centroids), np.array(input_centroids))

				# Find smallest value in each row, and sort them
				rows = D.min(axis=1).argsort()

				# Find smallest value in each col, and sort them
				cols = D.argmin(axis=1)[rows]

				# To determine if we need to update, add or delete 
				# and object, keep track of which of the rows and column
				# indexes we have already examined
				used_rows, used_cols = set(), set()

				# Loop over the combination of the (row, col) index tuples
				for (row, col) in zip(rows, cols):

					# Check if row, or col have already been examined
					if row in used_rows or col in used_cols:
						continue

					# Otherwise, update and reset lives
					object_id = object_ids[row]
					self.objects[object_id].centroid = input_centroids[col]
					self.objects[object_id].reset_lives(self.max_lives)

					# Indicate that row and col have been used
					used_rows.add(row)
					used_cols.add(col)

				# Compute row and col that have not been examined
				unused_rows = set(range(0, D.shape[0])).difference(used_rows)
				unused_cols = set(range(0, D.shape[1])).difference(used_cols)

				# If the number of current objects >= input objects, check 
				# the objects have disappeared. 
				if D.shape[0] >= D.shape[1]:
					# Loop over the unused row indexes
					for row in unused_rows:
						# Grab object ID, and decrement lives
						object_id = object_ids[row]

						self.objects[object_id].reduce_lives()

						# If lives == 0, delete object.
						if self.objects[object_id].get_lives() == 0:
							self.delete_object(self.objects[object_id])
				# If number of current objects < input objects, add 
				# all the new objects. 
				else:
					for col in unused_cols:
						self.add_object(detected_objs[col])