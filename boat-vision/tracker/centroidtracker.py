"""
	@modified: Fri Dec 21, 2018
	@author: Ingrid Navarro 
	@brief: Simple centroid-based object tracker using object detector YOLO
	@file: centroidtracker.py
	@version: 1.0
"""
from scipy.spatial import distance as dist
from collections import OrderedDict
import numpy as np

class 	CentroidTracker():
	def __init__(self, max_disappeared=50):
		"""
			- int: next object ID
			- ordered dict: to keep track of mapping an object's I
					D with its centroid			
			- ordered dict: to keep the number of consecutive frames it
			 		has been marked as "disappeared"
			- int: max number of frames an object can be marked as 
				   disappeared
		"""
		self.next_object_ID = 0
		self.objects = OrderedDict()
		self.disappeared = OrderedDict()
		self.max_disappeared = max_disappeared

	def add_object(self, centroid):
		""" To add an object, use the next available ID and store the 
			centroid. Set the disappeared mark to 0. Increment next ID.
		"""
		self.objects[self.next_object_ID] = centroid
		self.disappeared[self.next_object_ID] = 0
		self.next_object_ID += 1

	def delete_object(self, object_ID):
		""" Delete the object ID from the dictionaries. """
		del self.objects[object_ID]
		del self.disappeared[object_ID]

	def update(self, bounding_boxes):
		""" Object tracking """
		# If there are no detections, i.e. no rectangles, mark every
		# existing object as disappeared. 
		if len(bounding_boxes) == 0:
			for object_ID in self.disappeared.keys():
				self.disappeared[object_ID] += 1

				# If limit of marks as disappeared is reached, delete object
				if self.disappeared[object_ID] > self.max_disappeared:
					self.delete_object(object_ID)

			return self.objects

		# Array of input centroids for the bounding boxes
		input_centroids = np.zeros((len(bounding_boxes), 2), dtype="int")

		# x, y, w, h = bboxes[:0], bboxes[:1], bboxes[:2], bboxes[:3] 
		# centroid_x = int( w / 2.0 + x)
		# centroid_y = int( h / 2.0 + y)
		# input_centroids = (centroid_x, centroid_y)

		for (i, (x, y, w, h)) in enumerate(bounding_boxes):
			# Compute centroid
			centroid_x = int( w / 2.0 + x)
			centroid_y = int( h / 2.0 + y)
			input_centroids[i] = (centroid_x, centroid_y)

		# No new centroids? add objects, else match input
		# centroids to existing centroids
		if len(self.objects) == 0:
			for i in range(len(input_centroids)):
				self.add_object(input_centroids[i])
		else:
			object_IDs = list(self.objects.keys())
			object_centroids = list(self.objects.values())

			# Distance between each pair of object centroids 
			D = dist.cdist(np.array(object_centroids), input_centroids)

			# Find the smallest value in each row and sort the row indexes
			rows = D.min(axis=1).argsort()

			# Find the smallest value in each column and sort them based on the rows
			cols = D.argmin(axis=1)[rows]

			# To determine if we need to update, add, or delete an object,
			# keep track of which of the rows and column indexes we have 
			# already examined
			used_rows = set()
			used_cols = set()

			# loop over the combination of the (row, column) index tuples
			for (row, col) in zip(rows, cols):
				
				# if we have already examined either the row or
				# column value before, ignore it
				if row in used_rows or col in used_cols:
					continue

				# otherwise, grab the object ID for the current row,
				# set its new centroid, and reset the disappeared
				# counter
				object_ID = object_IDs[row]
				self.objects[object_ID] = input_centroids[col]
				self.disappeared[object_ID] = 0

				# indicate that we have examined each of the row and
				# column indexes, respectively
				used_rows.add(row)
				used_cols.add(col)

			# compute both the row and column index we have NOT yet
			# examined
			unused_rows = set(range(0, D.shape[0])).difference(used_rows)
			unused_cols = set(range(0, D.shape[1])).difference(used_cols)

			# in the event that the number of object centroids is
			# equal or greater than the number of input centroids
			# we need to check and see if some of these objects have
			# potentially disappeared
			if D.shape[0] >= D.shape[1]:
				# loop over the unused row indexes
				for row in unused_rows:
					# grab the object ID for the corresponding row
					# index and increment the disappeared counter
					object_ID = object_IDs[row]
					self.disappeared[object_ID] += 1

					# check to see if the number of consecutive
					# frames the object has been marked "disappeared"
					# for warrants deregistering the object
					if self.disappeared[object_ID] > self.max_disappeared:
						self.delete_object(object_ID)

			# otherwise, if the number of input centroids is greater
			# than the number of existing object centroids we need to
			# register each new input centroid as a trackable object
			else:
				for col in unused_cols:
					self.add_object(input_centroids[col])

		# return the set of trackable objects
		return self.objects