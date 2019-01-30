"""
	@modified: Wed Dec 19, 2018
	@author: Ingrid Navarro 
	@brief: RobotBoat Vision System 
	@file: main.py
	@version: 1.0
"""
#from colors import *
from detection.detector import Detector
from tracker.centroidtracker import CentroidTracker
import argparse
import cv2
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('-c', '--config', required=True, help = 'path to yolo config file')
ap.add_argument('-w', '--weights', required=True, help = 'path to yolo pre-trained weights')
ap.add_argument('-cl', '--classes', required=True, help = 'path to text file containing class names')
ap.add_argument('-v', '--video', required=True, help = 'path to the video' )
args = ap.parse_args()

# Initialize detector and tracker
detector = Detector(args.config, args.weights, args.classes)
tracker = CentroidTracker()
(H, W) = (None, None)

# Load model method
print("[INFO] loading network model...")
net = detector.load_model()

# Initialize Video Stream
print("[INFO] starting video stream...")
if args.video == "0": # Open WebCam
	video = cv2.VideoCapture(0)
else: # Open specified video file
	video = cv2.VideoCapture(args.video)

# Loop over frames
count = 0

while video.isOpened():
	
	ret, frame = video.read()
	roi = np.zeros((frame.shape[0], frame.shape[1], 3), np.uint8)
	
	if detector.get_w() is None or detector.get_h() is None:
		detector.set_h(frame.shape[0])
		detector.set_w(frame.shape[1])

	if not ret:
		print("[INFO] done processing...")
		cv2.waitKey(2000)
		break
	elif cv2.waitKey(1) & 0xFF == ord ('q'):
		print("[INFO] quitting program...")
		break

	boxes = []
	confidences = []
	indices = []
	sea_markers = []
	bouys = []
	if count % 24 == 0:
		# Get dections 
		boxes, confidences, indices, cls_ids = detector.get_detections(net, frame)

		# Draw predictions
		for i in indices:
			i = i[0]
			box = boxes[i]
			x, y, w, h = box
			x, y, w, h = round(x), round(y), round(w), round(h)

			if cls_ids[i] == 1:
				sea_markers.append([x, y, w, h])
				roi[y:y+h, x:x+w] = frame[y:y+h, x:x+w] 
			else:
				bouys.append([x, y, w, h])

			detector.draw_prediction(frame, cls_ids[i], confidences[i], x, y, x+w, y+h)

		# Track objects 
		objects = tracker.update( boxes )
		for (obj_id, centroid) in objects.items():
			text = "ID: {}".format(obj_id)
			cv2.putText(frame, text, (centroid[0] - 10, centroid[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
		
		cv2.imshow("Frame", frame)
		cv2.imshow("ROIs", roi)

		while not cv2.waitKey(1) & 0xFF == ord('n'):
			cv2.imshow("HSV ROIs", hsv_roi)

	elif count == 240:
		count = 0
	count += 1

cv2.destroyAllWindows()


