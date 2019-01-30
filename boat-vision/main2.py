"""
	@modified: Mon Jan 28, 2019
	@author: Ingrid Navarro 
	@brief: RobotBoat Vision System 
	@file: main2.py
	@version: 1.0
"""

# TODO: implement in ROS
# TODO: add object ID's
# TODO: implement max_disappeard tracking to perform object detection 

from detection.detector import Detector
import argparse
import cv2
import numpy as np

from imutils.video import VideoStream
from imutils.video import FPS
import imutils
import time


ap = argparse.ArgumentParser()
ap.add_argument('-c', '--config', required=True, help = 'Path to yolo config file')
ap.add_argument('-w', '--weights', required=True, help = 'Path to yolo pre-trained weights')
ap.add_argument('-cl', '--classes', required=True, help = 'Path to text file containing class names')
ap.add_argument('-v', '--video', required=True, help = 'Path to the video' )
ap.add_argument("-t", "--tracker", type=str, default="kcf", help="OpenCV object tracker type")
args = ap.parse_args()

# Initialize detector 
detector = Detector(args.config, args.weights, args.classes)
(H, W) = (None, None)

# Initialize Video Stream
print("[INFO] starting video stream...")
if args.video == "0": # Open WebCam
	video = cv2.VideoCapture(0)
else: # Open specified video file
	video = cv2.VideoCapture(args.video)

# Load model method
print("[INFO] loading network model...")
net = detector.load_model()

# Initialize tracker
print("[INFO] initializing multitracker...")
TRACKERS = {
	"kcf": cv2.TrackerKCF_create,
	"boosting": cv2.TrackerBoosting_create,
	"mil": cv2.TrackerMIL_create,
	"tld": cv2.TrackerTLD_create,
	"medianflow": cv2.TrackerMedianFlow_create,
}

# initialize the bounding box coordinates of the object we are going
# to track
fps = None
count = 0 
# loop over frames from video stream
detect = True
while video.isOpened():
	count += 1

	if count == 24:
		detect = True
		count = 0

	# grab current frame
	ret, frame = video.read()
	frame = imutils.resize(frame, width=800)
	(H, W) = frame.shape[:2]

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
	BOXES = []

	if detect:
		# Get dections 
		trackers = cv2.MultiTracker_create()
		boxes, confidences, indices, cls_ids = detector.get_detections(net, frame)

		# Draw predictions
		for i in indices:
			i = i[0]
			box = boxes[i]
			x, y, w, h = box
			x, y, w, h = round(x), round(y), round(w), round(h)
			BOXES.append((x, y, w, h))

			#detector.draw_prediction(frame, cls_ids[i], confidences[i], x, y, x+w, y+h)

		for b in range(len(BOXES)):
			trackers.add(TRACKERS[args.tracker](), frame, BOXES[b])

		fps = FPS().start()
		detect = False

	(success, bbx) = trackers.update(frame)
	time.sleep(0.02)

	for box in bbx:
		(x, y, w, h) = [int(v) for v in box]
		cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

	# update the fps counter
	fps.update()
	fps.stop()

	# initialize the set of information we'll be displaying on the frame
	info = [
		("Tracker", args.tracker), 
		("FPS", "{:.2F}".format(fps.fps())),
	]

	# loop over the info tuples and draw them on the frame
	for (i, (k, v)) in enumerate(info):
		text = "{}: {}".format(k, v)
		cv2.putText(frame, text, (10, detector.get_h() - ((i * 20) + 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

	# show the output frame
	cv2.imshow("Frame", frame)

cv2.destroyAllWindows()

