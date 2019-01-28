"""
	@modified: Wed Dec 19, 2018
	@author: Ingrid Navarro 
	@brief: RobotBoat Vision System 
	@file: main.py
	@version: 1.0
"""
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

# Load model method
print("[INFO] loading network model...")
net = detector.load_model()

# Initialize Video Stream
print("[INFO] starting video stream...")
if args.video == "0": # Open WebCam
	video = cv2.VideoCapture(0)
else: # Open specified video file
	video = cv2.VideoCapture(args.video)

# Initialize tracker
# Extract the OpenCV version info
(major, minor) = cv2.__version__.split(".")[:2]
 
# if we are using OpenCV 3.2 OR BEFORE, we can use a special factory
# function to create our object tracker
if int(major) == 3 and int(minor) < 3:
	tracker = cv2.Tracker_create(args.tracker.upper())
# otherwise, for OpenCV 3.3 OR NEWER, we need to explicity call the
# approrpiate object tracker constructor:
else:
	# initialize a dictionary that maps strings to their corresponding
	# OpenCV object tracker implementations
	OPENCV_OBJECT_TRACKERS = {
		"kcf": cv2.TrackerKCF_create,
		"boosting": cv2.TrackerBoosting_create,
		"mil": cv2.TrackerMIL_create,
		"tld": cv2.TrackerTLD_create,
		"medianflow": cv2.TrackerMedianFlow_create,
	}
	# grab the appropriate object tracker using our dictionary of
	# OpenCV object tracker objects
	#tracker = OPENCV_OBJECT_TRACKERS[args.tracker]()
 
# initialize the bounding box coordinates of the object we are going
# to track
initBB = None
fps = None
count = 0
trackers = cv2.MultiTracker_create()

# loop over frames from video stream 
while video.isOpened():
	
	# grab current frame
	ret, frame = video.read()
	roi = np.zeros((frame.shape[0], frame.shape[1], 3), np.uint8)
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
	sea_markers = []
	bouys = []
	detect = True
	BOXES = []
	if detect:
		# Get dections 
		boxes, confidences, indices, cls_ids = detector.get_detections(net, frame)

		# Draw predictions
		for i in indices:
			i = i[0]
			box = boxes[i]
			x, y, w, h = box
			x, y, w, h = round(x), round(y), round(w), round(h)
			BOXES.append((x, y, w, h))

			if cls_ids[i] == 1:
				sea_markers.append([x, y, w, h])
				roi[y:y+h, x:x+w] = frame[y:y+h, x:x+w] 
			else:
				bouys.append([x, y, w, h])

			detector.draw_prediction(frame, cls_ids[i], confidences[i], x, y, x+w, y+h)
			detect = False

	for b in range(len(BOXES)):
		tracker = OPENCV_OBJECT_TRACKERS[args.tracker]()
		trackers.add(tracker, frame, BOXES[b])

	fps = FPS().start()
	(success, bbx) = trackers.update(frame)

	for box in bbx:
		(x, y, w, h) = [int(v) for v in box]
		cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

	# update the fps counter
	fps.update()
	fps.stop()

	# initialize the set of information we'll be displaying on the frame
	info = [
		("Tracker", args.tracker), 
		("Success", "Yes" if success else "No"),
		("FPS", "{:.2F}".format(fps.fps())),
	]

	# loop over the info tuples and draw them on the frame
	for (i, (k, v)) in enumerate(info):
		text = "{}: {}".format(k, v)
		cv2.putText(frame, text, (10, detector.get_h() - ((i * 20) + 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

	# show the output frame
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF

	# if the 's' key is selected, we are going to "select" a bounding
	# box to track
	if key == ord("s"):
		# select the bounding box of the object we want to track (make
		# sure you press ENTER or SPACE after selecting the ROI)
		initBB = cv2.selectROI("Frame", frame, fromCenter=False, showCrosshair=True)
		# start OpenCV object tracker using the supplied bounding box
		# coordinates, then start the FPS throughput estimator as well
		tracker.init(frame, initBB)
		fps = FPS().start()
	elif key == ord("q"):
		break


# if not args.get("video", False):
# 	vs.stop()
# else:
# 	vs.release()

cv2.destroyAllWindows()

