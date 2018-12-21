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


ap = argparse.ArgumentParser()
ap.add_argument('-c', '--config', required=True, help = 'path to yolo config file')
ap.add_argument('-w', '--weights', required=True, help = 'path to yolo pre-trained weights')
ap.add_argument('-cl', '--classes', required=True, help = 'path to text file containing class names')
ap.add_argument( '-v', '--video', required=True, help = 'path to the video' )

args = ap.parse_args()


# Initialize detector
detector = Detector( args.config, args.weights, args.classes )

(H, W) = (None, None)

# Load model method
print ( "[INFO] loading network model..." )
net = detector.load_model()

# Initialize Video Stream
print ( "[INFO] starting video stream..." )
if args.video == "0": # Open WebCam
	video = cv2.VideoCapture( 0 )
else: # Open specified video file
	video = cv2.VideoCapture( args.video )

# Loop over frames
while video.isOpened():
	ret, frame = video.read()

	if not ret:
		print ( "[INFO] done processing..." )
		cv2.waitKey ( 2000 )
		break
	elif cv2.waitKey ( 1 ) & 0xFF == ord ( 'q' ):
		print ( "[INFO] quitting program...")
		break

	# Compute image blob
	scale = 0.00392 # ?
	blob = detector.get_blob( scale )
	net.setInput(blob)

	if W is None or H is None:
    	H = frame.shape[0]
    	W = frame.shape[1]

	cv2.imshow ( "Frame", frame )

cv2.destroyAllWindows ()

