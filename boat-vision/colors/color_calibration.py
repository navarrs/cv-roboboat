"""
	@modified: Thu Dec 27, 2018
	@author: Ingrid Navarro 
	@brief: Calibrate object color 
	@file: colorspace.py
	@version: 1.0
"""

import argparse
import cv2
import numpy as np


ap = argparse.ArgumentParser()
ap.add_argument('-v', '--video', required=True, help = 'path to the video' )
args = ap.parse_args()


# Initialize Video Stream
print("[INFO] starting video stream...")
if args.video == "0": # Open WebCam
	video = cv2.VideoCapture(0)
else: # Open specified video file
	video = cv2.VideoCapture(args.video)

# Loop over frames
while video.isOpened():
	
	ret, frame = video.read()
	
	if not ret:
		print("[INFO] done processing...")
		cv2.waitKey(2000)
		break
	elif cv2.waitKey(1) & 0xFF == ord ('q'):
		print("[INFO] quitting program...")
		break

	if count % 24 == 0: 
		cv2.imshow("Frame", frame)
	elif count == 241:
		count = 0
	count += 1

cv2.destroyAllWindows()