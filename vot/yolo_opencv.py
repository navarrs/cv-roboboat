#############################################
# Object detection - YOLO - OpenCV
# Author : Arun Ponnusamy   (July 16, 2018)
# Website : http://www.arunponnusamy.com
############################################

from pyimagesearch.centroidtracker import CentroidTracker
import cv2
import argparse
import numpy as np
from imutils.video import FPS, VideoStream
import imutils
import time

def get_output_layers(net):
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    return output_layers

def draw_prediction(img, class_id, confidence, x, y, x_plus_w, y_plus_h):
    label = str(classes[class_id])
    color = COLORS[class_id]
    cv2.rectangle(img, (x,y), (x_plus_w,y_plus_h), color, 2)
    cv2.putText(img, label, (x-10,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

ap = argparse.ArgumentParser()
ap.add_argument('-c', '--config', required=True, help = 'path to yolo config file')
ap.add_argument('-w', '--weights', required=True, help = 'path to yolo pre-trained weights')
ap.add_argument('-cl', '--classes', required=True, help = 'path to text file containing class names')
args = ap.parse_args()

# Initialize centroid tracker
ct = CentroidTracker()
(H, W) = (None, None)

# Load model 
print("[INFO] loading model...")
net = cv2.dnn.readNet(args.weights, args.config)
classes = None
with open(args.classes, 'r') as f:
	classes = [line.strip() for line in f.readlines()]

# Initialize stream
print("[INFO] starting video stream...")
stream = cv2.VideoCapture(0)
time.sleep(2.0)

# Loop over frames
while stream.isOpened():
    
    ret, frame = stream.read()
    #frame = imutils.resize(frame, width=400)

    if W is None or H is None:
    	H = frame.shape[0]
    	W = frame.shape[1]

    image = frame 
    
    #image = cv2.imread(args.image)
    scale = 0.00392
    COLORS = np.random.uniform(0, 255, size=(len(classes), 3))
    
    blob = cv2.dnn.blobFromImage(image, scale, (416,416), (0,0,0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(get_output_layers(net))

    class_ids = []
    confidences = []
    boxes = []
    conf_threshold = 0.5
    nms_threshold = 0.4

    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:
                center_x = int(detection[0] * W)
                center_y = int(detection[1] * H)
                w = int(detection[2] * W)
                h = int(detection[3] * H)
                x = center_x - w / 2
                y = center_y - h / 2
                class_ids.append(class_id)
                confidences.append(float(confidence))
                boxes.append([x, y, w, h])

    objects = ct.update(boxes)

    for (objectID, centroid) in objects.items():
    	text = "ID {}".format(objectID)
    	cv2.putText(image, text, (centroid[0] - 10, centroid[1] - 10), 
    		cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    	cv2.circle(image, (centroid[0], centroid[1]), 4, (0, 255, 0), -1)

    indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)
    for i in indices:
        i = i[0]
        box = boxes[i]
        x, y, w, h = box
        draw_prediction(image, class_ids[i], confidences[i], round(x), round(y), round(x+w), round(y+h))
        

    cv2.imshow("object detection", image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
stream.stop()
