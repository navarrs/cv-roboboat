import glob
import numpy as np
import random
import cv2

# fetch dataset

buoy_list = []
# this is going to look like [[<filename>, [0,x,y,dx,dy]], ...]

labels_path = '/home/rreyes/Repos/VantTEC-ComputerVision/boat-vision/colors/color-detection-dataset/labels/'

for filename in glob.glob(labels_path + '*.txt'):
    with open(filename) as txt_file:
        file_list = []
        for row in txt_file:
            element_list = [float(element) for element in row.split(' ')]
            element_list[1] = int(element_list[1] * 1920) # x
            element_list[2] = int(element_list[2] * 1080) # y
            element_list[3] = int(element_list[3] * 1920) # width
            element_list[4] = int(element_list[4] * 1080) # height
            if element_list[0] == 0:
                file_list.append(element_list)
        if file_list:
            file_list.insert(0, filename)
            buoy_list.append(file_list)

buoy_list = np.array(buoy_list)

# selecting a random image to analyze
random_entry = random.choice(buoy_list)
filename = random_entry[0][len(labels_path):-4]
x = random_entry[1][1]
y = random_entry[1][2]
dx = random_entry[1][3]
dy = random_entry[1][4]

# uncomment to select an image manually
# for i in range(len(buoy_list)):
#     if buoy_list[i][0] == '/home/rreyes/Repos/VantTEC-ComputerVision/boat-vision/colors/color-detection-dataset/labels/dock_061.txt':
#         filename = 'dock_061'
#         k = 1 # for k=1,2,3,...,N where N is the number of buoys in a given picture
#         x = buoy_list[i][k][1]
#         y = buoy_list[i][k][2]
#         dx = buoy_list[i][k][3]
#         dy = buoy_list[i][k][4]

# use filename to acquire image path
imgs_path = '/home/rreyes/Repos/VantTEC-ComputerVision/boat-vision/colors/color-detection-dataset/images/'
img_path = imgs_path + filename + '.JPEG'

# generate image, crop it to buoy region
img = cv2.imread(img_path)
img_bgr = img[y-dy/2:y+dy/2, x-dx/2:x+dx/2]

# show original image
cv2.imshow('Imagen', img_bgr)

# convert to hsv and split channels
img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
(h, s, v) = cv2.split(img_hsv)

# uncomment to show h, s, v channels
# cv2.imshow('hue',h)
# cv2.imshow('sat',s)
# cv2.imshow('val',v)

# same color detection logic (see color-detection.py)
if np.median(s) < 50:
    print("The most abundant color in the picture was: white")
else:
    hist = cv2.calcHist([img_hsv], [0], None, [6], [0, 180])
    colors = ["red", "green", "green", "blue", "blue", "red"]
    hist_list = [hist[i][0] for i in range(len(hist))]
    hist_dic = dict(zip(hist_list, colors))
    print(hist_dic)
    print("The most abundant color in the picture was: " + hist_dic[max(hist_dic)])

cv2.waitKey(0)