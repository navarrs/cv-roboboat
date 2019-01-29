# import necessary packages
from matplotlib import  pyplot as plt
import numpy as np
import argparse
import cv2

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required = True, help = "Path to the image")
args = vars(ap.parse_args())

# load the image and show it
image = cv2.imread(args["image"])
cv2.imshow("image", image)

# convert the image to hsv and crate a histogram
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
cv2.imshow("hsv", hsv)
hist = cv2.calcHist([hsv], [0], None, [6], [0, 180])
# hist[0][0] accesses the number of pixels belonging to bin 0 (red)

# print the most frequent color range from the six bins
colors = ["red", "yellow-green", "green", "blue", "blue-magenta", "magenta-red"]
hist_list = [hist[i][0] for i in range(len(hist))]
hist_dic = dict(zip(hist_list, colors))
print(hist_dic)
print("The most abundant color in the picture was: " + str(hist_dic[max(hist_dic)]))

# plot figure
plt.figure()
plt.title("HSV Histogram")
plt.xlabel("Bins")
plt.ylabel("# of pixels")
plt.plot(hist)
plt.xlim([0, 5])
plt.show()

cv2.waitKey(0)
