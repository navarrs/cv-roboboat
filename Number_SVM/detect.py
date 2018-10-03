# Import the modules
import cv2
from sklearn.externals import joblib
from skimage.feature import hog
import numpy as np

# Load the classifier
clf = joblib.load("digits_cls.pkl")

# Read the input image 
im = cv2.imread("2.png")


# Convert to grayscale and apply Gaussian filtering
im_gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
im_gray = cv2.GaussianBlur(im_gray, (5, 5), 0)

# Threshold the image
ret, im_th = cv2.threshold(im_gray, 90, 255, cv2.THRESH_BINARY_INV)


im_th= cv2.copyMakeBorder(im_th,10,10,10,10,cv2.BORDER_CONSTANT)


cv2.imshow("Image",im_th)


roi = cv2.resize(im_th, (28, 28), interpolation=cv2.INTER_AREA)
cv2.imshow("roi",roi)
roi_hog_fd = hog(roi, orientations=9, pixels_per_cell=(14, 14), cells_per_block=(1, 1), visualise=False)

nbr = clf.predict(np.array([roi_hog_fd], 'float64'))

print("Result number", nbr)
cv2.waitKey(0)