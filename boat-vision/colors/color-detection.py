# import necessary packages
import argparse
import cv2

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required = True, help = "Path to the image")
args = vars(ap.parse_args())

# load the image and show it
image = cv2.imread(args["image"])
image = cv2.resize(image, (340, 220))
cv2.imshow("image", image)

# convert the image to hsv
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# split each channel from hsv tensor
(h, s, v) = cv2.split(image)

if np.median(s) < 50:
    predicted_color = "white"
else:
    # use histogram obtain frequency of hue regions of interest
    # in this case its useful to divide it in 6 regions of equal length
    # then map color code to frequency using the histogram
    hist = cv2.calcHist([hsv], [0], None, [6], [0, 180])
    colors = ["red", "green", "green", "blue", "blue", "red"]

    # uncomment in case 6 bins case is detecting false positives use 12 and adjust color mapping
    # hist = cv2.calcHist([hsv], [0], None, [12], [0, 180])
    # colors = ["red", "-", "-", "green", "green", "-", "-", "blue", "blue", "-", "-", "red"]

    hist_list = [hist[i][0] for i in range(len(hist))]
    hist_dic = dict(zip(hist_list, colors))

    # uncomment to see the frequencies of each color region
    # print(hist_dic)

    # predicted_color contains the region of maximum frequency
    predicted_color = hist_dic[max(hist_dic)]
    print("Predicted color: " + predicted_color)

cv2.waitKey(0)
