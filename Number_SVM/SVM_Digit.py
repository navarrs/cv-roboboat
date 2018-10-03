from sklearn.externals import joblib
from sklearn import datasets
from sklearn.datasets.mldata import fetch_mldata
from skimage.feature import hog
from sklearn.svm import LinearSVC
import numpy as np

	
#dataset = fetch_mldata("MNIST Original")

dataset = fetch_mldata('mnist-original', data_home='/home/yesus/anaconda3/envs/pyopencv/EXAMPLE/files/')


features = np.array(dataset.data, 'int16') 
labels = np.array(dataset.target, 'int')


list_hog_fd = []
for feature in features:
    fd = hog(feature.reshape((28, 28)), orientations=9, pixels_per_cell=(14, 14), cells_per_block=(1, 1), visualise=False)
    list_hog_fd.append(fd)
hog_features = np.array(list_hog_fd, 'float64')


	
clf = LinearSVC()

clf.fit(hog_features, labels)

joblib.dump(clf, "digits_cls.pkl", compress=3)
