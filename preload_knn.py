import platform
import cv2
import numpy as np

debug = True

def knn_accuracy_test(knn,train_array,trainedlabels): #Checks that the NN can recognise the dataset with 100% accuracy
	ret,result,neighbours,dist = knn.findNearest(train_array,k=1)
	correct = np.count_nonzero(result==trainedlabels)
	accuracy = correct*100.0/result.size
	if debug:
		print(f'{accuracy}% accuracy')
	if accuracy != 100:
		print('Accuracy not 100%')
	return accuracy

if platform.system().lower() == 'linux':
	try:
		import RPi.GPIO as GPIO
		comp_setup = 'Raspi'
		knnData = '/home/pi/Desktop/AntennaTracker/knn_data.npz'
	except ImportError:
		comp_setup = 'PC'
		knnData = 'knn_data.npz'
else:
	comp_setup = 'PC'
	knnData = 'knn_data.npz'

if debug:
	print(str(knnData) + ' knn data filepath used')

with np.load(knnData) as data: #Loads training dataset from images
		train_array = data['train_array']
		trainedlabels = data['trainedlabels']
		global knn
		knn = cv2.ml.KNearest_create() #Creates simple KNN 
		knn.train(train_array, cv2.ml.ROW_SAMPLE, trainedlabels)
	
if(knn_accuracy_test(knn, train_array, trainedlabels) <= 92): #Kills the program if accuracy isnt above 92
	exit()