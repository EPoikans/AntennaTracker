import cv2
import numpy as np
import initialize_data

#Loading training images and labels and saving the data
images = []
labels = np.array([1,1,2,2,2,2,2,0,0,0,0,0,0,3,3,3,3,3,3,9,9,9,9,9,9,9,6,6,6,6,6,6,6,8,8,8,8,8,8,6,6,6,6,6,6,1,1,1,1,1,1,1,9,9,9,9,9,9,6,6,6,6,6,6,5,5,5,5,5,5,5,8,8,8,8,8,8,7,7,7,7,7,7,7,7,7,7,7,7,7,5,5,5,5,5,5,5,7,7,7,7,7,7,1,1,1,1,1,1,3,3,3,3,3,4,4,4,4,4,2,2,9,9,9,9,9,9,9,9,4,4,4,4,4,6,6,6,6,6,6,6,6,6,6,6,1,1,1,1,9,9,9,9,9,9,9,6,6,6,5,5,5,5,5,1,0,0,0,0,0,0]) #Picture label array
trainedlabels = labels[:,np.newaxis]
picnum = 171 #Number of training images
img_data = []

#Settings for testing run speeds on different img sizes and visualize progress in a window
resize_newsize = (20,20) #Default pictures are 30x30 px
resize = True
show_img_examples = False

if initialize_data.debug:
	print(resize_newsize + ' - resize newsize' + resize + ' - resize' + show_img_examples + ' - show img examples')
	print(picnum + ' - Dataset size')

for i in range(picnum):
	tempimg = cv2.imread('./images/%d.jpg' %i) #Reads image with the name of int(i).jpg
	tempimg = cv2.cvtColor(tempimg, cv2.COLOR_BGR2GRAY) #Forces image to not be RGB
	if(resize): #False by default
		tempimg = cv2.resize(tempimg, resize_newsize, interpolation=cv2.INTER_AREA)
	if(show_img_examples): #False by default, if true, creates a window showing each picture for 500ms and printing the filename in the console
		cv2.waitKey(500)
		print("%d.png" %i)
		cv2.imshow('img_example', tempimg)
	images.append(tempimg.copy())
	img_data.append(tempimg.copy())
img_data = np.array(img_data)

if(resize):
	train_array = img_data.reshape(-1,(resize_newsize[0]*resize_newsize[1])).astype(np.float32) #Image pixel data gets flattened by a dimension if resize is true by the new resize
else:
	train_array = img_data.reshape(-1,900).astype(np.float32) #Default image data flattening 30x30 -> 900

if(show_img_examples): #Kills all windows after image showing
	cv2.waitKey(5000)
	cv2.destroyAllWindows()

np.savez('knn_data.npz',train_array=train_array, trainedlabels=trainedlabels) #Saves the training data
print('Trained')