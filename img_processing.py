import cv2
import os
import time
import pymavlink
from multiprocessing import Process
import numpy as np

with np.load('knn_data.npz') as data: #Loads training dataset from images
    train_array = data['train_array']
    trainedlabels = data['trainedlabels']
knn = cv2.ml.KNearest_create() #Creates simple KNN 
knn.train(train_array, cv2.ml.ROW_SAMPLE, trainedlabels) #Trains it on 39 samples of 10 digits with the same font

# Must be the same as training data!
resize_newsize = (20,20) #Default pictures are 30x30 px
resize = False

def knn_accuracy_test(): #Checks that the NN can recognise the dataset with 100% accuracy
    ret,result,neighbours,dist = knn.findNearest(train_array,k=1)
    correct = np.count_nonzero(result==trainedlabels)
    accuracy = correct*100.0/result.size
    print( accuracy )
    if(accuracy!= 100):
        print('Accurracy not 100%')
    return accuracy

if(knn_accuracy_test() != 100): #Kills the program if accuracy isnt 100%
    exit()

#
#	Video feed input data collection
#
initial = True
usesamplevid = True #Used in testing a video recording
show_full_frame = True #If true, shows a window with the OSD and treshold processed background and bounding boxes drawn
if(usesamplevid):
	videofeed = cv2.VideoCapture('drone_feed_test.mp4')
else:
    videofeed = cv2.VideoCapture(1) #Value specifies which video input device is used camera or usb hdmi capture card
while((videofeed.isOpened()!= True) and (initial == True)): #If videofeed is not opened during the initial launch waits 1s until it loads
	cv2.waitKey(1000)
	print('Awaiting video')
initial = False
videofps = videofeed.get(cv2.CAP_PROP_FPS)
capture_frequency = 1 # analyzed frames per second, 1 recomened
x_res = videofeed.get(cv2.CAP_PROP_FRAME_WIDTH)
y_res = videofeed.get(cv2.CAP_PROP_FRAME_HEIGHT)

# Creates arrays of locations for the readable fields
# Native 1920x1080, video test 960x544
# left top corner (x,y), right top corner(x,y), right bottom corner(x,y), left bottom corner(x,y)

lat_boundbox = np.array([[int(730/960*x_res),int(326/544*y_res)],[int(920/960*x_res),int(326/544*y_res)],[int(920/960*x_res),int(354/544*y_res)],[int(730/960*x_res),int(354/544*y_res)]]) 
lat_width = abs(lat_boundbox[0,0]-lat_boundbox[1,0])
lat_height = abs(lat_boundbox[0,1]-lat_boundbox[2,1]) #Data for getting the latitude

lon_boundbox = np.array([[int(730/960*x_res),int(355/544*y_res)],[int(920/960*x_res),int(355/544*y_res)],[int(920/960*x_res),int(380/544*y_res)],[int(730/960*x_res),int(380/544*y_res)]]) 
lon_width = abs(lon_boundbox[0,0]-lon_boundbox[1,0])
lon_height = abs(lon_boundbox[0,1]-lon_boundbox[2,1]) #Data for getting the longitude

alt_boundbox = np.array([[int(20/960*x_res),int(410/544*y_res)],[int(110/960*x_res),int(410/544*y_res)],[int(110/960*x_res),int(440/544*y_res)],[int(20/960*x_res),int(440/544*y_res)]]) 
alt_width = abs(alt_boundbox[0,0]-alt_boundbox[1,0])
alt_height = abs(alt_boundbox[0,1]-alt_boundbox[2,1]) #Data for getting the altitude

boundingbox_arr = np.array([lat_boundbox, lon_boundbox, alt_boundbox])

def img_to_number(img_frame, resize, resize_newsize):
	number_contour = cv2.findContours(img_frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	number_contour = number_contour[0]
	good_contour = []
	good_bounding_boxes = []
	img_array = []
	img_array_resized = []
	if(resize):
		resize_size = resize_newsize
	else:
		resize_size = (30,30)
	for i in number_contour:
		(x,y,w,h) = cv2.boundingRect(i)
		if w>=5 and (h>= 16):
			good_contour.append(i)
			good_bounding_boxes.append(cv2.boundingRect(i))

	good_bounding_boxes_arr = np.array(good_bounding_boxes)
	i = 0
	for i in range(good_bounding_boxes_arr.shape[0]):
		y1 = good_bounding_boxes_arr[i,1].item()
		y2 = int(good_bounding_boxes_arr[i,1])+int(good_bounding_boxes_arr[i,3])
		x1 = int(good_bounding_boxes_arr[i,0])
		x2 = int(good_bounding_boxes_arr[i,0])+int(good_bounding_boxes_arr[i,2])
		tempimg = img_frame[y1:y2, x1:x2]
		img_array.append(tempimg.copy())
		temp_resized_img = cv2.resize(tempimg, resize_size, interpolation=cv2.INTER_AREA)
		img_array_resized.append(temp_resized_img.copy())
	img_array_resized = np.array(img_array_resized)
	img_array_resized = img_array_resized.reshape(-1,(int(resize_size[0])*int(resize_size[1]))).astype(np.float32)
	ret,result,neighbours,dist = knn.findNearest(img_array_resized,k=1)
	endstring = str(int(result[(len(result)-1)]))
	i=0
	for i in range (len(result)-1):
		endstring = endstring + str(int(result[(len(result)-2-i)]))
		
	if(len(result)>=7):
		endnum = int(endstring)/(10000000)
	else:
		endnum = int(endstring) 
	return endnum

def videoloop(show_full_frame, videofeed,boundingbox_arr, videofps, capture_frequency,lat_boundbox, lat_width, lat_height, lon_boundbox, lon_width,lon_height,alt_boundbox,alt_width,alt_height):
	while(videofeed.isOpened()):
		i = 0
		for i in range (int(videofps/capture_frequency)):
			framestatus, frame = videofeed.read()
	
		if not framestatus:
			print('No more frames, stopping')
			break
		grayframe = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		framestatus2, tresholdframe = cv2.threshold(grayframe,220,255,cv2.THRESH_TOZERO) #Removes background
		if not framestatus2:
			print('No more frames, stopping')
			break
		lat_area = tresholdframe[lat_boundbox[0,1]:lat_boundbox[0,1]+lat_height, lat_boundbox[0,0]:lat_boundbox[0,0]+lat_width]
		lon_area = tresholdframe[lon_boundbox[0,1]:lon_boundbox[0,1]+lon_height, lon_boundbox[0,0]:lon_boundbox[0,0]+lon_width]
		alt_area = tresholdframe[alt_boundbox[0,1]:alt_boundbox[0,1]+alt_height, alt_boundbox[0,0]:alt_boundbox[0,0]+alt_width]
		lat = img_to_number(lat_area, resize, resize_newsize)
		lon = img_to_number(lon_area, resize, resize_newsize)
		alt = img_to_number(alt_area, resize, resize_newsize)
		print(lat,lon,alt)
		if(show_full_frame): #Testing the full view
			cv2.drawContours(tresholdframe, boundingbox_arr, -1 ,(255,255,255), 1)
			cv2.imshow('frame', tresholdframe)
		if cv2.waitKey(1000) == ord('q'):
			break
	videofeed.release()
	cv2.destroyAllWindows()

videoloop(show_full_frame, videofeed,boundingbox_arr, videofps, capture_frequency,lat_boundbox, lat_width, lat_height, lon_boundbox, lon_width,lon_height,alt_boundbox,alt_width,alt_height)