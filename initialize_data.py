import cv2
import numpy as np
from pymavlink import mavutil

global gpshome
gpshome = []

def initialize_data(useMavlink, useOSD, homegps_type, usesamplevid, accelerometer):
	accelerometer_bool = bool(accelerometer) 
	if(useOSD):
		with np.load('knn_data.npz') as data: #Loads training dataset from images
			train_array = data['train_array']
			trainedlabels = data['trainedlabels']
		global knn
		knn = cv2.ml.KNearest_create() #Creates simple KNN 
		knn.train(train_array, cv2.ml.ROW_SAMPLE, trainedlabels) #Trains it on 39 samples of 10 digits with the same font

		# Must be the same as training data!
		global resize_newsize
		resize_newsize = (20,20) #Default pictures are 30x30 px
		global resize
		resize = False


		if(knn_accuracy_test(knn, train_array, trainedlabels) != 100): #Kills the program if accuracy isnt 100%
			exit()

		#
		#	Video feed input data collection
		#
		initial = True
		usesamplevid = True #Used in testing a video recording
		global videofeed
		if(usesamplevid):
			videofeed = cv2.VideoCapture('./TestingFiles/drone_feed_test.mp4')
			show_full_frame = False #If true, shows a window with the OSD and treshold processed background and bounding boxes drawn
			videofps = videofeed.get(cv2.CAP_PROP_FPS)
			capture_frequency = 1 # analyzed frames per second, 1 recomened
		else:
			videofeed = cv2.VideoCapture(1) #Value specifies which video input device is used camera or usb hdmi capture card
		while((videofeed.isOpened()!= True) and (initial == True)): #If videofeed is not opened during the initial launch waits 1s until it loads
			cv2.waitKey(1000)
			print('Awaiting video')
		initial = False
		
		x_res = videofeed.get(cv2.CAP_PROP_FRAME_WIDTH)
		y_res = videofeed.get(cv2.CAP_PROP_FRAME_HEIGHT)

		# Creates arrays of locations for the readable fields
		# Native 1920x1080, video test 960x544
		# left top corner (x,y), right top corner(x,y), right bottom corner(x,y), left bottom corner(x,y)
		global lat_boundbox
		global lat_width
		global lat_height
		lat_boundbox = np.array([
				[int(730/960*x_res),int(355/544*y_res)],
				[int(920/960*x_res),int(355/544*y_res)],
				[int(920/960*x_res),int(380/544*y_res)],
				[int(730/960*x_res),int(380/544*y_res)]
			])
		lat_width, lat_height =  get_wh(lat_boundbox)

		global lon_boundbox
		global lon_width
		global lon_height
		lon_boundbox = np.array([
				[int(730/960*x_res),int(326/544*y_res)],
				[int(920/960*x_res),int(326/544*y_res)],
				[int(920/960*x_res),int(354/544*y_res)],
				[int(730/960*x_res),int(354/544*y_res)]
			])
		lon_width, lon_height =  get_wh(lon_boundbox) 

		global alt_boundbox
		global alt_width
		global alt_height
		alt_boundbox = np.array([
				[int(410/960*x_res),int(53/544*y_res)],
				[int(470/960*x_res),int(53/544*y_res)],
				[int(470/960*x_res),int(82/544*y_res)],
				[int(410/960*x_res),int(82/544*y_res)]
			])
		alt_width, alt_height =  get_wh(alt_boundbox) 

		global heading_boundbox
		global heading_width
		global heading_height
		heading_boundbox = np.array([
				[int(20/960*x_res),int(410/544*y_res)],
				[int(110/960*x_res),int(410/544*y_res)],
				[int(110/960*x_res),int(440/544*y_res)],
				[int(20/960*x_res),int(440/544*y_res)]
			])
		heading_width, heading_height =  get_wh(heading_boundbox) 

		global boundingbox_arr
		boundingbox_arr = np.array([lat_boundbox, lon_boundbox, alt_boundbox, heading_boundbox])

	if(useMavlink):
		connect_adress = '/dev/ttyUSB0'
		testfile = './TestingFiles/2023-09-22 12-26-58.tlog'
		usetestfile = True
		if(usetestfile):
			usedaddress = testfile
		else:
			usedaddress = connect_adress
		the_connection = mavutil.mavlink_connection(usedaddress)
		the_connection.wait_heartbeat()

		if(usetestfile == False):
			the_connection.mav.command_long_send(
				the_connection.target_system, the_connection.target_component,
				mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
				33, # The MAVLink message ID for GLOBAL_POSITION_INT
				1e6 / 4, #Frequency per second
				0, 0, 0, 0, # Unused parameters
				0,
			)

			the_connection.mav.command_long_send(
				the_connection.target_system, the_connection.target_component,
				mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
				74, # The MAVLink message ID for VFR_HUD
				1e6 / 4, #Frequency per second
				0, 0, 0, 0, # Unused parameters
				0,
			)

def knn_accuracy_test(knn,train_array,trainedlabels): #Checks that the NN can recognise the dataset with 100% accuracy
    ret,result,neighbours,dist = knn.findNearest(train_array,k=1)
    correct = np.count_nonzero(result==trainedlabels)
    accuracy = correct*100.0/result.size
    print( accuracy )
    if(accuracy!= 100):
        print('Accurracy not 100%')
    return accuracy

def get_wh(boundbox_arr):
	width = abs(boundbox_arr[0,0]-boundbox_arr[1,0])
	height = abs(boundbox_arr[0,1]-boundbox_arr[2,1])
	return width,height