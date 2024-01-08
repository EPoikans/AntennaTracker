import cv2
import numpy as np
import initialize_data
import ui_window
import concurrent.futures
from PIL import Image, ImageTk

#Translates subimage to number text
def img_to_number(img_frame, resize, resize_newsize, atribute_name, knn):
	number_contour = cv2.findContours(img_frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #Finds all contours in the image
	number_contour = number_contour[0] #Extracts only the contour data from the returned tuple
	good_contour, good_bounding_boxes, img_array, img_array_resized = [], [], [], []
	if(resize): #Set in initialize data and numberlearn.py to determine dataset image size
		resize_size = resize_newsize
	else:
		resize_size = (30,30)
	for i in number_contour: #Goes through all contours and checks if they are big enough to be a number
		(x,y,w,h) = cv2.boundingRect(i)
		if w>=4 and (h>= 16):
			good_contour.append(i)
			good_bounding_boxes.append(cv2.boundingRect(i))

	good_bounding_boxes_arr = np.array(good_bounding_boxes) 
	i = 0
	for i in range(good_bounding_boxes_arr.shape[0]): #Goes through all bounding boxes and extracts the number from the image
		y1 = good_bounding_boxes_arr[i,1].item()
		y2 = int(good_bounding_boxes_arr[i,1])+int(good_bounding_boxes_arr[i,3])
		x1 = int(good_bounding_boxes_arr[i,0])
		x2 = int(good_bounding_boxes_arr[i,0])+int(good_bounding_boxes_arr[i,2])
		tempimg = img_frame[y1:y2, x1:x2] #Creates a subimage from the bounding box coordinates of a single number
		img_array.append(tempimg.copy())  
		temp_resized_img = cv2.resize(tempimg, resize_size, interpolation=cv2.INTER_AREA) #Resizes the image to the same size as the training dataset
		img_array_resized.append(temp_resized_img.copy()) 
	img_array_resized = np.array(img_array_resized) #Array of all numbers in the image in order
	img_array_resized = img_array_resized.reshape(-1,(int(resize_size[0])*int(resize_size[1]))).astype(np.float32) #Flattens the image array to a single dimension
	ret,result,neighbours,dist = knn.findNearest(img_array_resized,k=1) #Using KNN finds the number in the image
	if(result is not None): 
		endstring = str(int(result[(len(result)-1)])) #Converts the result to a string
		i=0
		for i in range (len(result)-1):
			endstring = endstring + str(int(result[(len(result)-2-i)])) #Combines all numbers into a single number representation in string format
			
		if(len(result)>=7): #If the number is at least 7 digits long its either the latitude or longitude
			if(len(result)>7):
				endnum = int(endstring)/(10**(len(result)-2)) #Places the decimal point in the correct place
			else:
				return 0, atribute_name 
		else:
			endnum = int(endstring) #Can only be heading or altitude which dont need decimal point precision
		return endnum, atribute_name #Returns the number and the name of the analyzed feature for sorting
	else:
		return 0, atribute_name #If no number is found returns 0 and the name of the analyzed feature for sorting

#Realtime video feed obtains the first frame and pulls the coordinates that get returned. If the frame is not available, returns all coords as false
def video_get_gps(videofeed,lat_boundbox, lat_width, lat_height, lon_boundbox, lon_width,lon_height,alt_boundbox,alt_width,alt_height, heading_boundbox, heading_width, heading_height, resize, resize_newsize, knn, testosd):
	try:
		framestatus, frame = videofeed.read() #Reads a frame from the video feed
		if not framestatus:
			raise Exception [False, False, False, False] #On error returns all coords values as false 
		grayframe = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #Converts the frame to grayscale
		framestatus2, tresholdframe = cv2.threshold(grayframe,210,255,cv2.THRESH_TOZERO) #Removes background
		if not framestatus2:
			raise Exception [False, False, False, False] #On error returns all coords values as false
		returnimg = tresholdframe.copy() #Copies image that gets returned if in OSD testing mode
		lat_area = tresholdframe[lat_boundbox[0,1]:lat_boundbox[0,1]+lat_height, lat_boundbox[0,0]:lat_boundbox[0,0]+lat_width] #Creates a subimage from thresholded frame with latitude box coordinates
		lon_area = tresholdframe[lon_boundbox[0,1]:lon_boundbox[0,1]+lon_height, lon_boundbox[0,0]:lon_boundbox[0,0]+lon_width]	#Creates a subimage from thresholded frame with longitude box coordinates
		alt_area = tresholdframe[alt_boundbox[0,1]:alt_boundbox[0,1]+alt_height, alt_boundbox[0,0]:alt_boundbox[0,0]+alt_width]	#Creates a subimage from thresholded frame with altitude box coordinates
		heading_area = tresholdframe[heading_boundbox[0,1]:heading_boundbox[0,1]+heading_height, heading_boundbox[0,0]:heading_boundbox[0,0]+heading_width] #Creates a subimage from thresholded frame with heading box coordinates
		#Creates 4 parameter sets of img_to_number function parameters for each subimage for parallel processing
		paramlist = [(lat_area, resize, resize_newsize, 'lat', knn),(lon_area, resize, resize_newsize, 'lon', knn),(alt_area, resize, resize_newsize, 'alt', knn),(heading_area, resize, resize_newsize, 'heading', knn)]
		
		with concurrent.futures.ThreadPoolExecutor() as executor: #Runs all 4 parameter gathering concurently
			futures = [executor.submit(img_to_number, *parameters) for parameters in paramlist]
			results = [future.result() for future in concurrent.futures.as_completed(futures)]
		
		i, lat, lon, alt, heading=0,0,0,0,0 #Sets default values
		#Results returned by parallel processing are in the first processed first result order.
		for i in range(4): #Goes through all 4 results and assigns them to the correct variable
			if(results[i][1] == 'lat'):
				lat = float(results[i][0])
			if(results[i][1] == 'lon'):
				lon = float(results[i][0])
			if(results[i][1] == 'alt'):
				alt = int(results[i][0])
			if(results[i][1] == 'heading'):
				heading = int(results[i][0])
		coords = [lat, lon, alt, heading] #Coordinate array 
		if(initialize_data.debug): #Debug statement
			print(str(lat) + str(lon) + str(alt) +  str(heading) + ' OSD coordinates')
		if(testosd): #If in OSD testing mode, returns the coordinates and the image
			return coords, returnimg
		else:
			return coords
	except Exception as e:
		if(initialize_data.debug): #Debug statement
			coordstest = locals().get('coords', 'Null')
			print(str(coordstest) + ' OSD coordinates')
			print(e)
		return [False, False, False, False] #On error returns all coords values as false


#Loop of sample video feed that analyzes a frame and skips a set amount of frames before analyzing the next one and updates both the image and read coordinates.
def testvideo(videofeed, boundingbox_arr, videofps, capture_frequency,lat_boundbox, lat_width, lat_height, lon_boundbox, lon_width,lon_height,alt_boundbox,alt_width,alt_height, heading_boundbox, heading_width, heading_height, resize, resize_newsize, knn, sample_videofeed_coords, sample_videofeed, samplevideowindow ):
	while(videofeed.isOpened()):
		samplevideowindow.update_idletasks() #Updates sample video window to not freeze tkinter loop
		i = 0
		for i in range (int(videofps/capture_frequency)): #Skips a set amount of frames
			framestatus, frame = videofeed.read()
	
		if not framestatus: #If no more frames, stops the loop
			sample_videofeed_coords.config(text=('No more frames, stopping'))
			break
		grayframe = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #Converts the frame to grayscale
		framestatus2, tresholdframe = cv2.threshold(grayframe,210,255,cv2.THRESH_TOZERO) #Removes background
		if not framestatus2: #If no more frames, stops the loop
			sample_videofeed_coords.config(text=('No more frames, stopping'))
			break
		lat_area = tresholdframe[lat_boundbox[0,1]:lat_boundbox[0,1]+lat_height, lat_boundbox[0,0]:lat_boundbox[0,0]+lat_width] #Creates a subimage from thresholded frame with latitude box coordinates
		lon_area = tresholdframe[lon_boundbox[0,1]:lon_boundbox[0,1]+lon_height, lon_boundbox[0,0]:lon_boundbox[0,0]+lon_width] #Creates a subimage from thresholded frame with longitude box coordinates
		alt_area = tresholdframe[alt_boundbox[0,1]:alt_boundbox[0,1]+alt_height, alt_boundbox[0,0]:alt_boundbox[0,0]+alt_width] #Creates a subimage from thresholded frame with altitude box coordinates
		heading_area = tresholdframe[heading_boundbox[0,1]:heading_boundbox[0,1]+heading_height, heading_boundbox[0,0]:heading_boundbox[0,0]+heading_width] #Creates a subimage from thresholded frame with heading box coordinates
		#Creates 4 parameter sets of img_to_number function parameters for each subimage for parallel processing
		paramlist = [(lat_area, resize, resize_newsize, 'lat', knn),(lon_area, resize, resize_newsize, 'lon', knn),(alt_area, resize, resize_newsize, 'alt', knn),(heading_area, resize, resize_newsize, 'heading', knn)]
		with concurrent.futures.ThreadPoolExecutor() as executor: #Runs all 4 parameter gathering concurently
			futures = [executor.submit(img_to_number, *parameters) for parameters in paramlist]
			results = [future.result() for future in concurrent.futures.as_completed(futures)]
		
		i, lat, lon, alt, heading=0,0,0,0,0 #Sets default values
		for i in range(4): #Goes through all 4 results and assigns them to the correct variable
			if(results[i][1] == 'lat'):
				lat = float(results[i][0])
			if(results[i][1] == 'lon'):
				lon = float(results[i][0])
			if(results[i][1] == 'alt'):
				alt = int(results[i][0])
			if(results[i][1] == 'heading'):
				heading = int(results[i][0])
		coords = [lat, lon, alt, heading] #Coordinate array
		cv2.drawContours(tresholdframe, boundingbox_arr, -1 ,(255,255,255), 1) #Draws bounding boxes to the frame
		updateSampleVid(sample_videofeed_coords, sample_videofeed, tresholdframe.copy(), coords, samplevideowindow) #Updates the sample video window
		
		if cv2.waitKey(1000) == ord('q'): #If no more frames are obtained for 1s breaks the loop
			break
	videofeed.release()
	cv2.destroyAllWindows()

#Obtains frame from testvideo function and updates the sample video tkinter field with the new frame.
def updateSampleVid(sample_videofeed_coords, sample_videofeed, frame, coords,samplevideowindow):
	sample_videofeed_coords.config(text=str(coords)) #Updates the coordinates text field
	img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) #Converts the frame to RGB
	pil_img = Image.fromarray(img_rgb) #Converts the frame to a PIL image
	resized_img = pil_img.resize((700, 420)) #Resizes the image to fit the tkinter window
	tkinterimg = ImageTk.PhotoImage(resized_img) #Converts the PIL image to a tkinter image
	sample_videofeed.config(image=tkinterimg) #Updates the tkinter image field
	sample_videofeed.image=tkinterimg 
	samplevideowindow.update_idletasks() #Updates the tkinter window
	return