import cv2
import numpy as np
import initialize_data
import ui_window
import concurrent.futures
from PIL import Image, ImageTk

def img_to_number(img_frame, resize, resize_newsize, atribute_name, knn):
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
		if w>=4 and (h>= 16):
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
	if(result is not None):
		endstring = str(int(result[(len(result)-1)]))
		i=0
		for i in range (len(result)-1):
			endstring = endstring + str(int(result[(len(result)-2-i)]))
			
		if(len(result)>=7):
			if(len(result)>7):
				endnum = int(endstring)/(10**(len(result)-2))
			else:
				return 0, atribute_name
		else:
			endnum = int(endstring) 
		return endnum, atribute_name
	else:
		return 0, atribute_name

def video_get_gps(videofeed,lat_boundbox, lat_width, lat_height, lon_boundbox, lon_width,lon_height,alt_boundbox,alt_width,alt_height, heading_boundbox, heading_width, heading_height, resize, resize_newsize, knn, testosd):
	try:
		framestatus, frame = videofeed.read()
		if not framestatus:
			raise Exception [False, False, False, False] #On error returns all coords values as false 
		grayframe = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		framestatus2, tresholdframe = cv2.threshold(grayframe,235,255,cv2.THRESH_TOZERO) #Removes background
		if not framestatus2:
			raise Exception [False, False, False, False] #On error returns all coords values as false
		returnimg = tresholdframe.copy()
		lat_area = tresholdframe[lat_boundbox[0,1]:lat_boundbox[0,1]+lat_height, lat_boundbox[0,0]:lat_boundbox[0,0]+lat_width]
		lon_area = tresholdframe[lon_boundbox[0,1]:lon_boundbox[0,1]+lon_height, lon_boundbox[0,0]:lon_boundbox[0,0]+lon_width]
		alt_area = tresholdframe[alt_boundbox[0,1]:alt_boundbox[0,1]+alt_height, alt_boundbox[0,0]:alt_boundbox[0,0]+alt_width]
		heading_area = tresholdframe[heading_boundbox[0,1]:heading_boundbox[0,1]+heading_height, heading_boundbox[0,0]:heading_boundbox[0,0]+heading_width]
		paramlist = [(lat_area, resize, resize_newsize, 'lat', knn),(lon_area, resize, resize_newsize, 'lon', knn),(alt_area, resize, resize_newsize, 'alt', knn),(heading_area, resize, resize_newsize, 'heading', knn)]
		
		with concurrent.futures.ThreadPoolExecutor() as executor: #Runs all 4 parameter gathering concurently
			futures = [executor.submit(img_to_number, *parameters) for parameters in paramlist]
			results = [future.result() for future in concurrent.futures.as_completed(futures)]
		i, lat, lon, alt, heading=0,0,0,0,0
		for i in range(4):
			if(results[i][1] == 'lat'):
				lat = float(results[i][0])
			if(results[i][1] == 'lon'):
				lon = float(results[i][0])
			if(results[i][1] == 'alt'):
				alt = int(results[i][0])
			if(results[i][1] == 'heading'):
				heading = int(results[i][0])
		coords = [lat, lon, heading, alt]
		if(testosd):
			return coords, returnimg
		else:
			return coords
	except:
		return [False, False, False, False]

def testvideo(videofeed, boundingbox_arr, videofps, capture_frequency,lat_boundbox, lat_width, lat_height, lon_boundbox, lon_width,lon_height,alt_boundbox,alt_width,alt_height, heading_boundbox, heading_width, heading_height, resize, resize_newsize, knn, sample_videofeed_coords, sample_videofeed, samplevideowindow ):
	while(videofeed.isOpened()):
		samplevideowindow.update_idletasks()
		i = 0
		for i in range (int(videofps/capture_frequency)):
			framestatus, frame = videofeed.read()
	
		if not framestatus:
			sample_videofeed_coords.config(text=('No more frames, stopping'))
			break
		grayframe = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		framestatus2, tresholdframe = cv2.threshold(grayframe,235,255,cv2.THRESH_TOZERO) #Removes background
		if not framestatus2:
			sample_videofeed_coords.config(text=('No more frames, stopping'))
			break
		lat_area = tresholdframe[lat_boundbox[0,1]:lat_boundbox[0,1]+lat_height, lat_boundbox[0,0]:lat_boundbox[0,0]+lat_width]
		lon_area = tresholdframe[lon_boundbox[0,1]:lon_boundbox[0,1]+lon_height, lon_boundbox[0,0]:lon_boundbox[0,0]+lon_width]
		alt_area = tresholdframe[alt_boundbox[0,1]:alt_boundbox[0,1]+alt_height, alt_boundbox[0,0]:alt_boundbox[0,0]+alt_width]
		heading_area = tresholdframe[heading_boundbox[0,1]:heading_boundbox[0,1]+heading_height, heading_boundbox[0,0]:heading_boundbox[0,0]+heading_width]

		paramlist = [(lat_area, resize, resize_newsize, 'lat', knn),(lon_area, resize, resize_newsize, 'lon', knn),(alt_area, resize, resize_newsize, 'alt', knn),(heading_area, resize, resize_newsize, 'heading', knn)]
		with concurrent.futures.ThreadPoolExecutor() as executor: #Runs all 4 parameter gathering concurently
			futures = [executor.submit(img_to_number, *parameters) for parameters in paramlist]
			results = [future.result() for future in concurrent.futures.as_completed(futures)]
		i, lat, lon, alt, heading=0,0,0,0,0
		for i in range(4):
			if(results[i][1] == 'lat'):
				lat = float(results[i][0])
			if(results[i][1] == 'lon'):
				lon = float(results[i][0])
			if(results[i][1] == 'alt'):
				alt = int(results[i][0])
			if(results[i][1] == 'heading'):
				heading = int(results[i][0])
		coords = [lat, lon, heading, alt]
		cv2.drawContours(tresholdframe, boundingbox_arr, -1 ,(255,255,255), 1)
		updateSampleVid(sample_videofeed_coords, sample_videofeed, tresholdframe.copy(), coords, samplevideowindow)
		
		if cv2.waitKey(1000) == ord('q'):
			break
	videofeed.release()
	cv2.destroyAllWindows()

def updateSampleVid(sample_videofeed_coords, sample_videofeed, frame, coords,samplevideowindow):
	sample_videofeed_coords.config(text=str(coords))
	img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
	pil_img = Image.fromarray(img_rgb)
	resized_img = pil_img.resize((700, 420))
	tkinterimg = ImageTk.PhotoImage(resized_img)
	sample_videofeed.config(image=tkinterimg)
	sample_videofeed.image=tkinterimg
	samplevideowindow.update_idletasks()
	return
#videoloop(show_full_frame, videofeed,boundingbox_arr, videofps, capture_frequency,lat_boundbox, lat_width, lat_height, lon_boundbox, lon_width,lon_height,alt_boundbox,alt_width,alt_height, heading_boundbox, heading_width, heading_height)