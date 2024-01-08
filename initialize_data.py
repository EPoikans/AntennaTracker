import platform
import time
import cv2
import numpy as np
from pymavlink import mavutil
import serial
import ui_window
import serial.tools.list_ports

global gpshome
gpshome = []

global debug #Debug variable used in most files to print variable values if value is True
debug = False

def initialize_data(useMavlink, useOSD, homegps_type, usesamplevid, accelerometer):
	global accelerometer_bool #Currently retired
	cv2.VideoCapture(0).release() #Releases the first video device which should be a HDMI capture card incase its currently in use
	accelerometer_bool = bool(accelerometer) #Currently retired
	if platform.system().lower() == 'linux': #Checks if the computer is a raspberry pi or not
		try:
			import RPi.GPIO as GPIO #Non raspberry pi devices should not have RPi.GPIO module installed
			comp_setup = 'Raspi'
		except ImportError:
			comp_setup = 'PC'
	else:
		comp_setup = 'PC'
	if(debug):
		print(str(comp_setup) + ' computer used')

	if(useOSD): #If OSD is selected for use loads all the necessary data
		global resize_newsize
		resize_newsize = (20,20) #Default pictures are 30x30 px
		global resize
		resize = True # If true resizes the images to resize_newsize, must be the same value as in Numberlearn.py
		# 20x20px decreases training data file size and increases initial launch without sacrificing accuracy.

		#
		#	Video feed input data collection
		#
		initial = True #Used to wait for video feed to load on initial launch
		# usesamplevid variable is used in testing a video recording without flying drone identical to a recieved field if set to True
		if(comp_setup == 'Raspi'):
			usesamplevid = False #Raspberry pi is expected to be used during actual drone flights and is set to false by default. Set to True for testing the field setup without flying.
		else:
			usesamplevid = True #Set to false if controlling the antenna tracker in the field with another device that isnt a raspberry pi
		global videofeed, videofps, capture_frequency
		
		#usesamplevid=False #Uncomment the first # to override regardless of device for testing purposes
		
		if(usesamplevid):
			videofeed = cv2.VideoCapture('./TestingFiles/drone_feed_test.mp4') #Loads a video file instead of a live feed
			videofps = videofeed.get(cv2.CAP_PROP_FPS)
			capture_frequency = 1 # analyzed frames per second, 1 recomened
		else:
			if(comp_setup == 'Raspi'):
				videofeed = cv2.VideoCapture(0, cv2.CAP_V4L2) #Value specifies which video input device is used camera or usb hdmi capture card
			else:
				videofeed = cv2.VideoCapture(0) #cv2.CAP_V4L2 is for linux compatibility and might not work for windows. No specified video api works in windows
		while((videofeed.isOpened()!= True) and (initial == True)): #If videofeed is not opened during the initial launch waits 1s until it loads
			cv2.waitKey(1000) #Waits 1 seconds 
			if(debug):
				print('Awaiting video')
		initial = False #Initial launch is over
		
		x_res = videofeed.get(cv2.CAP_PROP_FRAME_WIDTH) #Gets the video feed resolution x value - field use is expected to be 1080p or 720p while some testing recordings are compressed to 960x544
		y_res = videofeed.get(cv2.CAP_PROP_FRAME_HEIGHT) #Gets the video feed resolution y value

		# Creates arrays of locations for the readable fields
		# Native 1920x1080, bounding box locations were obtained from a 960x544 sample video

		global lat_boundbox, lat_width, lat_height 
		lat_boundbox = np.array([ #Bounding box for the latitude location on the OSD
				[int(730/960*x_res),int(355/544*y_res)], #left top corner (x,y)
				[int(920/960*x_res),int(355/544*y_res)], #right top corner(x,y)
				[int(920/960*x_res),int(380/544*y_res)], #right bottom corner(x,y)
				[int(730/960*x_res),int(380/544*y_res)] #left bottom corner(x,y)
			])
		lat_width, lat_height =  get_wh(lat_boundbox) #Gets the width and height of the bounding box

		global lon_boundbox, lon_width, lon_height
		lon_boundbox = np.array([ #Bounding box for the longitude location on the OSD
				[int(730/960*x_res),int(326/544*y_res)], #left top corner (x,y)
				[int(920/960*x_res),int(326/544*y_res)], #right top corner(x,y)
				[int(920/960*x_res),int(354/544*y_res)], #right bottom corner(x,y)
				[int(730/960*x_res),int(354/544*y_res)] #left bottom corner(x,y)
			])
		lon_width, lon_height =  get_wh(lon_boundbox) #Gets the width and height of the bounding box

		global alt_boundbox, alt_width, alt_height
		alt_boundbox = np.array([ #Bounding box for the altitude location on the OSD
				[int(410/960*x_res),int(53/544*y_res)], #left top corner (x,y)
				[int(470/960*x_res),int(53/544*y_res)], #right top corner(x,y)
				[int(470/960*x_res),int(82/544*y_res)], #right bottom corner(x,y)
				[int(410/960*x_res),int(82/544*y_res)] #left bottom corner(x,y)
			])
		alt_width, alt_height =  get_wh(alt_boundbox) #Gets the width and height of the bounding box

		global heading_boundbox, heading_width, heading_height
		heading_boundbox = np.array([ #Bounding box for the heading location on the OSD
				[int(20/960*x_res),int(410/544*y_res)], #left top corner (x,y)
				[int(110/960*x_res),int(410/544*y_res)], #right top corner(x,y)
				[int(110/960*x_res),int(440/544*y_res)], #right bottom corner(x,y)
				[int(20/960*x_res),int(440/544*y_res)] #left bottom corner(x,y)
			])
		heading_width, heading_height =  get_wh(heading_boundbox) #Gets the width and height of the bounding box

		global boundingbox_arr #Array for all bounding boxes
		boundingbox_arr = np.array([lat_boundbox, lon_boundbox, alt_boundbox, heading_boundbox])

	if(useMavlink):
		connect_adress = findMavlinkRadioPort('windows') #Attempts to find COM port for the mavlink SiK radio
		testfile = './TestingFiles/2023-09-22 12-26-58.tlog' #Sample file of a flight
		if(comp_setup == 'Raspi'):
			connect_adress = findMavlinkRadioPort('raspberrypi') #Attempts to find the linux serial port for SiK radio
			usetestfile = False #Used for testing without connection to drone using logs like in sample viewing, should be False for actual flights
		else:
			usetestfile = True #Non raspberry pi devices are expected to be used for testing and are set to True by default. Set to False if using other devices for flights

		if(usetestfile): #Sets the connected serial port or file depending on previous parameters
			usedaddress = testfile
		else:
			usedaddress = connect_adress
		global the_connection #Mavlink connection
		if(debug): #Debug statement
			print(str(usedaddress) + "Mavlink connection adress")
		the_connection = mavutil.mavlink_connection(usedaddress, baud=57600) #Establishes a mavlink connection through a SiK telemetry radio or a flight log file
		the_connection.wait_heartbeat() #Waits for a heartbeat message to confirm the connection
		if(usetestfile == False): #If using a SiK radio sets the mavlink message interval to 4Hz
			the_connection.mav.command_long_send(
				the_connection.target_system, the_connection.target_component,
				mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
				33, # The MAVLink message ID for GLOBAL_POSITION_INT
				1e6 / 4, #Number after / is the frequency per second
				0, 0, 0, 0, # Unused parameters
				0,
			)
			
			the_connection.mav.command_long_send(
				the_connection.target_system, the_connection.target_component,
				mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
				74, # The MAVLink message ID for VFR_HUD
				1e6 / 4, #Number after / is the frequency per second
				0, 0, 0, 0, # Unused parameters
				0,
			)
			#For succesful message frequency change message all unused parameters should be set to 0. All possible parameters should have a value from mavlink documentation

def findMavlinkRadioPort(system): #Finds the serial port for the SiK radio
	if(system == 'linux' or system == 'raspberrypi'):
		portstart = "/dev/ttyUSB" #First part of linux serial port adress for the telemetry radio
	elif(system == 'windows'):
		portstart = "COM" #Windows serial port start
	i=0
	for i in range(256): #Windows can have up to 256 COM ports. Usually numbers 0-10 are used in windows and linux
		serial_port = (str(portstart) + str(i)) #Combines the system start of a serial port with the number
		try: #Sweeps all potential serial ports for any response. Either a SiK radio or the raspberry pi pico in this system will respond.
			with serial.Serial(serial_port, 57600, timeout=2) as ser:
				time.sleep(1) #Waits for the radio to establish a connection
				ser.write(b"ATI0\r\n") #Asks for banner_string in response
				response = ser.read(10) 
				if (b'') in response: #The SiK radio should respond with an empty bytearray
					ser.close()
					return serial_port
				else:
					ser.close()
		except:
			pass
	return ("Port not found")


def get_wh(boundbox_arr): #Gets the width and height of a bounding box
	width = abs(boundbox_arr[0,0]-boundbox_arr[1,0])
	height = abs(boundbox_arr[0,1]-boundbox_arr[2,1])
	return width,height