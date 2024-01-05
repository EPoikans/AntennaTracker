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

def initialize_data(useMavlink, useOSD, homegps_type, usesamplevid, accelerometer):
	global accelerometer_bool
	cv2.VideoCapture(0).release()
	
	accelerometer_bool = bool(accelerometer)
	if platform.system().lower() == 'linux':
		try:
			import RPi.GPIO as GPIO
			comp_setup = 'Raspi'
		except ImportError:
			comp_setup = 'PC'
	else:
		comp_setup = 'PC'
	if(useOSD):

		# Must be the same as training data!
		global resize_newsize
		resize_newsize = (20,20) #Default pictures are 30x30 px
		global resize
		resize = True
		#W
		#	Video feed input data collection
		#
		initial = True
		if(comp_setup == 'Raspi'):
			usesamplevid = False #Used in testing a video recording without flying drone, should be false when field testing
		else:
			usesamplevid = True
		global videofeed
		
		#usesamplevid=False #TEMP
		
		if(usesamplevid):
			videofeed = cv2.VideoCapture('./TestingFiles/drone_feed_test.mp4')
			#videofeed = cv2.VideoCapture('./TestingFiles/AvatarG0045_with_osd.mp4')
			videofps = videofeed.get(cv2.CAP_PROP_FPS)
			capture_frequency = 1 # analyzed frames per second, 1 recomened
		else:
			if(comp_setup == 'Raspi'):
				videofeed = cv2.VideoCapture(0, cv2.CAP_V4L2) #Value specifies which video input device is used camera or usb hdmi capture card
			else:
				videofeed = cv2.VideoCapture(0)
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
		connect_adress = findMavlinkRadioPort('windows')
		testfile = './TestingFiles/2023-09-22 12-26-58.tlog'
		if(comp_setup == 'Raspi'):
			connect_adress = findMavlinkRadioPort('raspberrypi')
			usetestfile = False #Used for testing without connection to drone using logs like in sample viewing, should be False for actual flights
		else:
			usetestfile = True
		if(usetestfile):
			usedaddress = testfile
		else:
			usedaddress = connect_adress
		global the_connection
		the_connection = mavutil.mavlink_connection(usedaddress, baud=57600)
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

def findMavlinkRadioPort(system):
	if(system == 'linux' or system == 'raspberrypi'):
		portstart = "/dev/ttyUSB"
	elif(system == 'windows'):
		portstart = "COM"
	i=0
	for i in range(256):
		serial_port = (str(portstart) + str(i))
		try:
			with serial.Serial(serial_port, 57600, timeout=1) as ser:
				time.sleep(1)
				ser.write(b"+++")
				response = ser.read(10)
				if (b"SiK") in response:
					ser.close()
					return serial_port
				else:
					ser.close()
		except:
			pass
		return ("Port not found")

def get_wh(boundbox_arr):
	width = abs(boundbox_arr[0,0]-boundbox_arr[1,0])
	height = abs(boundbox_arr[0,1]-boundbox_arr[2,1])
	return width,height