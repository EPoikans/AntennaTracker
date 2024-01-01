import threading
import time
from tkinter import *
from tkinter import ttk
import tkinter as tk
import numpy as np
import cv2
import initialize_data
import gps_calculation
import img_processing
import mavlink_msg_recieving
from collections.abc import Iterable
from PIL import Image, ImageTk
from pymavlink import mavutil
import servo_change
import serial_com
import platform


def main():
	global comp_setup, geometry_res
	if platform.system().lower() == 'linux':
		try:
			import RPi.GPIO as GPIO
			comp_setup = 'Raspi'
			geometry_res = '480x320'
		except ImportError:
			comp_setup = 'PC'
			geometry_res = '800x480'
	else:
		comp_setup = 'PC'
		geometry_res = '800x480'
	global mainwindow
	mainwindow = tk.Tk()
	mainwindow.geometry(geometry_res)
	mainwindow.title("AntennaTracker")
	home_pos_choice_label = tk.Label(mainwindow)
	home_pos_choice_label.grid(row=0, column=0, padx=10, pady=2)
	global home_pos_choice_selected
	home_pos_choice_selected = tk.IntVar()
	homebutton = tk.Radiobutton(mainwindow, text="Mavlink home", variable=home_pos_choice_selected, value=1)
	homebutton.grid(row=1, column=0, sticky="w", padx=8, pady=2)
	homebutton = tk.Radiobutton(mainwindow, text="OSD home", variable=home_pos_choice_selected, value=2)
	homebutton.grid(row=2, column=0, sticky="w", padx=8, pady=2)
	homebutton = tk.Radiobutton(mainwindow, text="Ground GPS", variable=home_pos_choice_selected, value=3)
	homebutton.grid(row=3, column=0, sticky="w", padx=8, pady=2)
	system_choice_label = tk.Label(mainwindow, text="Select which system or systems to use for obtaining GPS coordinates")
	system_choice_label.grid(row=0, column=1, padx=10, pady=2)
	global system_checkbox_var1
	system_checkbox_var1 = tk.IntVar()
	system_checkbox1 = tk.Checkbutton(mainwindow, text="Use OSD for GPS coordinates", variable=system_checkbox_var1)
	system_checkbox1.grid(row=1, column=1, sticky="w", padx=10, pady=5)
	global system_checkbox_var2
	system_checkbox_var2 = tk.IntVar()
	system_checkbox2 = tk.Checkbutton(mainwindow, text="Use Mavlink for GPS coordinates", variable=system_checkbox_var2)
	system_checkbox2.grid(row=2, column=1, sticky="w", padx=10, pady=5)
	"""
	global system_checkbox_var3
	system_checkbox_var3 = tk.IntVar()
	system_checkbox3 = tk.Checkbutton(mainwindow, text="Use accelerometer for vertical angle measurements", variable=system_checkbox_var3)
	system_checkbox3.grid(row=4, column=1, sticky="w", padx=10, pady=5)
	"""
	init_button = tk.Button(mainwindow, text="Initialize program", command=initialize)
	init_button.grid(row=5,column=0,columnspan=2, pady=5)
	init_pico = tk.Button(mainwindow, text="Initialize Pico", command=serial_com.init_pico) #No check if done currently!!!!!!!
	init_pico.grid(row=6,column=0,columnspan=2, pady=5)
	mavlink_sample= tk.Button(mainwindow, text="See mavlink sample", command=SampleMavlink)
	mavlink_sample.grid(row=10,column=1, pady=5)
	OSD_sample= tk.Button(mainwindow, text="See OSD sample", command=SampleVideo)
	OSD_sample.grid(row=12,column=1, pady=5)
	global error_label
	error_label = tk.Label(mainwindow, text="")
	error_label.grid(row=6, column=0, columnspan=2, pady=5)
	Return_btt= tk.Button(mainwindow, text="Quit", command=lambda: ReturnBttFn(mainwindow))
	Return_btt.grid(row=20,column=0, pady=5)
	if(comp_setup=='PC'):
		home_pos_choice_label.config(text="Select how to obtain home GPS position and compass heading")
		system_choice_label.config()
	elif(comp_setup=='Raspi'):
		print("")
	
	mainwindow.mainloop()

def initialize():
	home_gps_select = home_pos_choice_selected.get()
	global osd_for_gps
	osd_for_gps = system_checkbox_var1.get()
	global mavlink_for_gps
	mavlink_for_gps = system_checkbox_var2.get()
	accelerometer = False #system_checkbox_var3.get()
	global gps_home_window
	if(home_gps_select):
		if(osd_for_gps or mavlink_for_gps):
			if(home_gps_select == 1):
				if(mavlink_for_gps):
					error_label.config(text="")
					initialize_data.initialize_data(bool(mavlink_for_gps), bool(osd_for_gps), 'mavlink', False, accelerometer)
					gps_home_window = tk.Toplevel(mainwindow)
					gps_home_window.geometry(geometry_res)
					gps_home_window.title("Await home coordinates")
					home_label = tk.Label(gps_home_window, text="Arm drone to set home. Awaiting mavlink home set message")
					home_label.grid(row=0,column=0, pady=5)
					global home_gps_result_mav
					home_gps_result_mav = tk.Label(gps_home_window, text="No GPS data retrieved")
					home_gps_result_mav.grid(row=2,column=0, pady=5)
					home_gps_confirm = tk.Button(gps_home_window, text="Confirm", command=submitOSDHome)
					home_gps_confirm.grid(row=3,column=0, pady=5)
					Return_btt= tk.Button(gps_home_window, text="Return", command=lambda: ReturnBttFn(gps_home_window))
					Return_btt.grid_forget()
					global coords
					gps_home_window.update_idletasks()
					mavcoords = mavlink_msg_recieving.await_home_coords(initialize_data.the_connection)
					coords = [mavcoords[3],mavcoords[4],int(mavcoords[5]), int(mavcoords[2])]
					if(coords!="Timeout"):
						home_gps_result_mav.config(text=coords)
					else:
						home_gps_result_mav.config(text="Home set message not recieved in time, return and try again")
					Return_btt.grid(row=20,column=0, pady=5)
				else:
					error_label.config(text="Mavlink for GPS position is needed to obtain home coordinates via Mavlink")
			if(home_gps_select == 2):
				if(osd_for_gps):
					error_label.config(text="")
					initialize_data.initialize_data(bool(mavlink_for_gps), bool(osd_for_gps), 'OSD', False, accelerometer)
					gps_home_window = tk.Toplevel(mainwindow)
					gps_home_window.geometry(geometry_res)
					gps_home_window.title("Set home coordinates")
					home_label = tk.Label(gps_home_window, text="Set current drone GPS coordinates and heading as ground station")
					home_label.grid(row=0,column=0, pady=5)
					home_gps_button = tk.Button(gps_home_window, text="Get data", command=OSDHomePos)
					home_gps_button.grid(row=1,column=0, pady=5)
					global home_gps_result
					home_gps_result = tk.Label(gps_home_window, text="No GPS data retrieved")
					home_gps_result.grid(row=2,column=0, pady=5)
					home_gps_confirm = tk.Button(gps_home_window, text="Confirm", command=submitOSDHome)
					home_gps_confirm.grid(row=3,column=0, pady=5)
					Return_btt= tk.Button(gps_home_window, text="Return", command=lambda: ReturnBttFn(gps_home_window))
					Return_btt.grid(row=20,column=0, pady=5)
				else:
					error_label.config(text="OSD for GPS position is needed to obtain home coordinates via OSD")
			if(home_gps_select == 3):
				error_label.config(text="")
				initialize_data.initialize_data(bool(mavlink_for_gps), bool(osd_for_gps), 'GPS', False, accelerometer)
		else:
			error_label.config(text="Select the method for recieving realtime GPS coordinates")
	else:
		error_label.config(text="Select the method for obtaining home GPS coordinates")

def TestVideo(iter_count):
	testcoord, img = img_processing.video_get_gps(initialize_data.videofeed,initialize_data.lat_boundbox, initialize_data.lat_width,initialize_data.lat_height, initialize_data.lon_boundbox, initialize_data.lon_width,initialize_data.lon_height,initialize_data.alt_boundbox,initialize_data.alt_width,initialize_data.alt_height, initialize_data.heading_boundbox, initialize_data.heading_width, initialize_data.heading_height, initialize_data.resize, initialize_data.resize_newsize, initialize_data.knn, True)
	if(testcoord!=[False, False, False, False] and isinstance(img, Iterable)):
		osd_test_coords.config(text=testcoord)
		img_rgb = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2RGB)
		pil_img = Image.fromarray(img_rgb)
		resized_img = pil_img.resize((200, 120))
		tkinterimg = ImageTk.PhotoImage(resized_img)
		osd_test_screenshot.config(image=tkinterimg)
		osd_test_screenshot.image = tkinterimg
	else:
		osd_test_coords.config(text="No frame found")
		print("test")
	if(iter_count <= 100):
		testing_window.after(100,lambda: TestVideo(iter_count + 1))
	else:
		osd_test_screenshot.config(image=None)
		endstring = ('Test done, last coordinates -' + str(testcoord[0]) + " " + str(testcoord[1]) +  " " + str(testcoord[2]) + " " + str(testcoord[3]))
		osd_test_coords.config(text=endstring)
	
	
def TestVideoLoop():
	if(osd_for_gps):
		TestVideo(0)
	else:
		osd_test_coords.config(text="Not using OSD for GPS")

def TestMavlinkLoop():
	if(mavlink_for_gps):
		TestMavlink(mavlink_test1, mavlink_test2, mavlink_test3, mavlink_test4, 0,0)
	else:
		mavlink_test1.config(text = "Not using Mavlink for GPS")

def TestMavlink(mavlink_test1, mavlink_test2, mavlink_test3, mavlink_test4, i,iter):
	if(mavlink_for_gps):
		if(iter >= 100):
			mavlink_test1.config(text = "End of test")
			mavlink_test2.config(text = "End of test")
			mavlink_test3.config(text = "End of test")
			mavlink_test4.config(text = "End of test")
			return
		msg = mavlink_msg_recieving.test_mavlink_connection(initialize_data.the_connection)
		if(msg):
			if(i==0):
				mavlink_test1.config(text = str(msg["mavpackettype"]))
				testing_window.after(100, lambda: TestMavlink(mavlink_test1, mavlink_test2, mavlink_test3, mavlink_test4, i+1,iter+1))
			if(i==1):
				mavlink_test2.config(text = str(msg["mavpackettype"]))
				testing_window.after(100, lambda: TestMavlink(mavlink_test1, mavlink_test2, mavlink_test3, mavlink_test4, i+1,iter+1))
			if(i==2):
				mavlink_test3.config(text = str(msg["mavpackettype"]))
				testing_window.after(100, lambda: TestMavlink(mavlink_test1, mavlink_test2, mavlink_test3, mavlink_test4, i+1 ,iter+1))
			if(i==3):
				mavlink_test4.config(text = str(msg["mavpackettype"]))
				testing_window.after(100, lambda: TestMavlink(mavlink_test1, mavlink_test2, mavlink_test3, mavlink_test4, i+1,iter+1))
			if(i>=4):
				testing_window.after(100, lambda: TestMavlink(mavlink_test1, mavlink_test2, mavlink_test3, mavlink_test4, 0,iter+1))
		else:
			mavlink_test1.config(text = "No incoming messages")
			mavlink_test2.config(text = "No incoming messages")
			mavlink_test3.config(text = "No incoming messages")
			mavlink_test4.config(text = "No incoming messages")
	else:
		mavlink_test1.config(text = "Not using Mavlink for GPS")

def SampleVideo():
	global samplevideowindow
	initialize_data.initialize_data(False, True, '', False, False) 
	samplevideowindow = tk.Toplevel(mainwindow)
	samplevideowindow.geometry(geometry_res)
	samplevideowindow.title("Sample video")
	samplevideofeed = cv2.VideoCapture('./TestingFiles/drone_feed_test.mp4')
	videofps = samplevideofeed.get(cv2.CAP_PROP_FPS)
	capture_frequency = 1 # analyzed frames per second, 1 recomened
	global sample_videofeed, sample_videofeed_coords
	sample_videofeed = tk.Label(samplevideowindow)
	sample_videofeed.grid(row=2,columnspan=2, column=0)
	sample_videofeed_coords = tk.Label(samplevideowindow)
	sample_videofeed_coords.grid(row=1,column=0)
	Return_btt= tk.Button(samplevideowindow, text="Return", command=lambda: ReturnBttFn(samplevideowindow))
	Return_btt.grid_forget()
	img_processing.testvideo(samplevideofeed, initialize_data.boundingbox_arr, videofps, capture_frequency,initialize_data.lat_boundbox, initialize_data.lat_width, initialize_data.lat_height, initialize_data.lon_boundbox, initialize_data.lon_width,initialize_data.lon_height,initialize_data.alt_boundbox,initialize_data.alt_width,initialize_data.alt_height, initialize_data.heading_boundbox, initialize_data.heading_width, initialize_data.heading_height, initialize_data.resize, initialize_data.resize_newsize, initialize_data.knn, sample_videofeed_coords, sample_videofeed, samplevideowindow )
	Return_btt.grid(row=1,column=1)

def SampleMavlink():
	global sampleMavlinkWindow
	sampleMavlinkWindow = tk.Toplevel(mainwindow)
	sampleMavlinkWindow.geometry(geometry_res)
	sampleMavlinkWindow.title("Sample mavlink")
	the_connection_sample = mavutil.mavlink_connection('./TestingFiles/2023-09-22 12-26-58.tlog')
	the_connection_sample.wait_heartbeat()
	global mavlink_sample1, mavlink_sample2, mavlink_sample3, mavlink_sample4
	mavlink_sampletitle = tk.Label(sampleMavlinkWindow, text="1. time from boot 2. compass heading val 3. GPS heading val 4. GPS lat, 5. GPS lon 6. GPS relative altitude 7. GPS fix type 8. locked sattelite count")
	mavlink_sample1 = tk.Label(sampleMavlinkWindow, wraplength=780)
	mavlink_sample2 = tk.Label(sampleMavlinkWindow, wraplength=780)
	mavlink_sample3 = tk.Label(sampleMavlinkWindow, wraplength=780)
	mavlink_sample4 = tk.Label(sampleMavlinkWindow, wraplength=780)
	mavlink_sampletitle.grid(row=0,column=0, columnspan=2, pady=5)
	mavlink_sample1.grid(row=2,column=0, columnspan=2, pady=5)
	mavlink_sample2.grid(row=3,column=0, columnspan=2, pady=5)
	mavlink_sample3.grid(row=4,column=0, columnspan=2, pady=5)
	mavlink_sample4.grid(row=5,column=0, columnspan=2, pady=5)
	Return_btt= tk.Button(sampleMavlinkWindow, text="Return", command=lambda: ReturnBttFn(sampleMavlinkWindow))
	Return_btt.grid_forget()
	mavlink_msg_recieving.get_gps_logs(the_connection_sample,mavlink_sample1,mavlink_sample2,mavlink_sample3,mavlink_sample4, sampleMavlinkWindow)
	Return_btt.grid(row=1,column=1)

def OSDHomePos():
	global coords
	coords = img_processing.video_get_gps(initialize_data.videofeed,initialize_data.lat_boundbox, initialize_data.lat_width,initialize_data.lat_height, initialize_data.lon_boundbox, initialize_data.lon_width,initialize_data.lon_height,initialize_data.alt_boundbox,initialize_data.alt_width,initialize_data.alt_height, initialize_data.heading_boundbox, initialize_data.heading_width, initialize_data.heading_height, initialize_data.resize, initialize_data.resize_newsize, initialize_data.knn, False)
	home_gps_result.config(text=(coords))

def submitOSDHome():
	global gpshome
	gpshome = coords
	gps_home_window.destroy()
	workingWindow()

def testingWindow():
	global testing_window 
	testing_window = tk.Toplevel(workWindow)
	testing_window.title("Functionality testing")
	testing_window.geometry(geometry_res)
	
	mavlink_test = tk.Button(testing_window, text="Test mavlink incoming messages", command=TestMavlinkLoop)
	global mavlink_test1, mavlink_test2, mavlink_test3, mavlink_test4
	mavlink_test1 = tk.Label(testing_window)
	mavlink_test2 = tk.Label(testing_window)
	mavlink_test3 = tk.Label(testing_window)
	mavlink_test4 = tk.Label(testing_window)
	mavlink_test.grid(row=1,column=0, pady=5,)
	mavlink_test1.grid(row=2,column=0, pady=5)
	mavlink_test2.grid(row=3,column=0, pady=5)
	mavlink_test3.grid(row=4,column=0, pady=5)
	mavlink_test4.grid(row=5,column=0, pady=5)
	global osd_test_screenshot
	global osd_test_coords
	osd_test = tk.Button(testing_window, text="Test OSD coordinates and processed feed", command=TestVideoLoop)
	osd_test.grid(row=1,column=1, pady=5)
	
	osd_test_screenshot = tk.Label(testing_window)
	osd_test_screenshot.grid(row=3,rowspan=4,column=1, pady=5)
	
	osd_test_coords = tk.Label(testing_window)
	osd_test_coords.grid(row=2,column=1, pady=5)

	Return_btt= tk.Button(testing_window, text="Return", command=lambda: ReturnBttFn(testing_window))
	Return_btt.grid(row=15,column=0, pady=5)

def ReturnBttFn(window):
	window.destroy()

def StartTracking():
	global loop_running
	loop_running = True
	serial_com.init_pico
	time.sleep(0.5)
	threading.Thread(target=TrackingLoop).start()

def StopTracking():
	global loop_running
	loop_running = False

def TrackingLoop():
	global loop_running
	global osd_for_gps, mavlink_for_gps, gpshome, home_coords
	osd_lat_sanity = gpshome[0]
	osd_lon_sanity = gpshome[1]
	mav_lat_sanity = gpshome[0]
	mav_lon_sanity = gpshome[1]
	sanitycount, osd_sanitycount, mav_sanitycount, lastalt_osd, lastalt_mav = 0, 0, 0, 0, 0
	heading = gpshome[3]
	angle = 0
	if(len(gpshome) == 4):
		while loop_running:
			if(osd_for_gps):
				drone_coords_osd = img_processing.video_get_gps(initialize_data.videofeed,initialize_data.lat_boundbox, initialize_data.lat_width, initialize_data.lat_height, initialize_data.lon_boundbox, initialize_data.lon_width,initialize_data.lon_height,initialize_data.alt_boundbox,initialize_data.alt_width,initialize_data.alt_height, initialize_data.heading_boundbox, initialize_data.heading_width, initialize_data.heading_height, False, False, initialize_data.knn, False)
				if(drone_coords_osd != [False, False, False, False]):
					lat_diff = abs(osd_lat_sanity - drone_coords_osd[0])
					lon_diff = abs(osd_lon_sanity - drone_coords_osd[1])
					lastalt_osd = drone_coords_osd[2]
					if(lat_diff <= 0.009 and lon_diff <= 0.009): #About 500m distance change from last coordinate
						osd_lat_sanity = drone_coords_osd[0]
						osd_lon_sanity = drone_coords_osd[1]
					else:
						sanitycount+=1
						
				else:
					osd_sanitycount+=1
					print(osd_sanitycount)
			if(mavlink_for_gps):
				drone_coords_mav = mavlink_msg_recieving.get_gps_mavlink(initialize_data.the_connection)
				if(isinstance(drone_coords_mav, np.ndarray)):
					lat_diff = abs(mav_lat_sanity - drone_coords_mav[3])
					lon_diff = abs(mav_lon_sanity - drone_coords_mav[4])
					lastalt_mav = drone_coords_mav[5]
					if(lat_diff <= 0.01 and lon_diff <= 0.01): #About 1-1.5km distance change from last coordinate
						mav_lat_sanity = drone_coords_mav[3]
						mav_lon_sanity = drone_coords_mav[4]
					else:
						sanitycount+=1
				else:
					mav_sanitycount+=1
			if(osd_for_gps and mavlink_for_gps):
				if(isinstance(drone_coords_mav, np.ndarray) and drone_coords_osd != [False, False, False, False]):
					drone_coords = [((osd_lat_sanity + mav_lat_sanity)/2),((osd_lon_sanity + mav_lon_sanity)/2), ((lastalt_osd + lastalt_mav)/2)]
				elif(isinstance(drone_coords_mav, bool) and drone_coords_osd != [False, False, False, False]):
					drone_coords = [(osd_lat_sanity),(osd_lon_sanity),(lastalt_osd)]
				elif(isinstance(drone_coords_mav, np.ndarray) and drone_coords_osd == [False, False, False, False]):
					drone_coords = [(mav_lat_sanity),(mav_lon_sanity),(lastalt_mav)]
				else:
					drone_coords = dronecoords_save
				dronecoords_save = drone_coords
			elif(osd_for_gps and not mavlink_for_gps):
				drone_coords = [(osd_lat_sanity),(osd_lon_sanity),(lastalt_osd)]
				dronecoords_save = drone_coords
			elif(mavlink_for_gps and not osd_for_gps):
				drone_coords = [(mav_lat_sanity),(mav_lon_sanity),(lastalt_mav)]
				dronecoords_save = drone_coords
			else:
				drone_coords = dronecoords_save
			dronecoords_save[0] = int(dronecoords_save[0]*1000000000)/1000000000
			dronecoords_save[1] = int(dronecoords_save[1]*1000000000)/1000000000
			dronecoords_save[2] = int(dronecoords_save[2])
			direct_distance, newheading_from_home, new_angle = gps_calculation.calc_gps_distance(gpshome[0], gpshome[1], dronecoords_save[0], dronecoords_save[1], heading, angle, dronecoords_save[2])
			#if(abs(direct_distance_sanity - direct_distance)<=30):
			#	direct_distance_sanity = direct_distance
			#	new_angle = angle
			distancefromhome.config(text="Distance from home - " + str(int(direct_distance)) + " New heading - "+ str(int(newheading_from_home)) + " New angle - " + str(int(new_angle)))
			dronecoord.config(text="Drone coordinates - " + str(dronecoords_save))
			workWindow.after(50)
			heading = servo_change.headingchangeFn(heading, newheading_from_home, initialize_data.accelerometer_bool, gpshome[3])
			angle = servo_change.anglechangeFn(angle, new_angle, initialize_data.accelerometer_bool)
			if(sanitycount >=1000 or ((osd_sanitycount >=1000 or not osd_for_gps) and (mav_sanitycount >= 1000 or not mavlink_for_gps))):
				loop_running = False
				StartFailsafeTracking(heading, angle)
			time.sleep(0.2)
			
	else:
		home_coords.config(text=("Home coordinates - Invalid"))
		loop_running = False

def StartFailsafeTracking(heading, angle):
	global loop_running_failsafe
	loop_running_failsafe = True
	serial_com.init_pico
	time.sleep(0.5)
	threading.Thread(target=FailsafeTracking, args=(heading, angle)).start()

def StopTrackingFailsafe():
	global loop_running_failsafe
	loop_running_failsafe = False


def FailsafeTracking(lastheading, lastangle):
	global loop_running_failsafe
	global gpshome
	homeheading = gpshome[3]
	while loop_running_failsafe:
		workWindow.after(350)
		servo_change.headingChangeFailsafe(lastheading+20, homeheading)
		servo_change.angleChangeFailsafe(lastangle)
		workWindow.after(350)
		servo_change.headingChangeFailsafe(lastheading, homeheading)
		workWindow.after(350)
		servo_change.headingChangeFailsafe(lastheading-20, homeheading)
		workWindow.after(350)
		servo_change.headingChangeFailsafe(lastheading, homeheading)
		

def HaltTracker():
	global loop_running_failsafe, loop_running
	loop_running_failsafe, loop_running = False, False


def workingWindow():
	global workWindow
	workWindow = tk.Toplevel(mainwindow)
	workWindow.geometry(geometry_res)
	workWindow.title("Tracking")
	testing_win= tk.Button(workWindow, text="Test OSD coordinates and Mavlink processed feed", command=testingWindow)
	testing_win.grid(row=10,column=0, pady=50)
	global home_coords, distancefromhome,dronecoord 
	home_coords = tk.Label(workWindow)
	if(gpshome):
		home_coords.config(text=("Home coordinates(Lat, Lon, Alt, Heading) - " + str(gpshome)))
	else:
		home_coords.config(text=("Home coordinates - Null"))
	home_coords.grid(row=2,column=0, columnspan=3, pady=5)
	distancefromhome = tk.Label(workWindow)
	distancefromhome.grid(row=3,column=3, pady=5)
	dronecoord = tk.Label(workWindow)
	dronecoord.grid(row=4,column=3, pady=5)
	Start_btt= tk.Button(workWindow, text="Start antenna tracking", command=StartTracking)
	Start_btt.grid(row=8,column=0, pady=5)
	Stop_btt= tk.Button(workWindow, text="Stop antenna tracking", command=StopTracking)
	Stop_btt.grid(row=8,column=1, pady=5)
	Failsafe_Label = tk.Label(workWindow, text="Attempt to rotate antenna in large area in front")
	Failsafe_Label.grid(row=7,column=2, columnspan= 2, pady=5)
	Failsafe_btt= tk.Button(workWindow, text="Failsafe tracking", command=lambda: StartFailsafeTracking(gpshome[3], 40))
	Failsafe_btt.grid(row=8,column=2, pady=5)
	Failsafe_btt= tk.Button(workWindow, text="Failsafe tracking stop", command=StopTrackingFailsafe)
	Failsafe_btt.grid(row=9,column=2, pady=5)
	Halt_btt= tk.Button(workWindow, text="Halt tracker", command=HaltTracker)
	Halt_btt.grid(row=10,column=3, pady=5)
	Return_btt= tk.Button(workWindow, text="Return", command=lambda: ReturnBttFn(workWindow))
	Return_btt.grid(row=15,column=0, pady=50)

if __name__ == "__main__":
    main()
	