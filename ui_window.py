import io
import math
import threading
import time
from tkinter import ttk
import tkinter as tk
from tkinter import font
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
import preload_knn
from ipyleaflet import Map, Marker, basemaps
import folium
from folium.plugins import MarkerCluster

#UI debug/testing options
global debugRaspi, debugPC
debugRaspi = True #Forces raspberry pi UI
debugPC = False #Forces PC UI

def main(): #Main tkinter loop
	global comp_setup, geometry_res
	global mainwindow
	mainwindow = tk.Tk()

	#System specific settings
	if platform.system().lower() == 'linux': #Check if running on raspberry pi
		try:
			import RPi.GPIO as GPIO #RPi GPIO should only be installed on raspberry pi
			comp_setup = 'Raspi'
			geometry_res = '480x320'
			def_font = font.nametofont("TkDefaultFont")
			def_font.configure(size=14)
		except ImportError: #Linux PC that isn't a raspberry pi
			comp_setup = 'PC'
			geometry_res = '800x480'
			def_font = font.nametofont("TkDefaultFont")
			def_font.configure(size=10)
	else: #Every other OS
		comp_setup = 'PC'
		geometry_res = '800x480'
		def_font = font.nametofont("TkDefaultFont")
		def_font.configure(size=10)

	if(debugRaspi): #Debug option for forcing Raspberry pi UI
		comp_setup = 'Raspi'
		geometry_res = '480x320'
		def_font = font.nametofont("TkDefaultFont")
		def_font.configure(size=14)
	elif(debugPC): #Debut option for forcing PC UI
		comp_setup = 'PC'
		geometry_res = '800x480'
		def_font = font.nametofont("TkDefaultFont")
		def_font.configure(size=10)
	
	mainwindow.geometry(geometry_res)
	mainwindow.title("AntennaTracker")
	home_pos_choice_label = tk.Label(mainwindow)
	home_pos_choice_label.grid(row=0, column=0, padx=20, pady=15)

	#Three radio buttons for selecting home position obtaining method
	global home_pos_choice_selected
	home_pos_choice_selected = tk.IntVar()
	homebutton = tk.Radiobutton(mainwindow, text="Mavlink home", variable=home_pos_choice_selected, value=1)
	homebutton.grid(row=1, column=0, sticky="w", padx=20, pady=2)
	homebutton = tk.Radiobutton(mainwindow, text="OSD home", variable=home_pos_choice_selected, value=2)
	homebutton.grid(row=2, column=0, sticky="w", padx=20, pady=2)
	homebutton = tk.Radiobutton(mainwindow, text="Ground GPS", variable=home_pos_choice_selected, value=3)
	homebutton.grid(row=3, column=0, sticky="w", padx=20, pady=2)

	#Two checkboxes for selecting to use OSD or Mavlink or Both for realtime GPS coordinates from the drone
	system_choice_label = tk.Label(mainwindow)
	system_choice_label.grid(row=0, column=1, padx=20, pady=2)
	global system_checkbox_var1
	system_checkbox_var1 = tk.IntVar()
	system_checkbox1 = tk.Checkbutton(mainwindow, variable=system_checkbox_var1)
	system_checkbox1.grid(row=1, column=1, sticky="w", padx=20, pady=5)
	global system_checkbox_var2
	system_checkbox_var2 = tk.IntVar()
	system_checkbox2 = tk.Checkbutton(mainwindow, variable=system_checkbox_var2)
	system_checkbox2.grid(row=2, column=1, sticky="w", padx=20, pady=5)

	#Front page buttons
	init_button = tk.Button(mainwindow, command=initialize)
	init_button.grid(row=5,column=0,columnspan=2, pady=35)
	init_pico = tk.Button(mainwindow, command=serial_com.init_pico)
	init_pico.grid(row=7,column=0, pady=5)
	retry_serial = tk.Button(mainwindow, text="Retry Serial connection", command=serial_com.retrySerial)
	retry_serial.grid(row=7,column=1, pady=5)
	mavlink_sample= tk.Button(mainwindow, command=SampleMavlink)
	mavlink_sample.grid(row=10,column=1, pady=5)
	OSD_sample= tk.Button(mainwindow, command=SampleVideo)
	OSD_sample.grid(row=12,column=1, pady=5)
	global error_label
	error_label = tk.Label(mainwindow, text="")
	error_label.grid(row=6, column=0, columnspan=2, pady=5)
	Return_btt= tk.Button(mainwindow, command=lambda: ReturnBttFn(mainwindow))
	Return_btt.grid(row=20,column=0, pady=5)
	map_test= tk.Button(mainwindow, text="Map testing", command=MapTestWindow)
	map_test.grid(row=20,column=1, pady=5)

	#System specific changes to UI. Default values for layout and size are for PC that are overriden if raspberry pi is used.
	if(comp_setup=='PC'): #PC specific labels
		home_pos_choice_label.config(text="Select how to obtain home GPS position and compass heading")
		system_choice_label.config(text="Select which system(s) to use for obtaining GPS coords")
		system_checkbox1.config(text="Use OSD for GPS coordinates")
		system_checkbox2.config(text="Use Mavlink for GPS coordinates")
		init_button.config(text="Initialize program")
		init_pico.config(text="Initialize Pico")
		mavlink_sample.config(text="See mavlink sample")
		OSD_sample.config(text="See OSD sample")
		Return_btt.config(text="Quit")
	elif(comp_setup=='Raspi'): #Raspberry pi shortened labels and disabled & hidden buttons
		#Font size and choice for raspberry pi UI
		rp_font = font.nametofont("TkDefaultFont")
		rp_font.configure(size=16)
		home_pos_choice_label.config(text="Home GPS", font=rp_font)
		system_choice_label.config(text="Drone GPS", font=rp_font)
		system_checkbox1.config(text="OSD", font=rp_font)
		system_checkbox2.config(text="Mavlink", font=rp_font)
		init_button.config(text="Next", font=rp_font)
		Return_btt.config(text="Quit", font=rp_font)
		#Disabled buttons for unnecessary field features
		retry_serial.config(text="", state="disabled", bd=0, highlightthickness=0)
		init_pico.config(text="", state="disabled", bd=0, highlightthickness=0)
		map_test.config(text="", state="disabled", bd=0, highlightthickness=0)
		mavlink_sample.config(text="", state="disabled", bd=0, highlightthickness=0)
		OSD_sample.config(text="", state="disabled", bd=0, highlightthickness=0)
		#Layout changes for raspberry pi screen
		Return_btt.grid(row=5, column=0)
		init_button.grid(row=5,column=1, columnspan=2, pady=5)
		error_label.grid(row=6, column=1, columnspan=2, pady=2)
		system_checkbox1.grid(row=1, column=2, padx=70, sticky="w", pady=5)
		system_checkbox2.grid(row=2, column=2, padx=70, sticky="w", pady=5)
		system_choice_label.grid(row=0, column=2, padx=70, pady=2)
	
	mainwindow.mainloop()

def initialize():
	serial_com.init_pico()
	global home_gps_select
	home_gps_select = home_pos_choice_selected.get()
	global osd_for_gps
	osd_for_gps = system_checkbox_var1.get()
	global mavlink_for_gps
	mavlink_for_gps = system_checkbox_var2.get()
	global comp_setup
	accelerometer = False #system_checkbox_var3.get() If accelerometer feature is to be used.
	global gps_home_window
	global coords
	if(home_gps_select): #Checks if home position selection has been made
		if(osd_for_gps or mavlink_for_gps): #Checks if either OSD or Mavlink is selected for obtaining GPS coordinates

			#GPS home window labels and buttons for all three pathways
			gps_home_window = tk.Toplevel(mainwindow)
			gps_home_window.geometry(geometry_res)
			gps_home_window.title("Set home coordinates")
			home_label = tk.Label(gps_home_window)
			Return_btt= tk.Button(gps_home_window, text="Return", command=lambda: ReturnBttFn(gps_home_window))
			home_gps_confirm = tk.Button(gps_home_window, text="Confirm", command=submitOSDHome)

			#First path using mavlink for obtaining home coordinates
			if(home_gps_select == 1):
				if(mavlink_for_gps): #Checks if mavlink functionality checkbox is enabled
					error_label.config(text="") #Resets error label
					initialize_data.initialize_data(bool(mavlink_for_gps), bool(osd_for_gps), 'mavlink', False, accelerometer) #Initializes the apropriate data
					home_label.grid(row=0,column=0, pady=5,padx=200)
					global home_gps_result_mav
					home_gps_result_mav = tk.Label(gps_home_window, text="No GPS data retrieved")
					home_gps_result_mav.grid(row=2,column=0, pady=5)
					home_gps_confirm = tk.Button(gps_home_window, text="Confirm", command=submitOSDHome) #Home coordinate confirmation button
					home_gps_confirm.grid(row=3,column=0, pady=5)
					Return_btt.grid_forget()
					gps_home_window.update_idletasks()
					mavcoords = mavlink_msg_recieving.await_home_coords(initialize_data.the_connection) #Function for waiting for mavlink home message
					coords = [mavcoords[3],mavcoords[4],int(mavcoords[5]), int(mavcoords[2])] #Array with returned coordinates
					Return_btt.grid()

					#Computer system dependant text and layout setup
					if(comp_setup == 'PC'): #PC UI
						home_label.config(text="Await mavlink home set message", font=(font.nametofont("TkDefaultFont"), 18))
						home_gps_result_mav.config(font=(font.nametofont("TkDefaultFont"), 16), relief=tk.SUNKEN, bd=2)
						Return_btt.config(font=(font.nametofont("TkDefaultFont"), 16), bd=2, relief=tk.RAISED)
						home_gps_confirm.config(font=(font.nametofont("TkDefaultFont"), 16), bd=2, relief=tk.RAISED)
					else: #Raspberry pi UI
						#Font size and choice and alternate text
						home_label.config(text="Await Mavlink Home", font=(font.nametofont("TkDefaultFont"), 20))
						home_gps_result_mav.config(font=(font.nametofont("TkDefaultFont"), 20),bd=3, relief=tk.SUNKEN)
						home_gps_confirm.config(font=(font.nametofont("TkDefaultFont"), 20), bd=8, relief=tk.RAISED)
						Return_btt.config(font=(font.nametofont("TkDefaultFont"), 20), bd=8, relief=tk.RAISED)
						#Layout
						home_label.grid(row=0,column=0, padx=40, pady=3)
						home_gps_result_mav.grid(row=2,column=0, padx=40,pady=3)
						home_gps_confirm.grid(row=3,column=0,padx=40, pady=3)
						Return_btt.grid(row=20,column=0, padx=40,pady=3)
						Return_btt.grid(row=20,column=0, pady=5)

					if(coords!="Timeout"): #Coordinates are recieved and displayed based on system used
						if(comp_setup == 'PC'):
							home_gps_result_mav.config(text=("Latitude - " + str(coords[0]) + "  Longitude - " + str(coords[1]) + "  Heading - " + str(coords[2]) + "  Altitude - " + str(coords[3])))
						else:
							home_gps_result_mav.config(text=(coords))
					else: #Coordinates not recieved error messages
						if(comp_setup == 'PC'):
							home_gps_result_mav.config(text="Home set message not recieved in time, return and try again")
						else:
							home_gps_result_mav.config(text="Timeout err")
							
				else: #Error message in the main page if mavlink home is selected without mavlink functionality for realtime GPS coordinates
					if(comp_setup == 'PC'):
						error_label.config(text="Mavlink for GPS position is needed to obtain home coordinates via Mavlink")
					else: #Shortened error message for raspberry pi UI
						error_label.config(text="Mavlink GPS home err")

			#Second path using OSD for obtaining home coordinates
			if(home_gps_select == 2):
				if(osd_for_gps): #Checks if OSD functionality checkbox is enabled
					error_label.config(text="") #Resets error label
					initialize_data.initialize_data(bool(mavlink_for_gps), bool(osd_for_gps), 'OSD', False, accelerometer) #Initializes the apropriate data
					home_label.grid(row=0,column=0, pady=15)
					home_gps_button = tk.Button(gps_home_window, text="Get data", command=OSDHomePos, font=(font.nametofont("TkDefaultFont"), 26), bd=3) #Button for obtaining OSD home coordinates
					home_gps_button.grid(row=1,column=0, pady=15)
					global home_gps_result
					home_gps_result = tk.Label(gps_home_window, text="No GPS data retrieved")
					home_gps_result.grid(row=2,column=0, pady=15)
					home_gps_confirm.grid(row=3,column=0, pady=15)
					Return_btt.grid(row=20,column=0, pady=25)

					#Computer system dependant text and layout setup
					if(comp_setup == 'PC'): #PC UI
						home_label.config(text="Set current drone GPS coordinates and heading as ground station", font=(font.nametofont("TkDefaultFont"), 18))
						home_label.grid(padx=50)
						home_gps_result.config(font=(font.nametofont("TkDefaultFont"), 16), relief=tk.SUNKEN, bd=2)
						Return_btt.config(font=(font.nametofont("TkDefaultFont"), 26), bd=3)
						home_gps_confirm.config(font=(font.nametofont("TkDefaultFont"), 26), bd=3)
					else: #Raspberry pi UI
						#Font size and choice and alternate text
						home_label.config(text="Drone atitude == Station atitude", font=(font.nametofont("TkDefaultFont"), 20))
						home_gps_button.config(font=(font.nametofont("TkDefaultFont"), 20), bd=8, relief=tk.RAISED)
						home_gps_result.config(font=(font.nametofont("TkDefaultFont"), 20),bd=3, relief=tk.SUNKEN)
						home_gps_confirm.config(font=(font.nametofont("TkDefaultFont"), 20), bd=8, relief=tk.RAISED)
						Return_btt.config(font=(font.nametofont("TkDefaultFont"), 20), bd=8, relief=tk.RAISED)
						#Layout
						home_label.grid(row=0,column=0, padx=40, pady=3)
						home_gps_button.grid(row=1,column=0, padx=40,pady=3)
						home_gps_result.grid(row=2,column=0, padx=40,pady=3)
						home_gps_confirm.grid(row=3,column=0,padx=40, pady=3)
						Return_btt.grid(row=20,column=0, padx=40,pady=3)
				else: #Error message in the main page if OSD home is selected without OSD functionality for realtime GPS coordinates
					if(comp_setup == 'PC'):
						error_label.config(text="OSD for GPS position is needed to obtain home coordinates via OSD")
					else: #Shortened error message for raspberry pi UI
						error_label.config(text="OSD GPS home err")

			#Third path using ground GPS for obtaining home coordinates
			if(home_gps_select == 3):
				error_label.config(text="") #Resets error label
				initialize_data.initialize_data(bool(mavlink_for_gps), bool(osd_for_gps), 'GPS', False, accelerometer) #Initializes the apropriate data
				if(serial_com.getGPS() != 'No response'): #Checks if GPS module is connected to the raspberry pi pico. Will work with dummy data if Pico is not connected.
					global home_gps_result_gps
					home_label.grid(row=0,column=0, pady=5)
					home_gps_result_gps = tk.Label(gps_home_window, text="No GPS data retrieved")
					home_gps_result_gps.grid(row=2,column=0, columnspan=2, pady=5)
					home_gps_confirm.grid(row=3,column=0, pady=5)
					retry_btt = tk.Button(gps_home_window, text="Retry GPS", command=GetGPS, font=(font.nametofont("TkDefaultFont"), 22), bd=3) #Button for retrying GPS module data
					retry_btt.grid(row=4, column=0, pady=5)
					Return_btt.grid_forget()
					offset_compass1 = tk.Button(gps_home_window, text="Offset compass +10deg", command=offsetplus, font=(font.nametofont("TkDefaultFont"), 16), bd=3) #Buttons for offsetting the compass heading
					offset_compass2 = tk.Button(gps_home_window, text="Offset compass -10deg", command=offsetminus, font=(font.nametofont("TkDefaultFont"), 16), bd=3)
					offset_compass1.grid(row=4, column=1, pady=5)
					offset_compass2.grid(row=3, column=1, pady=5)
					GetGPS() #Gets the GPS data from the GPS module upon launch of window
					if(coords!="Timeout"): #If initial coordinates are recieved they are displayed based on system used
						if(comp_setup == 'PC'):
							home_gps_result_gps.config(text=('Latitude - ' + str(coords[0]) + '  Longitude - ' + str(coords[1]) + '  Heading - ' + str(coords[3]) + '  Altitude - ' + str(coords[2])))
						else:
							home_gps_result_gps.config(text=(str(coords)))
					else: #Initial coordinates not recieved
						home_gps_result_gps.config(text="No home coordinates retrieved")
					Return_btt.grid(row=20,column=0, pady=5)
					#System specific UI changes
					if(comp_setup == 'PC'):
						home_label.config(text="Local GPS module coordinates", font=(font.nametofont("TkDefaultFont"), 22), bd=3 )
						home_label.grid(padx=50, pady=30)
						home_gps_result_gps.config(font=(font.nametofont("TkDefaultFont"), 16), relief=tk.SUNKEN, bd=2)
						Return_btt.config(font=(font.nametofont("TkDefaultFont"), 22), bd=3)
						home_gps_confirm(font=(font.nametofont("TkDefaultFont"), 22), bd=3)
					else: #Raspberry pi UI
						#Font size and choice and alternate text
						home_label.config(text="GPS data from station", font=(font.nametofont("TkDefaultFont"), 20))
						retry_btt.config(font=(font.nametofont("TkDefaultFont"), 20), bd=8, relief=tk.RAISED)
						home_gps_result_gps.config(font=(font.nametofont("TkDefaultFont"), 20),bd=3, relief=tk.SUNKEN)
						home_gps_confirm.config(font=(font.nametofont("TkDefaultFont"), 20), bd=8, relief=tk.RAISED)
						Return_btt.config(font=(font.nametofont("TkDefaultFont"), 20), bd=8, relief=tk.RAISED)
						offset_compass1.config(text="+10deg", font=(font.nametofont("TkDefaultFont"), 20), bd=8, relief=tk.RAISED)
						offset_compass2.config(text="-10deg", font=(font.nametofont("TkDefaultFont"), 20), bd=8, relief=tk.RAISED)
						#Layout
						home_label.grid(row=0,column=0,columnspan=2, padx=40, pady=3)
						retry_btt.grid(row=4,column=0, padx=40,pady=3)
						home_gps_result_gps.grid(row=2,column=0,columnspan=2, padx=40,pady=3)
						home_gps_confirm.grid(row=3,column=0,columnspan=2, padx=40, pady=3)
						Return_btt.grid(row=20,column=0, padx=40,pady=3)
						offset_compass1.grid(row=4, column=1, pady=5)
						offset_compass2.grid(row=20, column=1, pady=5)
				else: #Error message in the main page if GPS module is not connected to the raspberry pi pico
					error_label.config(text="No response from GPS module")
		else: #Error message in the main page if neither OSD or Mavlink is selected for realtime GPS coordinates
			if(comp_setup == 'PC'):
				error_label.config(text="Select the method for recieving realtime GPS coordinates")
			else: #Shortened error message for raspberry pi UI
				error_label.config(text="Drone GPS method err")
	else: #Error message in the main page if home position selection has not been made
		if(comp_setup == 'PC'):
			error_label.config(text="Select the method for recieving realtime GPS coordinates")
		else: #Shortened error message for raspberry pi UI
			error_label.config(text="Drone GPS method err")

#Map window available only on PC UI for demonstration and checking servo direction and GPS calculations
def MapTestWindow():
	global map_window #Window settings
	map_window = tk.Toplevel(mainwindow)
	map_window.geometry("1600x900")
	map_window.title("Interactive map")

	global home_gps_result_gps #Current GPS location and heading labels
	home_label = tk.Label(map_window, text="Obtain coordinates from the GPS module as Map center point")
	home_label.grid(row=0,column=0,pady=20,padx=20)
	home_gps_result_gps = tk.Label(map_window, text="No GPS data retrieved")
	home_gps_result_gps.grid(row=2,column=0, pady=5, padx=50)

	retry_btt = tk.Button(map_window, text="Collect GPS", command=GetGPS) #Requests data from GPS module and magnetometer on the RP Pico
	retry_btt.grid(row=4, column=0, pady=5)

	global map_frame #Frame for the map image
	map_frame = tk.Label(map_window, width=800, height=480)
	map_frame.forget()

	createmap = tk.Button(map_window, text="Create Map", command=createMap) #Calls the map creation function which takes 5-10s. Tkinter window freezing in the meantime is normal.
	createmap.grid(row=5, column=0, pady=5)
	global datafield
	datafield = tk.Label(map_window) #Datafield for displaying GPS calculation results
	datafield.grid(row=6, column=0)

	Return_btt= tk.Button(map_window, text="Return", command=lambda: ReturnBttFn(map_window)) #Return button
	Return_btt.grid(row=0,column=1,pady=20)
	zero_pico = tk.Button(map_window, text="Zero Servos", command=serial_com.init_pico) #Reinitializes RP Pico and servos to the default state. Must do before each map creation.
	zero_pico.grid(row=1, column=1, pady=5)
	'''
	Magnetometer readings can be unstable due to servo motors. Compare to hand compass for accuracy and offset if needed.
	Generated map is North up, South down, West left, East right
	'''
	offset_compass1 = tk.Button(map_window, text="Offset compass +10deg", command=offsetplus) 
	offset_compass2 = tk.Button(map_window, text="Offset compass -10deg", command=offsetminus)
	offset_compass1.grid(row=2, column=1, pady=5)
	offset_compass2.grid(row=3, column=1, pady=5)

def offsetplus(): #Coordinate offset function
	global coords
	coords[3]+=10
	if(coords[3]>=360): #Handles magnetometer value going from 360 to 0
		coords[3]-=360
	if(comp_setup=='PC'): #Updates the coordinates with changed heading value
		home_gps_result_gps.config(text=("Latitude - " + str(coords[0]) + "  Longitude - " + str(coords[1]) + "  Heading - " + str(coords[3]) + "  Altitude - " + str(coords[2])))
	else:
		home_gps_result_gps.config(text=(str(coords)))

def offsetminus(): #Coordinate offset function
	global coords
	coords[3]-=10
	if(coords[3]<=0): #Handles magnetometer value going from 0 to 360
		coords[3]+=360
	if(comp_setup=='PC'): #Updates the coordinates with changed heading value
		home_gps_result_gps.config(text=("Latitude - " + str(coords[0]) + "  Longitude - " + str(coords[1]) + "  Heading - " + str(coords[3]) + "  Altitude - " + str(coords[2])))
	else:
		home_gps_result_gps.config(text=(str(coords)))
	
def createMap():
	global coords
	#Obtained from the previously used GPS module
	homelat, homelon = float(coords[0]), float(coords[1])

	global map_instance, map_frame, map_window, marker_cluster, bounds
	mapradius = 250 #Radius for map in meters
	bound_offset = (mapradius/1000)/111.2 # 111.2km per degree of latitude
	bounds = [[homelat - bound_offset, homelon - bound_offset], [homelat + bound_offset, homelon + bound_offset]]
	map_instance = folium.Map(location=[homelat, homelon], zoom_start=13) #Creates folium map
	marker_cluster = MarkerCluster().add_to(map_instance) #Creates marker instance for the map
	map_instance.fit_bounds(bounds) #Resizes the map
	folium.Marker([homelat, homelon], popup="Home Location").add_to(marker_cluster) #Adds home location marker to the map
	img_data = map_instance._to_png(1) #Converts map to png screenshot (Map is an interactive HTML file but displaying it isnt supported by tkinter)
	img = Image.open(io.BytesIO(img_data))
	img = img.resize((800, 480), Image.ANTIALIAS) #Resizes the image to fit the tkinter window
	img_tk = ImageTk.PhotoImage(img)
	map_frame.config(image=img_tk) #Displays the image
	map_frame.image = img_tk
	map_frame.grid(row=7, column=0, columnspan=2) #Shows the map frame
	map_frame.bind("<Button-1>", click_map) #Function for collecting left clicks on map
	
def click_map(event): #Starts seperate thread to not freeze tkinter with x,y coordinates of the map PNG
	threading.Thread(target=process_click, args=(event,)).start()

def process_click(event):
	angle=10 #Default value
	global map_instance, map_frame, coords, marker_cluster, bounds
	lon = float(bounds[0][1] + event.x / map_frame.winfo_width() * (bounds[1][1] - bounds[0][1])) #Converts png x,y coordinates to longitude
	lat = float(bounds[1][0] - event.y / map_frame.winfo_height() * (bounds[1][0] - bounds[0][0])) #Converts png x,y coordinates to latitude
	heading = float(coords[3]) #Heading from home
	setaltitude = 50 #Forced fake relative altitude of 'drone' for calculations
	direct_distance, newheading_from_home, new_angle = gps_calculation.alternate_calc_gps_distance(coords[0], coords[1], lat, lon, heading, angle, setaltitude) #GPS function
	angle = servo_change.anglechangeFn(new_angle, new_angle, False) #Sets angle servo first
	heading = servo_change.headingchangeFn(newheading_from_home, newheading_from_home, False, float(coords[3])) #Sets heading servo
	#Displays collected data from calculations between home position and click on the map
	datafield.config(text=("Ground distance - " + str(int(math.sqrt(direct_distance*direct_distance - setaltitude*setaltitude))) + " Direct distance - " + str(direct_distance) + " Calculated new heading - " + str(newheading_from_home) + " New angle at 100m alt - " + str(new_angle)))


def GetGPS():
	global coords
	latlon = serial_com.getGPS()
	#Get GPS may take 0.5-1s and if the magnetometer function gets called too early returning data might be crossed
	#This function is used by two windows so it forces a 500ms wait on the window active at the time
	try:
		gps_home_window.after(500)
	except:
		pass
	try:
		map_window.after(500)
	except:
		pass
	#Obtains magnetometer heading from the RP Pico
	heading = serial_com.getMagnetometer()
	lat = latlon[0]
	lon = latlon[1]
	coords = [lat,lon,0, heading] #Array with coordinates and heading and altitude
	if(coords!="Timeout"): #Coordinates are recieved and displayed based on system used
		if(comp_setup=='PC'):
			home_gps_result_gps.config(text=("Latitude - " + str(coords[0]) + "  Longitude - " + str(coords[1]) + "  Heading - " + str(coords[3]) + "  Altitude - " + str(coords[2])))
		else:
			home_gps_result_gps.config(text=(str(coords)))
	else:
		home_gps_result_gps.config(text="No home coordinates retrieved")
	#Updates the used window at the time
	try:
		gps_home_window.update_idletasks()
	except:
		pass
	try:
		map_window.update_idletasks()
	except:
		pass


#Used to see if the program sees frames on the HDMI capture card before starting tracking
def TestVideo(iter_count):
	#Obtains coordinates and image frame
	testcoord, img = img_processing.video_get_gps(initialize_data.videofeed,initialize_data.lat_boundbox, initialize_data.lat_width,initialize_data.lat_height, initialize_data.lon_boundbox, initialize_data.lon_width,initialize_data.lon_height,initialize_data.alt_boundbox,initialize_data.alt_width,initialize_data.alt_height, initialize_data.heading_boundbox, initialize_data.heading_width, initialize_data.heading_height, initialize_data.resize, initialize_data.resize_newsize, preload_knn.knn, True)
	if(testcoord!=[False, False, False, False] and isinstance(img, Iterable)): #Checks if coordinates are valid and image frame is not empty
		osd_test_coords.config(text=testcoord) #Displays coordinates
		img_rgb = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2RGB) #Converts image frame to RGB
		pil_img = Image.fromarray(img_rgb) #Converts image frame to PIL image
		resized_img = pil_img.resize((200, 120)) 
		tkinterimg = ImageTk.PhotoImage(resized_img) #Converts PIL image to tkinter image
		osd_test_screenshot.config(image=tkinterimg) #Displays image frame
		osd_test_screenshot.image = tkinterimg
	else: 
		osd_test_coords.config(text="No frame found")
	if(iter_count <= 100): #Timeout after 100 frames.
		testing_window.after(100,lambda: TestVideo(iter_count + 1))
	else:
		osd_test_screenshot.config(image=None)
		if(comp_setup == "PC"):
			endstring = ('Test done, last coordinates -' + str(testcoord[0]) + " " + str(testcoord[1]) +  " " + str(testcoord[2]) + " " + str(testcoord[3]))
		else:
			endstring = ('Test done')
		osd_test_coords.config(text=endstring)
	
#Loop that starts 100 iterations of the video feed
def TestVideoLoop():
	if(osd_for_gps):
		TestVideo(0)
	else: #Error message if OSD functionality wasn't selected before but attempted to get video feed test
		osd_test_coords.config(text="Not using OSD for GPS")

#Used to see if the program sees mavlink messages before starting tracking
def TestMavlinkLoop(): 
	if(mavlink_for_gps): #Calls 100 iterations of message recieving
		TestMavlink(mavlink_test1, mavlink_test2, mavlink_test3, mavlink_test4, 0,0)
	else: #Error message if mavlink functionality wasn't selected before but attempted to get mavlink messages
		mavlink_test1.config(text = "Not using Mavlink for GPS")

def TestMavlink(mavlink_test1, mavlink_test2, mavlink_test3, mavlink_test4, i,iter): #Recursive function for scrolling mavlink messages
	if(mavlink_for_gps):
		if(iter >= 100): #End of test
			mavlink_test1.config(text = "End of test")
			mavlink_test2.config(text = "End of test")
			mavlink_test3.config(text = "End of test")
			mavlink_test4.config(text = "End of test")
			return
		msg = mavlink_msg_recieving.test_mavlink_connection(initialize_data.the_connection) #Recieves single message
		if(msg):
			if(i==0): #Displays on apropriate label based on iteration
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
		else: #No message recieved
			mavlink_test1.config(text = "No incoming messages")
			mavlink_test2.config(text = "No incoming messages")
			mavlink_test3.config(text = "No incoming messages")
			mavlink_test4.config(text = "No incoming messages")
	else: #System specific error messages if mavlink functionality wasn't selected before but attempted to get mavlink messages
		if(comp_setup=='PC'):
			mavlink_test1.config(text = "Not using Mavlink for GPS")
		else:
			mavlink_test1.config(text = "Mavlink err")

#Runs the sample video and displays read coordinates
def SampleVideo():
	global samplevideowindow
	initialize_data.initialize_data(False, True, '', False, False) #Dummy data initialization with OSD used
	samplevideowindow = tk.Toplevel(mainwindow)
	samplevideowindow.geometry(geometry_res)
	samplevideowindow.title("Sample video")
	samplevideofeed = cv2.VideoCapture('./TestingFiles/drone_feed_test.mp4') #Filepath for the sample video
	global sample_videofeed, sample_videofeed_coords
	sample_videofeed = tk.Label(samplevideowindow)
	sample_videofeed.grid(row=2,columnspan=2, column=0)
	sample_videofeed_coords = tk.Label(samplevideowindow) #Coordinate label that is updated
	sample_videofeed_coords.grid(row=1,column=0)
	Return_btt= tk.Button(samplevideowindow, text="Return", command=lambda: ReturnBttFn(samplevideowindow)) 
	Return_btt.grid_forget() #hidden button until sample is over
	img_processing.testvideo(samplevideofeed, initialize_data.boundingbox_arr, initialize_data.videofps, initialize_data.capture_frequency,initialize_data.lat_boundbox, initialize_data.lat_width, initialize_data.lat_height, initialize_data.lon_boundbox, initialize_data.lon_width,initialize_data.lon_height,initialize_data.alt_boundbox,initialize_data.alt_width,initialize_data.alt_height, initialize_data.heading_boundbox, initialize_data.heading_width, initialize_data.heading_height, initialize_data.resize, initialize_data.resize_newsize, preload_knn.knn, sample_videofeed_coords, sample_videofeed, samplevideowindow )
	Return_btt.grid(row=1,column=1) 

#Displays sample mavlink messages until drone alivetime is over 3000
def SampleMavlink():
	global sampleMavlinkWindow
	sampleMavlinkWindow = tk.Toplevel(mainwindow)
	sampleMavlinkWindow.geometry(geometry_res)
	sampleMavlinkWindow.title("Sample mavlink")
	the_connection_sample = mavutil.mavlink_connection('./TestingFiles/2023-09-22 12-26-58.tlog')
	the_connection_sample.wait_heartbeat() #Waits for the first heartbeat incoming message
	global mavlink_sample1, mavlink_sample2, mavlink_sample3, mavlink_sample4
	#Labels for the meaning of each part of the recieved and processed message
	mavlink_sampletitle = tk.Label(sampleMavlinkWindow, text="1. time from boot 2. compass heading val 3. GPS heading val 4. GPS lat, 5. GPS lon 6. GPS relative altitude 7. GPS fix type 8. locked sattelite count", font=(font.nametofont("TkDefaultFont"), 9))
	mavlink_sample1 = tk.Label(sampleMavlinkWindow, wraplength=780) #First rolling text field
	mavlink_sample2 = tk.Label(sampleMavlinkWindow, wraplength=780) #Second rolling text field
	mavlink_sample3 = tk.Label(sampleMavlinkWindow, wraplength=780) #Third rolling text field
	mavlink_sample4 = tk.Label(sampleMavlinkWindow, wraplength=780) #Fourth rolling text field
	mavlink_sampletitle.grid(row=0,column=0, columnspan=2, pady=5)
	mavlink_sample1.grid(row=2,column=0, columnspan=2, pady=5)
	mavlink_sample2.grid(row=3,column=0, columnspan=2, pady=5)
	mavlink_sample3.grid(row=4,column=0, columnspan=2, pady=5)
	mavlink_sample4.grid(row=5,column=0, columnspan=2, pady=5)
	Return_btt= tk.Button(sampleMavlinkWindow, text="Return", command=lambda: ReturnBttFn(sampleMavlinkWindow))
	Return_btt.grid_forget() #hidden button until sample is over
	mavlink_msg_recieving.get_gps_logs(the_connection_sample,mavlink_sample1,mavlink_sample2,mavlink_sample3,mavlink_sample4, sampleMavlinkWindow)
	Return_btt.grid(row=1,column=1)
'''
Gets current OSD displayed coordinates for setting that as the home position
Drone coordinates must be close to the ground station and the heading must be the same as the 0 position of the tracker.
'''
def OSDHomePos():
	global coords
	coords = img_processing.video_get_gps(initialize_data.videofeed,initialize_data.lat_boundbox, initialize_data.lat_width,initialize_data.lat_height, initialize_data.lon_boundbox, initialize_data.lon_width,initialize_data.lon_height,initialize_data.alt_boundbox,initialize_data.alt_width,initialize_data.alt_height, initialize_data.heading_boundbox, initialize_data.heading_width, initialize_data.heading_height, initialize_data.resize, initialize_data.resize_newsize, preload_knn.knn, False)
	if(comp_setup=='PC'): #Updates label with read coordinates depending on system
		home_gps_result.config(text=("Latitude - " + str(coords[0]) + "  Longitude - " + str(coords[1]) + "  Heading - " + str(coords[2]) + "  Altitude - " + str(coords[3])))
	else: #Raspberry pi
		home_gps_result.config(text=(str(coords)))

#Confirms Home coordinates and destroys the window for obtaining home coordinates
def submitOSDHome():
	global gpshome
	gpshome = coords
	gps_home_window.destroy()
	workingWindow()

#Window for testing realtime incoming messages and read coordinates
def testingWindow():
	global testing_window 
	testing_window = tk.Toplevel(workWindow)
	testing_window.title("Functionality testing")
	testing_window.geometry(geometry_res)
	#Mavlink message test button
	mavlink_test = tk.Button(testing_window, command=TestMavlinkLoop)
	global mavlink_test1, mavlink_test2, mavlink_test3, mavlink_test4
	mavlink_test1 = tk.Label(testing_window)
	mavlink_test2 = tk.Label(testing_window)
	mavlink_test3 = tk.Label(testing_window)
	mavlink_test4 = tk.Label(testing_window)
	mavlink_test.grid(row=1,column=0, pady=5)
	mavlink_test1.grid(row=2,column=0, pady=5)
	mavlink_test2.grid(row=3,column=0, pady=5)
	mavlink_test3.grid(row=4,column=0, pady=5)
	mavlink_test4.grid(row=5,column=0, pady=5)

	#OSD test buttons and labels
	global osd_test_screenshot, osd_test_coords
	osd_test = tk.Button(testing_window, command=TestVideoLoop)
	osd_test.grid(row=1,column=1, pady=5)
	osd_test_screenshot = tk.Label(testing_window)
	osd_test_screenshot.grid(row=3,rowspan=4,column=1, pady=5)
	osd_test_coords = tk.Label(testing_window)
	osd_test_coords.grid(row=2,column=1, pady=5)
	Return_btt= tk.Button(testing_window, text="Return", command=lambda: ReturnBttFn(testing_window))
	Return_btt.grid(row=15,column=0, pady=5)
	#System specific UI changes
	if(comp_setup == 'PC'):
		mavlink_test.grid(padx=100)
		Return_btt.grid(pady=100)
		mavlink_test.config(text="Test Mavlink incoming messages")
		osd_test.config(text="Test OSD coordinates and processed feed")
	else:
		rp_font = font.nametofont("TkDefaultFont")
		rp_font.configure(size=11)
		mavlink_test1.config(font=rp_font)
		mavlink_test2.config(font=rp_font)
		mavlink_test3.config(font=rp_font)
		mavlink_test4.config(font=rp_font)
		osd_test_coords.config(font=rp_font)
		mavlink_test.config(font=rp_font)
		mavlink_test.config(text="Test Mavlink")
		mavlink_test.grid(padx=65)
		osd_test.grid(padx=85)
		osd_test.config(text="Test OSD")
		Return_btt.grid(pady=45)

#Function for closing the called window
def ReturnBttFn(window):
	window.destroy()

#Starts the tracking loop without freezing the tkinter window
def StartTracking():
	global loop_running
	loop_running = True
	serial_com.init_pico #Resets RP Pico to zero
	time.sleep(0.5)
	threading.Thread(target=TrackingLoop).start()

#Stops the tracking loop
def StopTracking():
	global loop_running
	loop_running = False

def TrackingLoop():
	global loop_running
	global home_gps_select
	global osd_for_gps, mavlink_for_gps, gpshome, home_coords
	'''
	If antenna tracker home is set by mavlink or OSD it is expected to be within couple of meters of the drone
	Sanity values keep unrealistic coordinates from affecting the tracking - one loop is faster than a normal drone can travel 1km
	'''
	if(home_gps_select == 1 or home_gps_select == 2):
		osd_lat_sanity = float(gpshome[0])
		osd_lon_sanity = float(gpshome[1])
		mav_lat_sanity = float(gpshome[0])
		mav_lon_sanity = float(gpshome[1])
		skipfirstosd, skipfirstmav = 0, 0
	else: #GPS home causes trouble in testing if point of testing is far away from sample flight location so first sanity is obtained from first frame of mavlink or OSD coordinates
		osd_lat_sanity = 0
		osd_lon_sanity = 0
		mav_lat_sanity = 0
		mav_lon_sanity = 0
		skipfirstosd, skipfirstmav = 1, 1
	sanitycount, osd_sanitycount, mav_sanitycount, lastalt_osd, lastalt_mav = 0, 0, 0, 0, 0
	heading = int(gpshome[3]) #Initial heading is obtained from home heading
	angle = 0 
	if(isinstance(gpshome, list)): #Sets initial values for drone coordinate saving 
		dronecoords_save = [gpshome[0],gpshome[1],gpshome[2]]
	else: #Kills tracking loop if home coordinates arent a valid list
		loop_running = False
	if(len(gpshome) == 4): #Home coordinates should be a list of 4 elements - lat, lon, rel. alt, heading
		while loop_running: #Tracking loop that is killed by StopTracking() function
			'''
			Obtains coordinates from video feed frame if the option is selected
			This method uses a KNN model to read the coordinates from the OSD and is more susceptible to noise and faulty reading due to snow or bright sky
			The sanity calculation therefore allows a max of 500m of change in coordinates from the last frame
			'''
			if(osd_for_gps):
				#Obtains coordinates
				drone_coords_osd = img_processing.video_get_gps(initialize_data.videofeed,initialize_data.lat_boundbox, initialize_data.lat_width, initialize_data.lat_height, initialize_data.lon_boundbox, initialize_data.lon_width,initialize_data.lon_height,initialize_data.alt_boundbox,initialize_data.alt_width,initialize_data.alt_height, initialize_data.heading_boundbox, initialize_data.heading_width, initialize_data.heading_height, False, False, preload_knn.knn, False)
				if(drone_coords_osd != [False, False, False, False]): #If coordinates are valid coordinates
					lat_diff = abs(float(osd_lat_sanity) - float(drone_coords_osd[0])) 
					lon_diff = abs(float(osd_lon_sanity) - float(drone_coords_osd[1]))
					lastalt_osd = int(drone_coords_osd[2]) 
					if((lat_diff <= 0.009 and lon_diff <= 0.009) or skipfirstosd == 1): #About 500m distance change from last coordinate
						osd_lat_sanity = float(drone_coords_osd[0])
						osd_lon_sanity = float(drone_coords_osd[1])
						if(skipfirstosd == 1): #If GPS home is used skips the first sanity check
							skipfirstosd +=1
					else: #If coordinates are too far from the last frame adds to  the sanity counter
						sanitycount+=1
				else:
					osd_sanitycount+=1 #If OSD coordinates arent recieved its added to the osd sanity counter
					if(initialize_data.debug): #Debug print
						print(osd_sanitycount)
			'''
			Mavlink coordinates are recieved in text format and are more reliable than OSD coordinates with a shorter range
			The sanity calculation allows roughly 1km deviation from last frame
			Faulty data should only occur if the mavlink messages get corrupted in air or drone GPS module is faulty
			'''
			if(mavlink_for_gps):
				drone_coords_mav = mavlink_msg_recieving.get_gps_mavlink(initialize_data.the_connection)
				print(str(drone_coords_mav[3]), str(drone_coords_mav[4]) + 'coords')
				if(isinstance(drone_coords_mav, np.ndarray)):
					lat_diff = abs(float(mav_lat_sanity) - float(drone_coords_mav[3]))
					lon_diff = abs(float(mav_lon_sanity) - float(drone_coords_mav[4]))
					lastalt_mav = int(drone_coords_mav[5])
					if((lat_diff <= 0.01 and lon_diff <= 0.01) or skipfirstmav == 1): #About 1-1.5km distance change from last coordinate
						mav_lat_sanity = float(drone_coords_mav[3])
						mav_lon_sanity = float(drone_coords_mav[4])
						print(mav_lat_sanity, mav_lon_sanity)
						if(skipfirstmav == 1):
							skipfirstmav +=1
					else: #If difference is too big too many times sanity stops the tracking to failsafe tracking
						sanitycount+=1
				else: #If mavlink doesnt recieve messages it adds to the mavlink sanity counter
					mav_sanitycount+=1
			'''
			If both methods for obtaining messages is used the coordinates are averaged
			If both methods are used but only one is working it continiues as normal with the working method
			'''
			if(osd_for_gps and mavlink_for_gps):
				if(isinstance(drone_coords_mav, np.ndarray) and drone_coords_osd != [False, False, False, False]): #If both methods are working
					drone_coords = [float((osd_lat_sanity + mav_lat_sanity)/2),float((osd_lon_sanity + mav_lon_sanity)/2), int((lastalt_osd + lastalt_mav)/2)]
				elif(isinstance(drone_coords_mav, bool) and drone_coords_osd != [False, False, False, False]): #If only OSD is working
					drone_coords = [float(osd_lat_sanity),float(osd_lon_sanity),int(lastalt_osd)]
				elif(isinstance(drone_coords_mav, np.ndarray) and drone_coords_osd == [False, False, False, False]): #If only mavlink is working
					drone_coords = [float(mav_lat_sanity),float(mav_lon_sanity),int(lastalt_mav)]
				else:
					drone_coords = dronecoords_save #If something goes wrong sets current coordinates as last safe coordinates
				dronecoords_save = drone_coords #If all goes well then drone coords are saved as last good coordinates
			
			elif(osd_for_gps and not mavlink_for_gps): #If only OSD is used
				drone_coords = [float(osd_lat_sanity),float(osd_lon_sanity),int(lastalt_osd)]
				dronecoords_save = drone_coords
			elif(mavlink_for_gps and not osd_for_gps): #If only mavlink is used
				drone_coords = [float(mav_lat_sanity),float(mav_lon_sanity),int(lastalt_mav)]
				print(drone_coords, dronecoords_save)
				dronecoords_save = drone_coords
			else:
				drone_coords = dronecoords_save #If coordinates are faulty sets last good coordinates
			#Rounds each coordinate to the same level of precision
			dronecoords_save[0] = float((int(dronecoords_save[0]*1000000000))/1000000000) #Latitude
			dronecoords_save[1] = float((int(dronecoords_save[1]*1000000000))/1000000000) #Longitude
			dronecoords_save[2] = int(dronecoords_save[2]) #Altitude
			#GPS calculation function
			direct_distance, newheading_from_home, new_angle = gps_calculation.alternate_calc_gps_distance(gpshome[0], gpshome[1], dronecoords_save[0], dronecoords_save[1], heading, angle, dronecoords_save[2])
			#Updates screen information with the calculations
			distancefromhome.config(text="Distance from home - " + str(int(direct_distance)) + " New heading - "+ str(int(newheading_from_home)) + " New angle - " + str(int(new_angle)))
			dronecoord.config(text="Drone coordinates - " + str(dronecoords_save))
			workWindow.after(50) # 50ms sleep 
			#Sets angle and heading servo to the calculated values
			angle = servo_change.anglechangeFn(angle, new_angle, initialize_data.accelerometer_bool)
			heading = servo_change.headingchangeFn(heading, newheading_from_home, initialize_data.accelerometer_bool, gpshome[3])
			#If a sanity variable is too large it enters a failsafe tracking mode
			if(sanitycount >=1000 or ((osd_sanitycount >=1000 or not osd_for_gps) and (mav_sanitycount >= 1000 or not mavlink_for_gps))):
				loop_running = False
				StartFailsafeTracking(heading, angle)
			time.sleep(0.2) #Sleep 200ms between each loop
			
	else: #Stop statement if home coordinates arent of expected list of 4 element lenght
		home_coords.config(text=("Home coordinates - Invalid"))
		loop_running = False

#Starts a failsafe tracking loop that rotates the antenna in a large area in front of the tracker
def StartFailsafeTracking(heading, angle):
	global loop_running_failsafe
	loop_running_failsafe = True
	serial_com.init_pico
	time.sleep(0.5)
	threading.Thread(target=FailsafeTracking, args=(heading, angle)).start()

#Stops the failsafe tracking loop
def StopTrackingFailsafe():
	global loop_running_failsafe
	loop_running_failsafe = False


def FailsafeTracking(lastheading, lastangle):
	global loop_running_failsafe
	global gpshome
	homeheading = gpshome[3] #Home heading is obtained from home coordinates
	while loop_running_failsafe:
		workWindow.after(850) #wait 850ms between each movement
		servo_change.headingChangeFailsafe(lastheading+20, homeheading) #Turns the antenna 20 degrees to the right from last heading
		servo_change.angleChangeFailsafe(lastangle)
		workWindow.after(850)
		servo_change.headingChangeFailsafe(lastheading, homeheading) #Turns the antenna back to last heading
		workWindow.after(850)
		servo_change.headingChangeFailsafe(lastheading-20, homeheading) #Turns the antenna 20 degrees to the left from last heading
		workWindow.after(850)
		servo_change.headingChangeFailsafe(lastheading, homeheading) #Turns the antenna back to last heading
		
#Stops all loops and kills the RP Pico program
def HaltTracker():
	global loop_running_failsafe, loop_running
	loop_running_failsafe, loop_running = False, False
	serial_com.send_cmd('exit', 0.01)

#Window with all tracking functionality
def workingWindow():
	global workWindow
	workWindow = tk.Toplevel(mainwindow)
	workWindow.geometry(geometry_res)
	workWindow.title("Tracking")
	testing_win= tk.Button(workWindow, text="Test OSD coordinates and Mavlink processed feed", command=testingWindow) #Goes to the optional pre tracking testing window 
	testing_win.grid(row=11,column=0, pady=50, padx=30)
	global home_coords, distancefromhome,dronecoord 
	home_coords = tk.Label(workWindow)
	if(gpshome): #home coordinates depending on setup
		if(comp_setup == 'PC'):
			home_coords.config(text=("Home coordinates(Lat, Lon, Alt, Heading) - " + str(gpshome)))
		else:
			home_coords.config(text=("Home coords - " + str(gpshome)))
	else:
		home_coords.config(text=("Home coordinates - Null"))
	#Datafields for incoming information and control buttons
	home_coords.grid(row=2,column=0, columnspan=3, pady=5)
	distancefromhome = tk.Label(workWindow)
	distancefromhome.grid(row=3,column=0, columnspan=3, pady=5)
	dronecoord = tk.Label(workWindow)
	dronecoord.grid(row=4,column=0, columnspan=3, pady=5)
	Start_btt= tk.Button(workWindow, command=StartTracking)
	Start_btt.grid(row=8,column=0, pady=5)
	Stop_btt= tk.Button(workWindow, command=StopTracking)
	Stop_btt.grid(row=9,column=0, pady=5)
	Failsafe_Label = tk.Label(workWindow, )
	Failsafe_Label.grid(row=8,column=2, columnspan= 2, pady=5)
	Failsafe_btt= tk.Button(workWindow, command=lambda: StartFailsafeTracking(gpshome[3], 40))
	Failsafe_btt.grid(row=9,column=2, pady=5)
	Failsafestop_btt= tk.Button(workWindow, command=StopTrackingFailsafe)
	Failsafestop_btt.grid(row=10,column=2, pady=5)
	Halt_btt= tk.Button(workWindow, command=HaltTracker, bd=8, relief=tk.RAISED, font=(font.nametofont("TkDefaultFont"), 25))
	Halt_btt.grid(row=11,column=2, pady=5)
	Return_btt= tk.Button(workWindow, text="Return", command=lambda: ReturnBttFn(workWindow))
	Return_btt.grid(row=15,column=0, pady=50)
	#System specific UI changes
	if(comp_setup == 'PC'):
		Start_btt.config(text="Start antenna tracking")
		Stop_btt.config(text="Stop antenna tracking")
		Failsafe_Label.config(text="Attempt to rotate antenna in large area in front")
		Failsafe_btt.config(text="Failsafe tracking")
		Failsafestop_btt.config(text="Failsafe tracking stop")
		Halt_btt.config(text="Halt tracker")
	else:
		#Label changes
		Start_btt.config(text="Start", bd=4, relief=tk.RAISED, font=(font.nametofont("TkDefaultFont"), 20))
		Stop_btt.config(text="Stop", bd=4, relief=tk.RAISED, font=(font.nametofont("TkDefaultFont"), 20))
		Failsafe_Label.config(text="Failsafe")
		Failsafe_btt.config(text="StartF", bd=2, relief=tk.RAISED)
		Failsafestop_btt.config(text="StopF", bd=2, relief=tk.RAISED)
		Halt_btt.config(text="Halt", bd=2, relief=tk.RAISED)
		testing_win.config(text="Test OSD & Mavlink",bd=2, relief=tk.RAISED)
		distancefromhome.config(font=(font.nametofont("TkDefaultFont"), 10))
		dronecoord.config(font=(font.nametofont("TkDefaultFont"), 10))
		home_coords.config(font=(font.nametofont("TkDefaultFont"), 10))
		#Layout changes
		home_coords.grid(row=1,column=0, columnspan=2, pady=2, padx=35)
		distancefromhome.grid(row=2,column=0,columnspan=2, pady=2, padx=5)
		dronecoord.grid(row=3,column=0,columnspan=2, pady=2, padx=35)
		Start_btt.grid(row=4,column=0, pady=2, padx=35)
		Stop_btt.grid(row=5,column=0, pady=2, padx=35)
		Failsafe_Label.grid(row=4,column=1, pady=2)
		Failsafe_btt.grid(row=4,column=1, pady=2)
		Failsafestop_btt.grid(row=5,column=1, pady=3)
		Halt_btt.grid(row=6,column=0, pady=3, padx=35)
		Return_btt.grid(row=6,column=1, pady=2)
		Return_btt.config(bd=2, relief=tk.RAISED)
		testing_win.grid(row=0,column=0, columnspan=2, pady=10, padx=90)
		
#tkinter main starting
if __name__ == "__main__":
    main()
	