import time
from tkinter import *
from tkinter import ttk
import tkinter as tk
import numpy as np
import cv2
import initialize_data
import gps_calculation
import img_processing
from collections.abc import Iterable
from PIL import Image, ImageTk



def main():
	global mainwindow
	mainwindow = tk.Tk()
	mainwindow.geometry("800x480")
	mainwindow.title("AntennaTracker")
	home_pos_choice_label = tk.Label(mainwindow, text="Select how to obtain home GPS position and compass heading")
	home_pos_choice_label.grid(row=0, column=0, padx=10, pady=2)
	global home_pos_choice_selected
	home_pos_choice_selected = tk.IntVar()
	homebutton = tk.Radiobutton(mainwindow, text="Mavlink home position", variable=home_pos_choice_selected, value=1)
	homebutton.grid(row=1, column=0, sticky="w", padx=8, pady=2)
	homebutton = tk.Radiobutton(mainwindow, text="OSD home position", variable=home_pos_choice_selected, value=2)
	homebutton.grid(row=2, column=0, sticky="w", padx=8, pady=2)
	homebutton = tk.Radiobutton(mainwindow, text="Ground station GPS module", variable=home_pos_choice_selected, value=3)
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
	global system_checkbox_var3
	system_checkbox_var3 = tk.IntVar()
	system_checkbox3 = tk.Checkbutton(mainwindow, text="Use accelerometer for vertical angle measurements", variable=system_checkbox_var3)
	system_checkbox3.grid(row=4, column=1, sticky="w", padx=10, pady=5)
	init_button = tk.Button(mainwindow, text="Initialize program", command=initialize)
	init_button.grid(row=5,column=0, pady=5)
	test_button = tk.Button(mainwindow, text="Show test sample video analysis", command=SampleVideo)
	test_button.grid(row=5,column=1, pady=5)
	global error_label
	error_label = tk.Label(mainwindow, text="")
	error_label.grid(row=6, column=0, pady=5)
	
	mainwindow.mainloop()

def initialize():
	home_gps_select = home_pos_choice_selected.get()
	osd_for_gps = system_checkbox_var1.get()
	mavlink_for_gps = system_checkbox_var2.get()
	accelerometer = system_checkbox_var3.get()
	if(home_gps_select):
		if(osd_for_gps or mavlink_for_gps):
			if(home_gps_select == 1):
				if(mavlink_for_gps):
					error_label.config(text="")
					initialize_data.initialize_data(bool(mavlink_for_gps), bool(osd_for_gps), 'mavlink', False, accelerometer)
				else:
					error_label.config(text="Mavlink for GPS position is needed to obtain home coordinates via Mavlink")
			if(home_gps_select == 2):
				if(osd_for_gps):
					error_label.config(text="")
					initialize_data.initialize_data(bool(mavlink_for_gps), bool(osd_for_gps), 'OSD', False, accelerometer)
					global gps_home_window
					gps_home_window = tk.Toplevel(mainwindow)
					gps_home_window.geometry("800x480")
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
	TestVideo(0)
	

def TestMavlink():
	print("")

def SampleVideo():
	print("")

def SampleMavlink():
	print("")

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
	testing_window.geometry("800x480")
	mavlink_test = tk.Button(testing_window, text="Test mavlink incoming messages", command=TestMavlink)
	mavlink_test.grid(row=1,column=0, pady=5)
	global osd_test_screenshot
	global osd_test_coords
	osd_test = tk.Button(testing_window, text="Test OSD coordinates and processed feed", command=TestVideoLoop)
	osd_test.grid(row=1,column=1, pady=5)
	
	osd_test_screenshot = tk.Label(testing_window)
	osd_test_screenshot.grid(row=3,column=1, pady=5)
	
	osd_test_coords = tk.Label(testing_window)
	osd_test_coords.grid(row=2,column=1, pady=5)
	mavlink_sample= tk.Button(testing_window, text="See mavlink sample", command=SampleMavlink)
	mavlink_sample.grid(row=10,column=0, pady=5)
	OSD_sample= tk.Button(testing_window, text="See OSD sample", command=SampleVideo)
	OSD_sample.grid(row=10,column=1, pady=5)
	Return_btt= tk.Button(testing_window, text="Return", command=ReturnBttFn)
	Return_btt.grid(row=15,column=0, pady=5)

def ReturnBttFn():
	testing_window.destroy()

def workingWindow():
	global workWindow
	workWindow = tk.Toplevel(mainwindow)
	workWindow.geometry("800x480")
	workWindow.title("Tracking")
	testing_win= tk.Button(workWindow, text="Test OSD coordinates and processed feed", command=testingWindow)
	testing_win.grid(row=10,column=0, pady=5)

if __name__ == "__main__":
    main()