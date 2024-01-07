from pymavlink import mavutil
import numpy as np
import tkinter as tk
import ui_window
import initialize_data

def get_gps_mavlink(the_connection): #Main function for getting gps data from the first incoming mavlink message of type GLOBAL_POSITION_INT, VFR_HUD and GPS_RAW_INT
    vfr = False
    gps_raw = False
    i=0
    while True:
        i+=1
        try:
            msg = the_connection.recv_match().to_dict()
            if(msg['mavpackettype']=='GPS_RAW_INT'): #Checks for message to obtain gps 3D fix type and locked sattelite count
                fix_type = msg['fix_type']
                sat_count = msg['satellites_visible']
                gps_raw = True
            if(msg['mavpackettype'] == 'VFR_HUD'): #Checks for message to obtain compass heading
                heading = msg['heading']
                vfr = True            
            if(msg['mavpackettype'] == 'GLOBAL_POSITION_INT' ): #Checks for message to obtain gps latitude, longitude and relative altitude
                temp_gps_data = np.array([int(msg['time_boot_ms']/100), heading, int(msg['hdg']/100), msg['lat']/(10000000), msg['lon']/(10000000),int(msg['relative_alt']/1000), int(fix_type), int(sat_count)])
                if(vfr and gps_raw): #If all messages have been recieved, returns array with extracted data from messages
                    return_gps = temp_gps_data.copy()
                    if initialize_data.debug: #Debug print if enabled
                        print(return_gps + "Mavlink GPS data")
                    return return_gps
        except:
            pass
        if(i>=10000): #Timeout
            return False

def test_mavlink_connection(the_connection): #Used in testing mavlink window, returns the first incoming message after function is called
    i=0
    while True:
        try:
            msg = the_connection.recv_match().to_dict() #Converts message to dictionary
            if initialize_data.debug: #Prints message to console if debug is enabled
                print(msg)
            return msg
        except:
            i+=1
            pass
        if(i==1000): #Timeout
            return "No messages"

def get_gps_logs(the_connection,mavlink_sample1,mavlink_sample2,mavlink_sample3,mavlink_sample4, sampleMavlinkWindow): #Prints all gps related msg from flight log file
    i=0
    savei = 0
    j=0
    sampleMavlinkWindow.update_idletasks() #Updates sample mavlink window to not freeze tkinter loop
    while True:
        savei = i
        try:
            msg = the_connection.recv_match().to_dict()
            if(msg):
                i+=1
            if(msg['mavpackettype']=='GPS_RAW_INT'):
                fix_type = msg['fix_type']
                sat_count = msg['satellites_visible']
            if(msg['mavpackettype'] == 'VFR_HUD'):
                heading = msg['heading']            
            if(msg['mavpackettype'] == 'GLOBAL_POSITION_INT' ):
                temp_gps_data = np.array([int(msg['time_boot_ms']/100), heading, int(msg['hdg']/100), msg['lat']/(10000000), msg['lon']/(10000000),int(msg['relative_alt']/1000), int(fix_type), int(sat_count)])
                if(j==0): #Updates sample mavlink window first textbox with gps data
                    mavlink_sample1.config(text="") #Clears first textbox
                    sampleMavlinkWindow.update_idletasks() #Updates sample mavlink window to not freeze tkinter loop
                    copyarray = temp_gps_data.copy() #Copies gps data array to not change the original
                    mavlink_sample1.config(text=("Alivetime - " + str(int(copyarray[0])) + " Compass HDG - " + str(int(copyarray[1])) + " GPS HDG - " +str(int(copyarray[2])) +" Lat - " + str(copyarray[3]) + " Lon - " +str(copyarray[4]) +" Rel Alt - " + str(int(copyarray[5])) +" Fix type - " + str(int(copyarray[6])) +" Sat - " + str(int(copyarray[7]))))
                    sampleMavlinkWindow.update_idletasks() #Refreshes sample window
                    sampleMavlinkWindow.after(10) #Waits 10ms before continuing

                if(j==1): #Updates sample mavlink window second textbox with gps data
                    mavlink_sample2.config(text="") #Clears second textbox
                    sampleMavlinkWindow.update_idletasks() #Updates sample mavlink window to not freeze tkinter loop
                    copyarray = temp_gps_data.copy() #Copies gps data array to not change the original
                    mavlink_sample2.config(text=("Alivetime - " + str(int(copyarray[0])) + " Compass HDG - " + str(int(copyarray[1])) + " GPS HDG - " +str(int(copyarray[2])) +" Lat - " + str(copyarray[3]) + " Lon - " +str(copyarray[4]) +" Rel Alt - " + str(int(copyarray[5])) +" Fix type - " + str(int(copyarray[6])) +" Sat - " + str(int(copyarray[7]))))
                    sampleMavlinkWindow.update_idletasks() #Refreshes sample window
                    sampleMavlinkWindow.after(10) #Waits 10ms before continuing

                if(j==2):   #Updates sample mavlink window third textbox with gps data
                    mavlink_sample3.config(text="") #Clears third textbox
                    sampleMavlinkWindow.update_idletasks() #Updates sample mavlink window to not freeze tkinter loop
                    copyarray = temp_gps_data.copy() #Copies gps data array to not change the original
                    mavlink_sample3.config(text=("Alivetime - " + str(int(copyarray[0])) + " Compass HDG - " + str(int(copyarray[1])) + " GPS HDG - " +str(int(copyarray[2])) +" Lat - " + str(copyarray[3]) + " Lon - " +str(copyarray[4]) +" Rel Alt - " + str(int(copyarray[5])) +" Fix type - " + str(int(copyarray[6])) +" Sat - " + str(int(copyarray[7]))))
                    sampleMavlinkWindow.update_idletasks() #Refreshes sample window
                    sampleMavlinkWindow.after(10) #Waits 10ms before continuing

                if(j==3):  #Updates sample mavlink window fourth textbox with gps data
                    mavlink_sample4.config(text="") #Clears fourth textbox
                    sampleMavlinkWindow.update_idletasks() #Updates sample mavlink window to not freeze tkinter loop
                    copyarray = temp_gps_data.copy() #Copies gps data array to not change the original
                    mavlink_sample4.config(text=("Alivetime - " + str(int(copyarray[0])) + " Compass HDG - " + str(int(copyarray[1])) + " GPS HDG - " +str(int(copyarray[2])) +" Lat - " + str(copyarray[3]) + " Lon - " +str(copyarray[4]) +" Rel Alt - " + str(int(copyarray[5])) +" Fix type - " + str(int(copyarray[6])) +" Sat - " + str(int(copyarray[7]))))
                    sampleMavlinkWindow.update_idletasks() #Refreshes sample window
                    sampleMavlinkWindow.after(10) #Waits 10ms before continuing
                    
                j+=1
                if(j>=4): #Resets the rolling textbox update loop back to the first textbox
                    sampleMavlinkWindow.update_idletasks() #Updates sample mavlink window to not freeze tkinter loop
                    sampleMavlinkWindow.after(10) #Waits 10ms before continuing
                    j=0
                
                
                if((msg['time_boot_ms']/100)>=3000): #Stops the sample after drone alivetime in a recieved message is over 3000ms
                    savei = i #Enables the end of sample if statement
        except:
            pass
        if(savei == i): #Displays end of sample in all 4 rolling textboxes
            mavlink_sample1.config(text="End of sample")
            mavlink_sample2.config(text="End of sample")
            mavlink_sample3.config(text="End of sample")
            mavlink_sample4.config(text="End of sample")
            break

def await_home_coords(the_connection): #Usable if no GPS & compass unit on ground station, uses the drone arming location and heading as home gps and home heading
    homegpsdata = []
    i=0
    fix_type, sat_count, heading = 0,0,0 #Default variables to not block returning of home gps data. 
    while True:
        i+=1
        try:
            msg = the_connection.recv_match().to_dict()
            if initialize_data.debug: #Prints message to console if debug is enabled
                print(msg)
            if(msg['mavpackettype']=='GPS_RAW_INT'):
                fix_type = msg['fix_type']
                sat_count = msg['satellites_visible']
            if(msg['mavpackettype'] == 'VFR_HUD'):
                heading = msg['heading']            
            if(msg['mavpackettype'] == 'GLOBAL_POSITION_INT' ):

                temp_gps_data = np.array([int(msg['time_boot_ms']/100), heading, int(msg['hdg']/100), msg['lat']/(10000000), msg['lon']/(10000000),int(msg['relative_alt']/1000), int(fix_type), int(sat_count)])
            if((msg['mavpackettype']=='HOME_POSITION') and isinstance(temp_gps_data, np.ndarray)): #Awaits mavlink home position set message
                homegpsdata = temp_gps_data.copy() #Copies data array
                return homegpsdata #Returns last recorded coordinates before home position set message          
        except:
            pass
        if(i>=500000): #Timeout
            return "Timeout"

#GPS data array features 1. time since boot 2. compass heading value 3. gps heading value 4. gps latitude 5. gps latitude 6. gps relative altitude 7. gps fix type 8. locked sattelite count 