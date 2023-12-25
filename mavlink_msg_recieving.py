from pymavlink import mavutil
import numpy as np
import tkinter as tk
import ui_window

def get_gps_mavlink(the_connection):
    vfr = False
    gps_raw = False
    while True:
        try:
            msg = the_connection.recv_match().to_dict()
            if(msg['mavpackettype']=='GPS_RAW_INT'):
                fix_type = msg['fix_type']
                sat_count = msg['satellites_visible']
                gps_raw = True
            if(msg['mavpackettype'] == 'VFR_HUD'):
                heading = msg['heading']
                vfr = True            
            if(msg['mavpackettype'] == 'GLOBAL_POSITION_INT' ):
                temp_gps_data = np.array([int(msg['time_boot_ms']/100), heading, int(msg['hdg']/100), msg['lat']/(10000000), msg['lon']/(10000000),int(msg['relative_alt']/1000), int(fix_type), int(sat_count)])
                if(vfr and gps_raw):
                    return_gps = temp_gps_data.copy()
                    return return_gps
        except:
            pass

def test_mavlink_connection(the_connection):
    i=0
    while True:
        try:
            msg = the_connection.recv_match().to_dict()
            return msg
        except:
            i+=1
            pass
        if(i==1000):
            return "No messages"

def get_gps_logs(the_connection,mavlink_sample1,mavlink_sample2,mavlink_sample3,mavlink_sample4, sampleMavlinkWindow): #Prints all gps related msg from flight log file
    i=0
    savei = 0
    j=0
    sampleMavlinkWindow.update_idletasks()
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
                if(j==0):
                    mavlink_sample1.config(text="")
                    sampleMavlinkWindow.update_idletasks()
                    copyarray = temp_gps_data.copy()
                    mavlink_sample1.config(text=("Alivetime - " + str(int(copyarray[0])) + " Compass HDG - " + str(int(copyarray[1])) + " GPS HDG - " +str(int(copyarray[2])) +" Lat - " + str(copyarray[3]) + " Lon - " +str(copyarray[4]) +" Rel Alt - " + str(int(copyarray[5])) +" Fix type - " + str(int(copyarray[6])) +" Sat - " + str(int(copyarray[7]))))
                    sampleMavlinkWindow.update_idletasks()
                    sampleMavlinkWindow.after(15)
                if(j==1):
                    mavlink_sample2.config(text="")
                    sampleMavlinkWindow.update_idletasks()
                    copyarray = temp_gps_data.copy()
                    mavlink_sample2.config(text=("Alivetime - " + str(int(copyarray[0])) + " Compass HDG - " + str(int(copyarray[1])) + " GPS HDG - " +str(int(copyarray[2])) +" Lat - " + str(copyarray[3]) + " Lon - " +str(copyarray[4]) +" Rel Alt - " + str(int(copyarray[5])) +" Fix type - " + str(int(copyarray[6])) +" Sat - " + str(int(copyarray[7]))))
                    sampleMavlinkWindow.update_idletasks()
                    sampleMavlinkWindow.after(15)

                if(j==2):
                    mavlink_sample3.config(text="")
                    sampleMavlinkWindow.update_idletasks()
                    copyarray = temp_gps_data.copy()
                    mavlink_sample3.config(text=("Alivetime - " + str(int(copyarray[0])) + " Compass HDG - " + str(int(copyarray[1])) + " GPS HDG - " +str(int(copyarray[2])) +" Lat - " + str(copyarray[3]) + " Lon - " +str(copyarray[4]) +" Rel Alt - " + str(int(copyarray[5])) +" Fix type - " + str(int(copyarray[6])) +" Sat - " + str(int(copyarray[7]))))
                    sampleMavlinkWindow.update_idletasks()
                    sampleMavlinkWindow.after(15)

                if(j==3):
                    mavlink_sample4.config(text="")
                    sampleMavlinkWindow.update_idletasks()
                    copyarray = temp_gps_data.copy()
                    mavlink_sample4.config(text=("Alivetime - " + str(int(copyarray[0])) + " Compass HDG - " + str(int(copyarray[1])) + " GPS HDG - " +str(int(copyarray[2])) +" Lat - " + str(copyarray[3]) + " Lon - " +str(copyarray[4]) +" Rel Alt - " + str(int(copyarray[5])) +" Fix type - " + str(int(copyarray[6])) +" Sat - " + str(int(copyarray[7]))))
                    sampleMavlinkWindow.update_idletasks()
                    sampleMavlinkWindow.after(15)
                    
                j+=1
                if(j>=4):
                    sampleMavlinkWindow.update_idletasks()
                    sampleMavlinkWindow.after(15)
                    j=0
                
                
                if((msg['time_boot_ms']/100)>=3000):
                    savei = i
        except:
            pass
        if(savei == i):
            mavlink_sample1.config(text="End of sample")
            mavlink_sample2.config(text="End of sample")
            mavlink_sample3.config(text="End of sample")
            mavlink_sample4.config(text="End of sample")
            break

def await_home_coords(the_connection): #Usable if no GPS & compass unit on ground station, uses the drone arming location and heading as home gps and home heading
    homegpsdata = []
    while True:
        try:
            msg = the_connection.recv_match().to_dict()
            #print(msg)
            if(msg['mavpackettype']=='GPS_RAW_INT'):
                fix_type = msg['fix_type']
                sat_count = msg['satellites_visible']
            if(msg['mavpackettype'] == 'VFR_HUD'):
                heading = msg['heading']            
            if(msg['mavpackettype'] == 'GLOBAL_POSITION_INT' ):
                temp_gps_data = np.array([int(msg['time_boot_ms']/100), heading, int(msg['hdg']/100), msg['lat']/(10000000), msg['lon']/(10000000),int(msg['relative_alt']/1000), int(fix_type), int(sat_count)])
            if((msg['mavpackettype']=='HOME_POSITION')):
                homegpsdata = temp_gps_data.copy()
                return homegpsdata           
        except:
            pass

#GPS data array features 1. time since boot 2. compass heading value 3. gps heading value 4. gps latitude 5. gps latitude 6. gps relative altitude 7. gps fix type 8. locked sattelite count 