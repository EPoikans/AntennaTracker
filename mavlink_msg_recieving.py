from pymavlink import mavutil
import numpy as np

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

def get_gps_logs(the_connection): #Prints all gps related msg from flight log file
    i=0
    savei = 0
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
                print(temp_gps_data)
        except:
            pass
        if(savei == i):
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