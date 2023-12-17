from pymavlink import mavutil
import numpy as np

connect_adress = '/dev/ttyUSB0'
testfile = './TestingFiles/2023-09-22 12-26-58.tlog'
usetestfile = True
if(usetestfile):
    usedaddress = testfile
else:
    usedaddress = connect_adress
the_connection = mavutil.mavlink_connection(usedaddress)
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
i=0
homegpsset = False
homegpsdata = []
while True:
    savei = i
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
        if((msg['mavpackettype']=='HOME_POSITION') and (homegpsset == False)):
            homegpsdata = temp_gps_data.copy()
            homegpsset = True            
        i+=1
    except:
        pass
    if(savei == i and usetestfile == True):
        break

#GPS data array features 1. time since boot 2. compass heading value 3. gps heading value 4. gps latitude 5. gps latitude 6. gps relative altitude 7. gps fix type 8. locked sattelite count 