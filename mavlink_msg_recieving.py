from pymavlink import mavutil
import numpy as np

connect_adress = '/dev/ttyUSB0'
testfile = './TestingFiles/2023-09-21 09-45-14.tlog'
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
while True:
    savei = i
    try:
        msg = the_connection.recv_match().to_dict()
        if(msg['mavpackettype'] == 'GLOBAL_POSITION_INT' ):
            temp_gps_data = np.array([msg['lat']/(10000000),msg['lon']/(10000000),int(msg['relative_alt']/1000)])
            print(temp_gps_data)
             
        i+=1
    except:
        pass
    if(savei == i and usetestfile == True):
        break
