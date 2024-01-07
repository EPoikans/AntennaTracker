 
#Main script for the Raspberry Pi Pico. Does nothing on its own. 
#Communication over USB from serial_com.py file

from machine import Pin, I2C, UART, PWM
import sys
from time import sleep
import select
import utime
import ustruct
import math

compass_reg = { #QMC5883L magnetometer - https://datasheet.lcsc.com/lcsc/QST-QMC5883L-TR_C192585.pdf
    "MAGNETO_ADDRESS": 0xD,
    "MAGNETO_ID": 0xFF,
    "MAGNETO_ID_ADD": 0xD,
    "CONTROLREG1": 0x09,
    "CONTROLREG2": 0x0A,
    "DATAREG": 0x00,
    "DATACONV": 100.0 / 3000.0 #+/-8Gauss field range - 3000 LSb/Gauss -> 100LSb/uT
}

def initialize_pico(accelerometer = False):  
    global state, vertservo, horizonservo, i2c, uart_gps
    i2c = I2C(1, scl=Pin(19), sda=Pin(18), freq=400000) #I2C for magnetometer
    uart_gps = UART(0, baudrate=115200, tx=Pin(16), rx=Pin(17)) #UART for GPS
    
    if compass_reg["MAGNETO_ADDRESS"] in i2c.scan(): #Checks if magnetometer is connected
        i2c.writeto_mem(compass_reg["MAGNETO_ADDRESS"], compass_reg['CONTROLREG1'], bytes([0x1D]))
    vertservo = PWM(Pin(5)) #Vertical angle servo PWM pin
    horizonservo = PWM(Pin(6)) #Horizontal angle servo PWM pin
    vertservo.freq(50) #PWM frequency
    horizonservo.freq(50) #PWM frequency
    #Default servo states - 0deg vertical and horizontal with +/-115deg range
    vertservo.duty_u16(2000) #Vertical angle servo 2000-0deg, 5400 -90deg
    horizonservo.duty_u16(4900) # 4900 is center with roughly 115deg in each direction
    global vertservo, horizonservo, state
    state = True
    return state, vertservo, horizonservo

def readMagnetometer():
    global i2c
    try:
        data=i2c.readfrom_mem(compass_reg["MAGNETO_ADDRESS"], compass_reg["DATAREG"], 6)
        x = ((data[1] << 8) | data[0]) #Combines LSB and MSB in the correct order
        y = ((data[3] << 8) | data[2])
        z = ((data[5] << 8) | data[4])
        if x > 32767:
            x -= 65536
        if y > 32767:
            y -= 65536
        if z > 32767:
            z -= 65536
        x=(x)*compass_reg["DATACONV"] #Converts to uT
        #X axis offset due to servo motor magnetic field. Might need to be adjusted depending on location and servo
        x = x-136 
        y=(y)*compass_reg["DATACONV"] #Converts to uT
        #Y axis offset due to servo motor magnetic field. Might need to be adjusted depending on location and servo
        y = y - 50
        z=(z)*compass_reg["DATACONV"] #Converts to uT
        heading = math.atan2(y, x) #Calculates heading from magnetometer data
        heading = math.degrees(heading)
        day = 6
        declination_angle = math.sin(((-23.45)*math.pi/180) * math.cos(360/365 * (day + 10)))
        heading += declination_angle
        if heading < 0:
            heading += 360
        #print("Magnetic field in X: %.2f uT, Y: %.2f uT, Z: %.2f uT, Heading: %.2f°" % (x, y, z, heading)) #Debug print statement
        return str(heading)
    except:
        return ('err')

def setServoCycle (device, position): #Sets servo PWM cycle

    device.duty_u16(position)

    sleep(0.01)

def exec_cmd(command): #Processes incoming messages over USB to apropriate functions
    if(len(command)==0):
        return
    cmdparts = command.split() #Splits message into parts
    global vertservo, horizonservo, state
    fnname = cmdparts[0] #First part of message is the function name
    if(len(cmdparts) >=3 and cmdparts[0] == "setServoCycle"): #Checks if servo PWM cycle is being set
        servoname = cmdparts[1]
        pwmint = int(cmdparts[2])
    if fnname == "setServoCycle": #Sets servo PWM cycle
        if(servoname == "vert_servo"):
            setServoCycle(vertservo, pwmint)
            return("Set vert servo") #Returns a message to confirm the servo has been set
        elif(servoname == "horizon_servo"):
            setServoCycle(horizonservo, pwmint)
            return("Set horizontal servo") #Returns a message to confirm the servo has been set
    elif fnname == "initialize_pico": #Initializes the pico
        if(len(cmdparts)>=2 and cmdparts[1] == "accelerometer"):
            state, vertservo, horizonservo = initialize_pico(False) #True if use adxl345 - retired currently
        else:
            state, vertservo, horizonservo = initialize_pico()
        if(state):
            return("Initialized") #Returns a message to confirm the pico has been initialized
    elif fnname == "readMagnetometer": #Reads magnetometer data
        heading = readMagnetometer()
        return heading
    elif fnname == "pollGPS": #Requests GPS data from GPS module
        try:
            lat, lon = pollGPS()
            return lat, lon
        except:
            return 0, 0
    elif fnname == "checkGPSSat": #Requests active satellite count from GPS module
        sat_count = checkGPSSat()
        return sat_count
    elif fnname == "PortCheck": #PortCheck is sent to every possible serial port to find the correct one which the pico is connected to
        portconfirm = "RPPico" #Returns a message to confirm the pico has been found
        return str(portconfirm)

#GPS module chip - M10 using NMEA 0183 messages
"""
NMEA 0183 messages

Talker IDs -
BD or GB - Beidou
GA - Galileo
GP - GPS
GL - GLONASS
GN - Combined talkers

$(Talker ID) + GGA  - Global Positioning System Fixed Data
$(Talker ID) + GLL - Geographic Position-- Latitude and Longitude 
$(Talker ID) + GSA - GNSS Dilution of Precision and active satellites 
$(Talker ID) + GSV - GNSS satellites in view 
$(Talker ID) + RMC - Recommended minimum specific GPS data 
$(Talker ID) + VTG - Course over ground and ground speed 

<CR><LF> or \r\n - end of message
$ - begining of message

$GNGSA - active sattelite count
$GNGLL - lat and lon 
"""


def pollGPS():
    global uart_gps
    lat, lon, lat_deg, lat_min, lon_deg, lon_min = 0,0,0,0,0,0
    timeoutvar = 0
    while True:
        gps_data = uart_gps.readline() #Reads GPS data from UART
        if(gps_data != 'None'): #Checks if data is None or a NMEA message
            try:    
                gps_fields = gps_data.decode().split(',') #NMEA messages are split with ,
                if(gps_fields[0] == '$GNGLL' and gps_fields[6] == 'A'): #Latitude and Longitude with active satellites
                    lat_deg = gps_fields[1][:2]
                    lat_min = gps_fields[1][2:]
                    lon_deg = gps_fields[3][:3]
                    lon_min = gps_fields[3][3:]
                    lat = round((float(lat_deg) + (float(lat_min)/60)), 7) #Converts latitude and longitude from minutes to decimal
                    lon = round((float(lon_deg) + (float(lon_min)/60)), 7)
                    return lat, lon
                elif(gps_fields[0] == '$GNGGA' and int(gps_fields[7])>=6): #Global Positioning System Fix Data for all gps systems with 6 or more active satellites
                    lat_deg = gps_fields[2][:2]
                    lat_min = gps_fields[2][2:]
                    lon_deg = gps_fields[4][:3]
                    lon_min = gps_fields[4][3:]
                    lat = round((float(lat_deg) + (float(lat_min)/60)), 7) #Converts latitude and longitude from minutes to decimal
                    lon = round((float(lon_deg) + (float(lon_min)/60)), 7)
                    return lat, lon
            except:
                timeoutvar +=1
                if(timeoutvar >= 10000): #Timeout
                    break
                else:
                    pass
                
            
def checkGPSSat():
    global uart_gps
    sat_count = 0
    timeoutvar = 0
    while True:
        gps_data = uart_gps.readline() #Reads GPS data from UART
        if(gps_data != 'None'): #Checks if data is None or a NMEA message
            try:
                gps_fields = gps_data.decode().split(',') #NMEA messages are split with ,
                if(gps_fields[0] == '$GNGSA' and gps_fields[1] == 'A' and int(gps_fields[2]) >= 3): #Global Positioning System Dilution of Precision and active satellites
                    i = 1
                    while True:
                        if(gps_fields[2+i] != ''): #Checks each field for satelite number and adds to sattelite count. The message contains from the 3rd field onwards active satellite number
                            sat_count += 1
                        else:
                            break
                        i+=1
                    return sat_count
            except:
                timeoutvar +=1 #Timeout
                if(timeoutvar >= 50000):
                    break
                else:
                    pass
    return sat_count #Returns active satellite count or 0 if no satellites are found

poll_obj = select.poll()
poll_obj.register(sys.stdin, 1)
sleep(4) #Waits 4 seconds before centering servos after plugging in power to the pico
initialize_pico() #Calls initialize_pico function that initializes I2C, UART and PWM
sleep(2) #Waits 2 seconds before listening for serial incoming messages
while True:
    global state, vertservo, horizonservo
    if poll_obj.poll(20):     
        reciev = sys.stdin.readline().strip() #Reads incoming message
        if(not reciev):
            continue
        else:
            if(reciev == 'exit'):
                sys.exit()
            res = exec_cmd(reciev) #Calls exec_cmd function to process incoming message
            if res is not None:
                print(str(res)) #Print statements are picked up as response by serial_com.py
            else:
                print('No response') 

