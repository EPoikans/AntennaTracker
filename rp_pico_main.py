"""
from machine import Pin, I2C, UART, PWM
import sys
from time import sleep
import select
import utime
import ustruct
import math

compass_reg = { #QMC5883L
    "MAGNETO_ADDRESS": 0xD,
    "MAGNETO_ID": 0xFF,
    "MAGNETO_ID_ADD": 0xD,
    "CONTROLREG1": 0x09,
    "CONTROLREG2": 0x0A,
    "DATAREG": 0x00,
    "DATACONV": 100.0 / 3000.0 #+/-2Gauss field range - 12000 LSb/Gauss
}

def initialize_pico(accelerometer = False):  
    global state, vertservo, horizonservo, i2c, uart_gps
    i2c = I2C(1, scl=Pin(19), sda=Pin(18), freq=400000)
    uart_gps = UART(0, baudrate=115200, tx=Pin(16), rx=Pin(17))
    
    if compass_reg["MAGNETO_ADDRESS"] in i2c.scan():
        i2c.writeto_mem(compass_reg["MAGNETO_ADDRESS"], compass_reg['CONTROLREG1'], bytes([0x1D]))
    vertservo = PWM(Pin(5))
    horizonservo = PWM(Pin(6))
    vertservo.freq(50)
    horizonservo.freq(50)
    vertservo.duty_u16(2000) #Vertical angle servo 2000-0deg, 5400 -90deg
    horizonservo.duty_u16(5000)
    global vertservo, horizonservo, state
    state = True
    return state, vertservo, horizonservo

def readMagnetometer():
    global i2c
    try:
        data=i2c.readfrom_mem(compass_reg["MAGNETO_ADDRESS"], compass_reg["DATAREG"], 6)
        x = ((data[1] << 8) | data[0])
        y = ((data[3] << 8) | data[2])
        z = ((data[5] << 8) | data[4])
        if x > 32767:
            x -= 65536
        if y > 32767:
            y -= 65536
        if z > 32767:
            z -= 65536
        x=(x)*compass_reg["DATACONV"]
        x = x-136
        y=(y)*compass_reg["DATACONV"]
        y = y - 50
        z=(z)*compass_reg["DATACONV"]
        heading = math.atan2(y, x)
        heading = math.degrees(heading)
        day = 1
        declination_angle = math.sin(((-23.45)*math.pi/180) * math.cos(360/365 * (day + 10)))
        heading += declination_angle
        if heading < 0:
            heading += 360
        print("Magnetic field in X: %.2f uT, Y: %.2f uT, Z: %.2f uT, Heading: %.2f°" % (x, y, z, heading))
        return heading
    except:
        return ('err')

def setServoCycle (device, position):

    device.duty_u16(position)

    sleep(0.01)

def exec_cmd(command):
    if(len(command)==0):
        return
    cmdparts = command.split()
    global vertservo, horizonservo, state
    fnname = cmdparts[0]
    if(len(cmdparts) >=3 and cmdparts[0] == "setServoCycle"):
        servoname = cmdparts[1]
        pwmint = int(cmdparts[2])
    if fnname == "setServoCycle":
        if(servoname == "vert_servo"):
            setServoCycle(vertservo, pwmint)
            return("Set vert servo")
        elif(servoname == "horizon_servo"):
            setServoCycle(horizonservo, pwmint)
            return("Set horizontal servo")
    elif fnname == "initialize_pico":
        if(len(cmdparts)>=2 and cmdparts[1] == "accelerometer"):
            state, vertservo, horizonservo = initialize_pico(False) #True if use adxl345
        else:
            state, vertservo, horizonservo = initialize_pico()
        if(state):
            return("Initialized")
    elif fnname == "readMagnetometer":
        heading = readMagnetometer()
        return heading
    elif fnname == "pollGPS":
        lat, lon = pollGPS()
        return lat, lon
    elif fnname == "checkGPSSat":
        sat_count = checkGPSSat()
        return sat_count

"""
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
"""

def pollGPS():
    global uart_gps
    lat, lon, lat_deg, lat_min, lon_deg, lon_min = 0,0,0,0,0,0
    timeoutvar = 0
    while True:
        gps_data = uart_gps.readline()
        if(gps_data != 'None'):
            try:    
                gps_fields = gps_data.decode().split(',')
                if(gps_fields[0] == '$GNGLL' and gps_fields[6] == 'A'):
                    lat_deg = gps_fields[1][:2]
                    lat_min = gps_fields[1][2:]
                    lon_deg = gps_fields[3][:3]
                    lon_min = gps_fields[3][3:]
                    lat = round((float(lat_deg) + (float(lat_min)/60)), 7)
                    lon = round((float(lon_deg) + (float(lon_min)/60)), 7)
                    return lat, lon
                elif(gps_fields[0] == '$GNGGA' and int(gps_fields[7])>=6):
                    lat_deg = gps_fields[2][:2]
                    lat_min = gps_fields[2][2:]
                    lon_deg = gps_fields[4][:3]
                    lon_min = gps_fields[4][3:]
                    lat = round((float(lat_deg) + (float(lat_min)/60)), 7)
                    lon = round((float(lon_deg) + (float(lon_min)/60)), 7)
                    return lat, lon
            except:
                timeoutvar +=1
                if(timeoutvar >= 50000):
                    break
                else:
                    pass
                
            
def checkGPSSat():
    global uart_gps
    sat_count = 0
    timeoutvar = 0
    while True:
        gps_data = uart_gps.readline()
        if(gps_data != 'None'):
            try:
                gps_fields = gps_data.decode().split(',')
                if(gps_fields[0] == '$GNGSA' and gps_fields[1] == 'A' and int(gps_fields[2]) >= 3):
                    i = 1
                    while True:
                        if(gps_fields[2+i] != ''):
                            sat_count += 1
                        else:
                            break
                        i+=1
                    return sat_count
            except:
                timeoutvar +=1
                if(timeoutvar >= 50000):
                    break
                else:
                    pass
    return sat_count

poll_obj = select.poll()
poll_obj.register(sys.stdin, 1)
sleep(4)
initialize_pico()
sleep(2)
while True:
    global state, vertservo, horizonservo
    if poll_obj.poll(20):     
        reciev = sys.stdin.readline().strip()
        if(not reciev):
            continue
        else:
            if(reciev == 'exit'):
                sys.exit()
            res = exec_cmd(reciev)
            if res is not None:
                print(str(res))
            else:
                print('No response')



"""