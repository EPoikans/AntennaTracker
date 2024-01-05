import platform
import serial
import time

def findSerialPort(system):
    if(system == 'linux' or system == 'raspberrypi'):
        portstart = "/dev/ttyACM"
    elif(system == 'windows'):
        portstart = "COM"
    i=0
    for i in range(256):
        serial_port = (str(portstart) + str(i))
        try:
            with serial.Serial(serial_port, 115200, timeout=2) as ser:
                msg = ('PortCheck ' + '\n')
                ser.write(msg.encode())
                time.sleep(1)
                response = ser.readline().decode().strip()
                if(response == "RPPico"):
                    return serial_port
        except:
            pass
    return ("Port not found")

if platform.system().lower() == 'linux':
    try:
        import RPi.GPIO as GPIO
        serial_port = findSerialPort("raspberrypi")
        baud = 115200
    except ImportError:
        serial_port = findSerialPort("linux")
        baud = 115200
else:
    serial_port = findSerialPort("windows")
    print(serial_port)
    baud = 115200


def send_cmd(command, sleeptime):
    try:
        with serial.Serial(serial_port, baud, timeout=2) as ser:
            ser.write(command.encode())
            time.sleep(sleeptime)
            response = ser.readline().decode().strip()
            #print(f"Sent: {command}Received: {response}")
            return response
    except Exception as e:
        print(f"Error: {e}")
        return None

def init_pico(accel = False):
    send_cmd('initialize_pico '+ str(accel) + '\n')

def getGPS():
    res = send_cmd('pollGPS ' + '\n', 0.3)
    if(res != ''):
        res = res[1:]
        res = res[:-1]
        resarr = res.split(',')
        #print(resarr[0])
        #print(resarr[1])
        return str(float(resarr[0])), str(float(resarr[1]))
    else:
        return str(0), str(0)

def getSatCount():
    return send_cmd('checkGPSSat ' + '\n', 0.3)

def getMagnetometer():
    heading = send_cmd('readMagnetometer ' + '\n', 0.05)
    return int(float(heading))

def setVerticalServo(pwm_freq, pwm_current_estimate):
    if isinstance(pwm_freq, int) or isinstance(pwm_freq, float):
        if(pwm_freq > pwm_current_estimate):
            pwm_diff = pwm_freq - pwm_current_estimate
            i=1
            for i in range(int(pwm_diff/25)):
                if(int(pwm_current_estimate + (i * (pwm_diff/int(pwm_diff/25)))) >= int(pwm_freq)):
                    send_cmd('setServoCycle vert_servo '+ str(int(pwm_freq)) + '\n', 0.01)
                    break
                send_cmd('setServoCycle vert_servo '+ str(int(pwm_current_estimate + (i * (pwm_diff/int(pwm_diff/25))))) + '\n', 0.01)
        elif(pwm_freq < pwm_current_estimate):
            pwm_diff = pwm_current_estimate - pwm_freq
            i=1
            for i in range(int(pwm_diff/25)):
                if(int(pwm_current_estimate - (i * (pwm_diff/int(pwm_diff/25)))) >= int(pwm_freq)):
                    send_cmd('setServoCycle vert_servo '+ str(int(pwm_freq)) + '\n', 0.01)
                    break
                send_cmd('setServoCycle vert_servo '+ str(int(pwm_current_estimate - (i * (pwm_diff/int(pwm_diff/25))))) + '\n', 0.01)
        else:
            send_cmd('setServoCycle vert_servo '+ str(int(pwm_freq)) + '\n', 0.01)

def setHorizontalServo(pwm_freq, pwm_current_estimate):
    if isinstance(pwm_freq, int) or isinstance(pwm_freq, float):
        if(pwm_freq > pwm_current_estimate):
            pwm_diff = pwm_freq - pwm_current_estimate
            i=0
            for i in range(int(pwm_diff/15)):
                if(int(pwm_current_estimate + (i * (pwm_diff/int(pwm_diff/15)))) >= int(pwm_freq)):
                    send_cmd('setServoCycle horizon_servo '+ str(int(pwm_freq)) + '\n', 0.01)
                    break
                send_cmd('setServoCycle horizon_servo '+ str(int(pwm_current_estimate + (i * (pwm_diff/int(pwm_diff/15))))) + '\n', 0.01)
            
        elif(pwm_freq < pwm_current_estimate):
            pwm_diff = pwm_current_estimate - pwm_freq
            i=0
            for i in range(int(pwm_diff/15)):
                if(int(pwm_current_estimate - (i * (pwm_diff/int(pwm_diff/15)))) >= int(pwm_freq)):
                    send_cmd('setServoCycle horizon_servo '+ str(int(pwm_freq)) + '\n', 0.01)
                    break
                send_cmd('setServoCycle horizon_servo '+ str(int(pwm_current_estimate - (i * (pwm_diff/int(pwm_diff/15))))) + '\n', 0.01)
        else:
            send_cmd('setServoCycle horizon_servo '+ str(int(pwm_freq)) + '\n', 0.01)
    #print(pwm_freq, pwm_current_estimate)

def getAccelVal():
    return send_cmd('getADXL' + '\n', 0.05)

#send_cmd('setServoCycle horizon_servo 4000' + '\n')
#init_pico()
#print(getGPS()) 
#print(getMagnetometer())