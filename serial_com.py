import platform
import serial
import time

debug = True

def findSerialPort(system):
    if(system == 'linux' or system == 'raspberrypi'):
        portstart = "/dev/ttyACM"
    elif(system == 'windows'):
        portstart = "COM"
    i=0
    for i in range(10):
        serial_port = (str(portstart) + str(i))
        try:
            with serial.Serial(serial_port, 115200, timeout=2) as ser:
                msg = ('PortCheck ' + '\n')
                ser.write(msg.encode())
                time.sleep(1)
                response = ser.readline().decode().strip()
                if debug:
                    print(msg, response)
                if(response == "RPPico"):
                    return serial_port
        except:
            pass
    return ("Port not found")

global serial_port, system
if platform.system().lower() == 'linux':
    try:
        import RPi.GPIO as GPIO
        system = "raspberrypi"
        serial_port = "/dev/ttyACM0" #findSerialPort("raspberrypi")
        baud = 115200
    except ImportError:
        serial_port = findSerialPort("linux")
        system = "linux"
        baud = 115200
else:
    serial_port = findSerialPort("windows")
    system = "windows"
    if debug:
        print(serial_port)
    baud = 115200

def retrySerial():
    global serial_port, system
    serial_port = findSerialPort(system)
    if debug:
        print(serial_port)


def send_cmd(command, sleeptime):
    global serial_port
    try:
        with serial.Serial(serial_port, baud, timeout=2) as ser:
            ser.write(command.encode())
            time.sleep(sleeptime)
            response = ser.readline().decode().strip()
            if debug:
                print(f"Sent: {command}Received: {response}")
            return response
    except Exception as e:
        if debug:
            print(f"Error: {e}")
        return None

def init_pico(accel = False):
    send_cmd('initialize_pico '+ str(accel) + '\n', 0.02)

def getGPS():
    res = send_cmd('pollGPS ' + '\n', 1)
    try:
        if(res != ''):
            res = res[1:]
            res = res[:-1]
            resarr = res.split(',')
            if debug:
                print(resarr[0])
                print(resarr[1])
            return str(float(resarr[0])), str(float(resarr[1]))
        else:
            raise Exception
    except:
        return str(0), str(0)

def getSatCount():
    return send_cmd('checkGPSSat ' + '\n', 0.3)

def getMagnetometer():
    heading = send_cmd('readMagnetometer ' + '\n', 0.1)
    try:
        if(heading != ''):
            if debug:
                print(heading)
            return int(float(heading))
        else:
            raise Exception
    except:
        return False

def setVerticalServo(pwm_freq, pwm_current_estimate):
    if isinstance(pwm_freq, int) or isinstance(pwm_freq, float):
        if(pwm_freq > pwm_current_estimate):
            pwm_diff = pwm_freq - int(pwm_current_estimate)
            i=1
            for i in range(int(pwm_diff/25)):
                if(int(pwm_current_estimate + (i * (pwm_diff/int(pwm_diff/25)))) >= int(pwm_freq)):
                    send_cmd('setServoCycle vert_servo '+ str(int(pwm_freq)) + '\n', 0.01)
                    break
                send_cmd('setServoCycle vert_servo '+ str(int(pwm_current_estimate + (i * (pwm_diff/int(pwm_diff/25))))) + '\n', 0.01)
        elif(pwm_freq < pwm_current_estimate):
            pwm_diff = int(pwm_current_estimate) - pwm_freq
            i=1
            for i in range(int(pwm_diff/25)):
                if(int(pwm_current_estimate - (i * (pwm_diff/int(pwm_diff/25)))) >= int(pwm_freq)):
                    send_cmd('setServoCycle vert_servo '+ str(int(pwm_freq)) + '\n', 0.01)
                    break
                send_cmd('setServoCycle vert_servo '+ str(int(pwm_current_estimate - (i * (pwm_diff/int(pwm_diff/25))))) + '\n', 0.01)
        else:
            send_cmd('setServoCycle vert_servo '+ str(int(pwm_freq)) + '\n', 0.01)

def setHorizontalServo(pwm_freq, pwm_current_estimate):
    pwm_current_estimate = int(pwm_current_estimate)
    if debug:
        print(str(pwm_freq), str(pwm_current_estimate) + "sethorizontalServo freq, current estimate")
    if isinstance(pwm_freq, int) or isinstance(pwm_freq, float):
        if(pwm_freq > pwm_current_estimate):
            pwm_diff = pwm_freq - pwm_current_estimate
            i=0
            for i in range(int(pwm_diff/50)):
                if(int(pwm_current_estimate + (i * (pwm_diff/int(pwm_diff/50)))) >= int(pwm_freq)):
                    send_cmd('setServoCycle horizon_servo '+ str(int(pwm_freq)) + '\n', 0.02)
                    break
                send_cmd('setServoCycle horizon_servo '+ str(int(pwm_current_estimate + (i * (pwm_diff/int(pwm_diff/50))))) + '\n', 0.02)
            send_cmd('setServoCycle horizon_servo '+ str(int(pwm_freq)) + '\n', 0.02)
        elif(pwm_freq < pwm_current_estimate):
            pwm_diff = pwm_current_estimate - pwm_freq
            i=0
            for i in range(int(pwm_diff/50)):
                if(int(pwm_current_estimate - (i * (pwm_diff/int(pwm_diff/50)))) <= int(pwm_freq)):
                    send_cmd('setServoCycle horizon_servo '+ str(int(pwm_freq)) + '\n', 0.02)
                    break
                send_cmd('setServoCycle horizon_servo '+ str(int(pwm_current_estimate - (i * (pwm_diff/int(pwm_diff/50))))) + '\n', 0.02)
            send_cmd('setServoCycle horizon_servo '+ str(int(pwm_freq)) + '\n', 0.02)
        else:
            send_cmd('setServoCycle horizon_servo '+ str(int(pwm_freq)) + '\n', 0.02)

def getAccelVal():
    return send_cmd('getADXL' + '\n', 0.05)

