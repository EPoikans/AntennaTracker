import platform
import serial
import time


if platform.system().lower() == 'linux':
    try:
        import RPi.GPIO as GPIO
        serial_port = '/dev/ttyACM0'
        baud = 115200
    except ImportError:
        serial_port = 'COM10'
        baud = 115200
else:
    serial_port = 'COM10'
    baud = 115200



def send_cmd(command):
    try:
        with serial.Serial(serial_port, baud, timeout=2) as ser:
            ser.write(command.encode())
            time.sleep(0.05)
            response = ser.readline().decode().strip()
            #print(f"Sent: {command}Received: {response}")
            return response
    except Exception as e:
        print(f"Error: {e}")
        return None

def init_pico(accel = False):
    send_cmd('initialize_pico '+ str(accel) + '\n')

def getGPS():
    res = send_cmd('pollGPS ' + '\n')
    if(res != ''):
        res = res[1:]
        res = res[:-1]
        resarr = res.split(',')
        print(resarr[0])
        print(resarr[1])
        return str(float(resarr[0])), str(float(resarr[1]))
    else:
        return str(0), str(0)

def getSatCount():
    return send_cmd('checkGPSSat ' + '\n')

def getMagnetometer():
    heading = send_cmd('readMagnetometer ' + '\n')
    return int(float(heading))

def setVerticalServo(pwm_freq, pwm_current_estimate):
    if isinstance(pwm_freq, int) or isinstance(pwm_freq, float):
        if(pwm_freq > pwm_current_estimate):
            pwm_diff = pwm_freq - pwm_current_estimate
            i=1
            for i in range(int(pwm_diff/25)):
                if(int(pwm_current_estimate + (i * (pwm_diff/int(pwm_diff/25)))) >= int(pwm_freq)):
                    send_cmd('setServoCycle vert_servo '+ str(int(pwm_freq)) + '\n')
                    break
                send_cmd('setServoCycle vert_servo '+ str(int(pwm_current_estimate + (i * (pwm_diff/int(pwm_diff/25))))) + '\n')
        elif(pwm_freq < pwm_current_estimate):
            pwm_diff = pwm_current_estimate - pwm_freq
            i=1
            for i in range(int(pwm_diff/25)):
                if(int(pwm_current_estimate - (i * (pwm_diff/int(pwm_diff/25)))) >= int(pwm_freq)):
                    send_cmd('setServoCycle vert_servo '+ str(int(pwm_freq)) + '\n')
                    break
                send_cmd('setServoCycle vert_servo '+ str(int(pwm_current_estimate - (i * (pwm_diff/int(pwm_diff/25))))) + '\n')
        else:
            send_cmd('setServoCycle vert_servo '+ str(int(pwm_freq)) + '\n')

def setHorizontalServo(pwm_freq, pwm_current_estimate):
    if isinstance(pwm_freq, int) or isinstance(pwm_freq, float):
        if(pwm_freq > pwm_current_estimate):
            pwm_diff = pwm_freq - pwm_current_estimate
            i=0
            for i in range(int(pwm_diff/15)):
                if(int(pwm_current_estimate + (i * (pwm_diff/int(pwm_diff/15)))) >= int(pwm_freq)):
                    send_cmd('setServoCycle horizon_servo '+ str(int(pwm_freq)) + '\n')
                    break
                send_cmd('setServoCycle horizon_servo '+ str(int(pwm_current_estimate + (i * (pwm_diff/int(pwm_diff/15))))) + '\n')
            
        elif(pwm_freq < pwm_current_estimate):
            pwm_diff = pwm_current_estimate - pwm_freq
            i=0
            for i in range(int(pwm_diff/15)):
                if(int(pwm_current_estimate - (i * (pwm_diff/int(pwm_diff/15)))) >= int(pwm_freq)):
                    send_cmd('setServoCycle horizon_servo '+ str(int(pwm_freq)) + '\n')
                    break
                send_cmd('setServoCycle horizon_servo '+ str(int(pwm_current_estimate - (i * (pwm_diff/int(pwm_diff/15))))) + '\n')
        else:
            send_cmd('setServoCycle horizon_servo '+ str(int(pwm_freq)) + '\n')

def getAccelVal():
    return send_cmd('getADXL' + '\n')

#send_cmd('setServoCycle horizon_servo 4000' + '\n')
#init_pico()
#print(getGPS()) 
#print(getMagnetometer())