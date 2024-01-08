import platform
import serial
import time

debug = True #Debug variable for serial communication printing all responses from the pico

def findSerialPort(system): #Finds the correct serial port for the pico
    if(system == 'linux' or system == 'raspberrypi'): #Different serial port start for linux and windows
        portstart = "/dev/ttyACM"
    elif(system == 'windows'):
        portstart = "COM"
    i=0
    for i in range(256):
        serial_port = (str(portstart) + str(i)) #Combines the system start of a serial port with the number
        try:
            with serial.Serial(serial_port, 115200, timeout=2) as ser:
                msg = ('PortCheck ' + '\n') #Sends a message to the pico to check if it is the correct device
                ser.write(msg.encode())
                time.sleep(1)
                response = ser.readline().decode().strip() #Reads the response
                if debug:
                    print(msg, response)
                if(response == "RPPico"): #The pico should respond with RPPico
                    return serial_port
        except: #If the port is not the correct one it will throw an exception
            pass
    return ("Port not found") #If no port is found returns this

global serial_port, system
if platform.system().lower() == 'linux': #Finds the correct serial port depending on the system
    try:
        import RPi.GPIO as GPIO
        system = "raspberrypi"
        serial_port = findSerialPort("raspberrypi") #"/dev/ttyACM0" 
        baud = 115200
    except ImportError:
        serial_port = findSerialPort("linux")
        system = "linux"
        baud = 115200
else:
    serial_port = findSerialPort("windows")
    system = "windows"
    if debug: #Debug print
        print(serial_port) 
    baud = 115200

def retrySerial(): #Function for retrying serial port connection called by an UI button
    global serial_port, system
    serial_port = findSerialPort(system)
    if debug: #Debug print
        print(serial_port)

#Main function for sending any command over USB to the raspberry pi pico
#Outgoing messages must be in string format with spaces between each parameter and a newline at the end
#Incoming Messages are in string format and must be seperated if there are multiple parameters
def send_cmd(command, sleeptime):
    global serial_port
    try:
        with serial.Serial(serial_port, baud, timeout=2) as ser:
            ser.write(command.encode()) #Encodes the command to bytes and sends it over serial
            time.sleep(sleeptime)
            response = ser.readline().decode().strip()
            if debug: #Debug print
                print(f"Sent: {command}Received: {response}")
            return response
    except Exception as e:
        if debug: #Debug print
            print(f"Error: {e}")
        return None

def init_pico(accel = False): #Initializes the pico to its default state with all servos and I2C, UART, PWM
    send_cmd('initialize_pico '+ str(accel) + '\n', 0.02)


def getGPS(): #Gets the GPS data from the pico
    res = send_cmd('pollGPS ' + '\n', 1)
    try:
        if(res != ''):
            res = res[1:]
            res = res[:-1]
            resarr = res.split(',') #Response is in format string (latitude, longitude)
            if debug:
                print(resarr[0])
                print(resarr[1])
            return str(float(resarr[0])), str(float(resarr[1]))
        else:
            raise Exception
    except:
        return str(0), str(0) #Returns 0,0 if no response

def getSatCount(): #Gets the number of locked satellites from the pico
    return send_cmd('checkGPSSat ' + '\n', 0.3)

def getMagnetometer(): #Gets the magnetometer data from the pico
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

#Sets the vertical angle servo by doing multiple substeps from the current angle to the desired angle for less physical shaking
def setVerticalServo(pwm_freq, pwm_current_estimate):
    servoSpeed = 25 #Speed variable that determines the number of substeps - higher is faster
    if isinstance(pwm_freq, int) or isinstance(pwm_freq, float): #Checks if the input is a number
        if(pwm_freq > pwm_current_estimate): #Checks if the desired angle is higher than the current angle
            pwm_diff = pwm_freq - int(pwm_current_estimate) #Calculates the difference between the angles in PWM values
            i=1
            for i in range(int(pwm_diff/servoSpeed)): #For loop that does the substeps
                if(int(pwm_current_estimate + (i * (pwm_diff/int(pwm_diff/servoSpeed)))) >= int(pwm_freq)):  #Loop end condition in case of overshoot
                    send_cmd('setServoCycle vert_servo '+ str(int(pwm_freq)) + '\n', 0.01)
                    break
                send_cmd('setServoCycle vert_servo '+ str(int(pwm_current_estimate + (i * (pwm_diff/int(pwm_diff/servoSpeed))))) + '\n', 0.01) #Gradual step increases send command
            send_cmd('setServoCycle vert_servo '+ str(int(pwm_freq)) + '\n', 0.01) #Final step to ensure the desired angle is reached
        elif(pwm_freq < pwm_current_estimate): #Checks if the desired angle is lower than the current angle
            pwm_diff = int(pwm_current_estimate) - pwm_freq #Calculates the difference between the angles in PWM values
            i=1
            for i in range(int(pwm_diff/servoSpeed)): #For loop that does the substeps
                if(int(pwm_current_estimate - (i * (pwm_diff/int(pwm_diff/servoSpeed)))) >= int(pwm_freq)): #Loop end condition in case of overshoot
                    send_cmd('setServoCycle vert_servo '+ str(int(pwm_freq)) + '\n', 0.01)
                    break
                send_cmd('setServoCycle vert_servo '+ str(int(pwm_current_estimate - (i * (pwm_diff/int(pwm_diff/servoSpeed))))) + '\n', 0.01) #Gradual step decreases send command
            send_cmd('setServoCycle vert_servo '+ str(int(pwm_freq)) + '\n', 0.01) #Final step to ensure the desired angle is reached
        else:
            send_cmd('setServoCycle vert_servo '+ str(int(pwm_freq)) + '\n', 0.01) #If the angles are the same sends the command once

#Sets the horizontal heading servo by doing multiple substeps from the current heading to the desired heading for less shaking and smoother movement
def setHorizontalServo(pwm_freq, pwm_current_estimate):
    pwm_current_estimate = int(pwm_current_estimate)
    servoSpeed = 50 #Speed variable that determines the number of substeps - higher is faster speed
    if debug: #Debug print
        print(str(pwm_freq), str(pwm_current_estimate) + " sethorizontalServo freq, current estimate")
    if isinstance(pwm_freq, int) or isinstance(pwm_freq, float): #Checks if the input is a number
        if(pwm_freq > pwm_current_estimate): #Checks for the direction of movement by comparing the PWM values
            pwm_diff = pwm_freq - pwm_current_estimate # PWM value difference
            i=0
            for i in range(int(pwm_diff/servoSpeed)): 
                if(int(pwm_current_estimate + (i * (pwm_diff/int(pwm_diff/servoSpeed)))) >= int(pwm_freq)): #Loop end condition in case of overshoot
                    send_cmd('setServoCycle horizon_servo '+ str(int(pwm_freq)) + '\n', 0.02)
                    break
                send_cmd('setServoCycle horizon_servo '+ str(int(pwm_current_estimate + (i * (pwm_diff/int(pwm_diff/servoSpeed))))) + '\n', 0.02) #Gradual step increases send command
            send_cmd('setServoCycle horizon_servo '+ str(int(pwm_freq)) + '\n', 0.02) #Final step to ensure the desired heading is reached
        elif(pwm_freq < pwm_current_estimate):
            pwm_diff = pwm_current_estimate - pwm_freq
            i=0
            for i in range(int(pwm_diff/servoSpeed)):
                if(int(pwm_current_estimate - (i * (pwm_diff/int(pwm_diff/servoSpeed)))) <= int(pwm_freq)): #Loop end condition in case of overshoot
                    send_cmd('setServoCycle horizon_servo '+ str(int(pwm_freq)) + '\n', 0.02)
                    break
                send_cmd('setServoCycle horizon_servo '+ str(int(pwm_current_estimate - (i * (pwm_diff/int(pwm_diff/servoSpeed))))) + '\n', 0.02) #Gradual step decreases send command
            send_cmd('setServoCycle horizon_servo '+ str(int(pwm_freq)) + '\n', 0.02) #Final step to ensure the desired heading is reached
        else:
            send_cmd('setServoCycle horizon_servo '+ str(int(pwm_freq)) + '\n', 0.02) #If the heading is same as current heading sends the command once

#If ADXL feature is used, returns accelerometer values
def getAccelVal():
    return send_cmd('getADXL' + '\n', 0.05)

