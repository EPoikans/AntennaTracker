import serial
import time

serial_port = 'COM10'

baud = 115200

def send_cmd(command):
    try:
        with serial.Serial(serial_port, baud, timeout=2) as ser:
            ser.write(command.encode())
            time.sleep(0.01)
            response = ser.readline().decode().strip()
            #print(f"Sent: {command}Received: {response}")
            return response
    except Exception as e:
        print(f"Error: {e}")
        return None

def init_pico(accel = False):
    send_cmd('initialize_pico '+ str(accel) + '\n')

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
