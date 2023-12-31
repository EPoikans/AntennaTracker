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
            print(f"Sent: {command}Received: {response}")
            return response
    except Exception as e:
        print(f"Error: {e}")
        return None

send_cmd('initialize_pico accelerometer' + '\n')
time.sleep(2)
send_cmd('setServoCycle vert_servo 5000' + '\n')
time.sleep(2)
send_cmd('setServoCycle vert_servo 7000' + '\n')
send_cmd('setServoCycle horizon_servo 2000' + '\n')
send_cmd('getADXL' + '\n')
