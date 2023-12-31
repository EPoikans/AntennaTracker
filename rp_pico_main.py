"""

MicroPython code stored locally on the raspberry pi Pico


from machine import Pin, SPI, PWM
import sys
from time import sleep
import select

def initialize_pico():
    # Registers
    REG_DEVID = 0x00
    DEVID = 0xE5
    REG_POWER_CTL = 0x2D
    REG_DATAX0 = 0x32
    REG_DATAY0 = 0x34
    REG_DATAZ0 = 0x36
    SENSITIVITY_2G = 1.0 / 25
    EARTH_GRAVITY = 9.80665
    global state, vertservo, horizonservo
    # Initialize SPI
    spi = SPI(0, baudrate=500000, polarity=0, phase=1, bits=8, firstbit=SPI.MSB, sck=Pin(18), mosi=Pin(19), miso=Pin(16))
    cs = Pin(17, mode=Pin.OUT, value=1)
    #reg_read(spi, cs, REG_DEVID)
    #data = reg_read(spi, cs, REG_DEVID)
    #print(DEVID)
    #if (data != bytearray((DEVID,))):
     #   print("ERROR: Could not communicate with ADXL345")
     #   sys.exit()
    #data = reg_read(spi,cs, REG_POWER_CTL)
    #print(data)
    #data = int.from_bytes(data, "big") or (1 << 3)
    #reg_write(spi, cs, REG_POWER_CTL, data)
    #data = reg_read(spi, cs, REG_POWER_CTL)
    #print(data)
    #data = reg_read(spi, cs, REG_DATAX0, 6)
    #print(data)
    #print(data)

    vertservo = PWM(Pin(5))
    horizonservo = PWM(Pin(6))
    vertservo.freq(50)
    horizonservo.freq(50)
    vertservo.duty_u16(2000) #Vertical angle servo 2000-0deg, 5400 -90deg
    horizonservo.duty_u16(4300)
    global vertservo, horizonservo, state
    state = True
    return state, vertservo, horizonservo

def reg_write(spi, cs, reg, data):
    msg = bytearray()
    msg.append(0x00 or reg)
    msg.append(data)
    cs.value(0)
    spi.write(msg)
    cs.value(1)

def reg_read(spi, cs, reg, nbytes=1):
    if nbytes < 1:
        return bytearray()
    elif nbytes == 1:
        mb = 0
    else:
        mb = 1
    msg = bytearray()
    msg.append(0x80 or (mb << 6) or reg)
    cs.value(0)
    spi.write(msg)
    data = spi.read(nbytes)
    print(int.from_bytes(data, 'big'))
    cs.value(1)
    return data

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
        state, vertservo, horizonservo = initialize_pico()
        if(state):
            return("Initialized")  

poll_obj = select.poll()
poll_obj.register(sys.stdin, 1)
initialize_pico()
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
                print('boo')
#def getADXL()
#start, vertservo, horizonservo = initialize_pico()
#if(start):
#    sleep(2)
#    setServoCycle (vertservo, 1200)
#    sleep(2)
#    setServoCycle (vertservo, 5400)
#    print(vertservo.duty_u16())d



"""