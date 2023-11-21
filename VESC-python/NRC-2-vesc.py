'''
NRC-vesc.py
Oct , 2023
Paul Talaga
This connects to a VESC over USB, asks it to stream 'realtime data', prints out that data
and allows the user to set a desired RPM.

To set an rpm, just type a number and hit enter. A separate thread manages this
but since the data is streaming on the screen, you won't see what you type.

'''

import pyvesc
from pyvesc import GetFirmwareVersion, \
    GetValues, SetRPM, SetCurrent, \
    SetRotorPositionMode, GetRotorPosition, \
    SetDutyCycle, SetPosition, \
    GetRotorPositionCumulative, \
    SetCurrentGetPosCumulative, \
    SetPositionCumulative, SetTerminalCommand, \
    GetPrint, GetConfig, SetConfig, \
    SetAlive, GetDetectEncoder 
import serial
import os
#import math
import time
import threading

#import struct

# Set your serial port here (either /dev/ttyX or COMX)
# serialport = '/dev/cu.usbmodem3011'
# serialport = '/dev/ttyACM0'
serialport = 'COM3'

print("port " + serialport)

desired_rpm = 0
duty_cycle = 0

def getKeyRPM():
    global desired_rpm
    while True:
        input_rpm = input("Rpm?")
        try:
            desired_rpm = int(input_rpm)
        except ValueError:
            print("Please enter a valid RPM value.")

def ctof(value):
    return (value * 9.0/5) + 32

def do_VESC():    
    global desired_rpm
    global duty_cycle
    last_sample_time = 0
    inbuf = b''    
    oldRPM = -1
    with serial.Serial(serialport, baudrate=115200, timeout=0) as ser:
        try:
            ser.flushInput()
            ser.flushOutput()

            ser.write(pyvesc.encode(SetTerminalCommand('ping')))            

            while True:
                ser.write(pyvesc.encode(SetAlive))
                ser.write(pyvesc.encode_request(GetValues))
                
                time.sleep(0.01)

                while True:
                    while ser.in_waiting > 0:
                        inbuf += ser.read(ser.in_waiting)
                    if len(inbuf) == 0:
                        break

                    (response, consumed) = pyvesc.decode(inbuf)
                    if consumed == 0:
                        break
                    inbuf = inbuf[consumed:]

                    if isinstance(response, GetValues):
                        for field in vars(response):
                            if field[0] == "_":
                                continue
                            print(f"{field}: {vars(response)[field]}")

                        if oldRPM != desired_rpm:
                            print(f"Desired RPM: {desired_rpm}")
                            if desired_rpm > 1500:
                                duty_cycle = 0
                                ser.write(pyvesc.encode(SetRPM(desired_rpm)))
                            else:
                                duty_cycle = 0.5  # You can adjust this value as needed for your application
                                ser.write(pyvesc.encode(SetDutyCycle(duty_cycle)))
                            oldRPM = desired_rpm
                            
                        print(f"VESC refresh rate: {1.0 / (time.time() - last_sample_time):0.1f}")
                        last_sample_time = time.time()
                        print()

        except KeyboardInterrupt:
            print("Turning off the VESC...")
            ser.flushOutput()
            ser.flushInput()
            ser.close()

if __name__ == "__main__":
    threading.Thread(target=getKeyRPM).start()
    do_VESC()
