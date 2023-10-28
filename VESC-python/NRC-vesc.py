'''
  NRC-vesc.py
  Oct , 2023
  Paul Talaga
  This connects to a VESC over USB, asks it to stream 'realtime data', prints out that data
  and allows the user to set a desired RPM.

  To set an rpm, just type a number and hit enter.  A separate thread manages this
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
#serialport = '/dev/cu.usbmodem3011'
serialport = '/dev/ttyACM0'


print("port " + serialport)

desired_rpm = 0

def getKeyRPM():
  global desired_rpm
  while True:
    desired_rpm = input("Rpm?")
    
def ctof(value):
  return (value * 9.0/5) + 32

def do_VESC():    
    global desired_rpm
    last_sample_time = 0
    inbuf = b''    
    oldRPM = -1
    with serial.Serial(serialport, baudrate=115200, timeout=0) as ser:
        try:
            ser.flushInput()
            ser.flushOutput()
                                                                     
            #sendConfig(ser)
            
            ser.write(pyvesc.encode(SetTerminalCommand('ping')))                        
                                                                             
                                    
            
            while True:
                ser.write(pyvesc.encode(SetAlive))                                                                    
                ser.write(pyvesc.encode_request(GetValues))                                                    
                
                

                time.sleep(0.01)
                
                                                
                while (True):
                  # Check if there is enough data back for a measurement
                  while ser.in_waiting > 0:                   
                    inbuf += ser.read(ser.in_waiting)                                   
                  if len(inbuf) == 0: break                 
                  
                  (response, consumed) = pyvesc.decode(inbuf)
                  if consumed == 0: break           # Need more data to get a full message                     
                  inbuf = inbuf[consumed:]                      
                  
                  # Only if we got a values packet do we report and update rpm                           
                  if isinstance(response, GetValues):
                    '''print(f" Fet temp C: {ctof(response.temp_fet_filtered):0.3f} " + 
                      f" motor temp C: {ctof(response.temp_motor_filtered):0.3f} " +
                      f" In volt: {response.input_voltage:0.2f}" + 
                      f" motor curr: {response.avg_motor_current:0.3f}" + 
                      f" avg input current: {response.avg_input_current:0.3f}" +
                      f" Duty: {response.duty_cycle_now:0.3f}" + 
                      #" Watt Hours:" + str(response.watt_hours) + 
                      #" Watt Hours Charged:" + str(response.watt_hours_charged) + 
                      #" amp Hours:" + str(response.amp_hours) + 
                      #" amp Hours Charged:" + str(response.amp_hours_charged) + 
                       "")'''
                    # Loop through all the properies of the reponse and print
                    for field in vars(response):
                      if field[0] == "_":
                        continue
                      print(f"{field}: {vars(response)[field]}")
                    
                    #print(dir(response))

                    # Send a new rpm    
                    #ser.write(pyvesc.encode(SetRPM(1000))) 
                    if oldRPM != desired_rpm: 
                      print(f"Desired: {desired_rpm}") 
                      ser.write(pyvesc.encode(SetRPM(desired_rpm)))  
                      oldRPM = desired_rpm    
                    # Report refesh rate
                    print(f"VESC refresh rate: {1.0/(time.time() - last_sample_time):0.1f}")   
                    last_sample_time = time.time()   
                    print()   


                  

        except KeyboardInterrupt:
            # Turn Off the VESC
            print ("turning off the VESC...")
            #ser.write(pyvesc.encode(SetCurrent(0)))            
            ser.flushOutput()
            ser.flushInput()
            ser.close()
            

if __name__ == "__main__":
    # Spawn a thread to listen to the keyboard
    threading.Thread(target=getKeyRPM).start()  
    do_VESC()