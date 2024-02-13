import pyvesc
from pyvesc import GetFirmwareVersion, \
    GetValues, SetRPM, SetCurrent, \
    SetRotorPositionMode, GetRotorPosition, \
    SetDutyCycle, SetServoPosition, SetPosition, \
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

serialport = '/dev/ttyVESC'

print("port " + serialport)

desired_rpm = 0
steering_angle = 0

def getKeyRPM():
  global desired_rpm
  while True:
    try:
        user_input = input("Rpm?")
        desired_rpm = float(user_input)
    except ValueError:
        print("Invalid input. Please enter a valid rpm.")
        
def getKeySteering():
  global steering_angle
  while True:
    try:
      user_input = input("Desired Steering angle: ")
      steering_angle = float(user_input)
    except ValueError:
      print("Invalid input. Please enter a valid steering angle.")
      
def ctof(value):
  return (value * 9.0/5) + 32
      
def do_VESC():
  global desired_rpm
  global steering_angle
  inbuf = 'b'
  oldRPM = -1
  oldSteeringAngle = -1
  with serial.Serial(serialport, baudrate=115200, timeout=0) as ser:
        try:
            ser.flushInput()
            ser.flushOutput()
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
                    '''for field in vars(response):
                    if field[0] == "_":
                      continue
                    print(f"{field}: {vars(response)[field]}")
                    '''
                    #print(dir(response))
                    if oldRPM != desired_rpm:
                        print(f"Desired Steering Movement: {desired_rpm}")
                        # Adjust the scale based on your servo and desired range
                        servo_pos = desired_rpm / 100.0  
                        ser.write(pyvesc.encode(SetServoPosition(servo_pos)))
                        oldRPM = desired_rpm   
                    # Report refesh rate
                    #print(f"VESC refresh rate: {1.0/(time.time() - last_sample_time):0.1f}")
                    
                    if oldSteeringAngle != steering_angle:
                      print(f"Desired Steering Movement: {steering_angle}")
                      # Adjust the scale based on your servo and desired range
                      servo_pos = steering_angle / 100.0
                      ser.write(pyvesc.encode(SetServoPosition(servo_pos)))
                      oldSteeringAngle = steering_angle
                      
                    last_sample_time = time.time()
                  
                
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
                    
