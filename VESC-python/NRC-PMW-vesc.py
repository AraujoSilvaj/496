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
from pyvesc import GetValues, SetDutyCycle, SetAlive
import serial
import time
import threading

serialport = 'COM3'

print("port " + serialport)

desired_pwm = 0.0
pwm_lock = threading.Lock()

def getKeyPWM():
    global desired_pwm
    while True:
        try:
            new_pwm = float(input("Desired PWM? "))
            with pwm_lock:
                desired_pwm = new_pwm
        except ValueError:
            print("Invalid input. Please enter a valid floating-point number.")

def do_VESC():
    global desired_pwm
    last_sample_time = 0
    inbuf = b''
    old_pwm = -1.0
    with serial.Serial(serialport, baudrate=115200, timeout=0) as ser:
        try:
            ser.flushInput()
            ser.flushOutput()

            ser.write(pyvesc.encode(SetAlive))

            while True:
                ser.write(pyvesc.encode(SetAlive))
                ser.write(pyvesc.encode_request(GetValues))

                time.sleep(0.01)

                while ser.in_waiting > 0:
                    inbuf += ser.read(ser.in_waiting)

                if len(inbuf) == 0:
                    continue

                (response, consumed) = pyvesc.decode(inbuf)

                if consumed == 0:
                    continue

                inbuf = inbuf[consumed:]

                if isinstance(response, GetValues):
                    for field in vars(response):
                        if field[0] == "_":
                            continue
                        print(f"{field}: {vars(response)[field]}")

                    with pwm_lock:
                        if old_pwm != desired_pwm:
                            print(f"Desired PWM: {desired_pwm}")
                            ser.write(pyvesc.encode(SetDutyCycle(desired_pwm)))
                            old_pwm = desired_pwm

                    print(f"VESC refresh rate: {1.0 / (time.time() - last_sample_time):0.1f}")
                    last_sample_time = time.time()
                    print()

        except KeyboardInterrupt:
            print("Turning off the VESC...")
            with pwm_lock:
                ser.write(pyvesc.encode(SetDutyCycle(0.0)))

if __name__ == "__main__":
    threading.Thread(target=getKeyPWM).start()
    do_VESC()
