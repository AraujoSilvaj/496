import pyvesc
from pyvesc import GetValues, SetRPM, SetCurrent, SetRotorPositionMode, GetRotorPosition
from pyvesc.messages.base import VESCMessage
from pyvesc.packet.structure import *
import pyvesc.packet.codec
import serial
import time

class Sample1(metaclass=VESCMessage):    
    """ sample answer
    """
    id = 0xAA
    can_id = None
    
    temp_fet_filtered = 1
    temp_motor_filtered = 2
    avg_motor_current = 3
    avg_input_current = 4
    avg_id = 5
    avg_iq = 6
    duty_cycle_now = 7
    rpm = 8
    input_voltage = 9
    amp_hours = 10
    amp_hours_charged = 11
    watt_hours = 12
    watt_hours_charged = 13
    tachometer_value = 14
    tachometer_abs = 15
    fault = 16

    fields = [
            ('temp_fet_filtered', 'e', 1),
            ('temp_motor_filtered', 'e', 1),
            ('avg_motor_current', 'f', 1),
            ('avg_input_current', 'f', 1),
            ('avg_id', 'f', 1),
            ('avg_iq', 'f', 1),
            ('duty_cycle_now',  'e', 1),
            ('rpm', 'f', 1),
            ('input_voltage', 'e', 1),
            ('amp_hours',  'f', 1),
            ('amp_hours_charged',  'f', 1),
            ('watt_hours', 'f', 1),
            ('watt_hours_charged', 'f', 1),
            ('tachometer_value', 'i', 1),
            ('tachometer_abs', 'i', 1),
            ('fault', 'i', 1)
    ]



def test():
    print("---sample request---")
    packet = pyvesc.encode_request(Sample1)
    print(packet)
    
    print("---sample answer---")
    buffer = pyvesc.encode(Sample1)
    print (buffer)           
    
    msg, consumed = pyvesc.decode(buffer)    
    
    #print(consumed)
    
    print(msg.rpm)


if __name__ == "__main__":
    test()
