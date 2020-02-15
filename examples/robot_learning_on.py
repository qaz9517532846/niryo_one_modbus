#!/usr/bin/env python

# ! You need to launch the server first !

from pymodbus.client.sync import ModbusTcpClient
import time

# Positive number : 0 - 32767
# Negative number : 32768 - 65535
def number_to_raw_data(val):
    if val < 0:
        val = (1 << 15) - val
    return val

def raw_data_to_number(val):
    if (val >> 15) == 1:
        val = - (val & 0x7FFF)
    return val

if __name__ == '__main__':
    print "--- START"
    client = ModbusTcpClient('169.254.200.200', port=5020)

    client.connect()
    print "Connected to modbus server"

    joints = [0, 0, 0, 0, 0, 0]
    joints_to_send = list(map(lambda j: number_to_raw_data(j*1000), joints))

    client.write_registers(0, joints_to_send)
    client.write_register(100, 1)

    # Wait for end of Move command
    while client.read_holding_registers(150, count=1).registers[0] == 1:
        time.sleep(0.05)


    print "learning off"
    client.write_register(300,1)
    

    print "Close connection to modbus server"
    print "--- END"
