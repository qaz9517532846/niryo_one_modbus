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

    print "Calibrate Robot if needed"
    client.write_register(311,1)
    time.sleep(1)

    # Wait for end of calibration
    while client.read_input_registers(402, 1).registers[0] == 1:
        time.sleep(0.05)

    time.sleep(1)
    print "learning off"
    client.write_register(300,0)
    

    print "Close connection to modbus server"
    print "--- END"
