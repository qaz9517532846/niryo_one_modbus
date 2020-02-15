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

    print "Send a Pose Move command to the robot"
    Pose = [0.2, 0.0, 0.3, 0, 1.5708, 0]
    Pose_to_send = list(map(lambda j: int(number_to_raw_data(j*1000)), Pose))
    print Pose_to_send

    client.write_registers(10, Pose_to_send)
    client.write_register(101, 1)

    # Wait for end of Move command
    while client.read_holding_registers(150, count=1).registers[0] == 1:
        time.sleep(0.01)

    print "Pose Move command is finished"

    print "Send a Pose Move command to the robot"
    Pose = [0.2, 0.0, 0.16, 0, 1.5708, 0]
    Pose_to_send = list(map(lambda j: int(number_to_raw_data(j*1000)), Pose))
    print Pose_to_send

    client.write_registers(10, Pose_to_send)
    client.write_register(101, 1)

    # Wait for end of Move command
    while client.read_holding_registers(150, count=1).registers[0] == 1:
        time.sleep(0.01)

    print "Pose Move command is finished"

    print "Select Gripper ID"
    client.write_register(500, 12)

    # gripper close 
    print "setting Gripper close speed"
    client.write_register(402, 300)

    print "Gripper close"
    client.write_register(511, 12)

    while client.read_holding_registers(150, count=1).registers[0] == 1:
          time.sleep(0.05)

    time.sleep(1)

    print "Send a Pose Move command to the robot"
    Pose = [0.2, 0, 0.3, 0, 1.5708, 0]
    Pose_to_send = list(map(lambda j: int(number_to_raw_data(j*1000)), Pose))
    print Pose_to_send

    client.write_registers(10, Pose_to_send)
    client.write_register(101, 1)

    # Wait for end of Move command
    while client.read_holding_registers(150, count=1).registers[0] == 1:
        time.sleep(0.01)

    print "Pose Move command is finished"

    print "Send a Pose Move command to the robot"
    Pose = [0.25, 0, 0.3, 0, 1.5708, 0]
    Pose_to_send = list(map(lambda j: int(number_to_raw_data(j*1000)), Pose))
    print Pose_to_send

    client.write_registers(10, Pose_to_send)
    client.write_register(101, 1)

    # Wait for end of Move command
    while client.read_holding_registers(150, count=1).registers[0] == 1:
        time.sleep(0.01)

    print "Pose Move command is finished"

    print "Send a Pose Move command to the robot"
    Pose = [0.25, 0, 0.16, 0, 1.5708, 0]
    Pose_to_send = list(map(lambda j: int(number_to_raw_data(j*1000)), Pose))
    print Pose_to_send

    client.write_registers(10, Pose_to_send)
    client.write_register(101, 1)

    # Wait for end of Move command
    while client.read_holding_registers(150, count=1).registers[0] == 1:
        time.sleep(0.01)

    print "Pose Move command is finished"

    # gripper open
    print "setting Gripper open speed"
    client.write_register(401, 300)

    print "Gripper open"
    client.write_register(510, 12)

    while client.read_holding_registers(150, count=1).registers[0] == 1:
          time.sleep(0.05)

    time.sleep(1)

    print "Send a Pose Move command to the robot"
    Pose = [0.25, 0, 0.3, 0, 1.5708, 0]
    Pose_to_send = list(map(lambda j: int(number_to_raw_data(j*1000)), Pose))
    print Pose_to_send

    client.write_registers(10, Pose_to_send)
    client.write_register(101, 1)

    # Wait for end of Move command
    while client.read_holding_registers(150, count=1).registers[0] == 1:
        time.sleep(0.01)

    client.close()
    print "Close connection to modbus server"
    print "--- END"
