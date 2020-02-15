#!/usr/bin/env python

import sys
import rospy
import time

from pymodbus.client.sync import ModbusTcpClient
from std_msgs.msg import String
from vision.srv import *
from agv_scl.srv import *

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

def robot_from_camera(camera_capcture):
    rospy.wait_for_service('robot_vision')
    global pos_x
    global pos_y
    global pos_theta
    try:
        vision2robot = rospy.ServiceProxy('robot_vision', robot2vision)
        response_cam = vision2robot(camera_capcture)
        object_num = response_cam.num
        pos_x      = response_cam.robot_x
        pos_y      = response_cam.robot_y
        pos_theta  = response_cam.robot_theta
        if object_num > 0:
           Robot_motion()       
    except rospy.ServiceException, e:
        print "Service call faild: %s" %e

def Robot_motion():

    # Select Gripper ID
    print "Select Gripper ID"
    client.write_register(500, 12)

    # Move to [obj_x, obj_y, 0.3, 0, 1.5708, obj_theta]
    print "Send a Pose Move command to the robot"
    Pose = [pos_x, pos_y, 0.3, 0, 1.5708, pos_theta]
    Pose_to_send = list(map(lambda j: int(number_to_raw_data(j*1000)), Pose))
    print Pose_to_send

    client.write_registers(10, Pose_to_send)
    client.write_register(101, 1)

    while client.read_holding_registers(150, count=1).registers[0] == 1:
        time.sleep(0.01)

    print "Pose Move command is finished"

    # Move to [obj_x, obj_y, 0.16, 0, 1.5708, obj_theta]
    print "Send a Pose Move command to the robot"
    Pose = [pos_x, pos_y, 0.15, 0, 1.5708, pos_theta]
    Pose_to_send = list(map(lambda j: int(number_to_raw_data(j*1000)), Pose))
    print Pose_to_send

    client.write_registers(10, Pose_to_send)
    client.write_register(101, 1)

    while client.read_holding_registers(150, count=1).registers[0] == 1:
        time.sleep(0.01)

    print "Pose Move command is finished"

    # wait time
    time.sleep(1)

    # Gripper Close
    print "setting Gripper close speed"
    client.write_register(402, 300)

    print "Gripper close"
    client.write_register(511, 12)

    while client.read_holding_registers(150, count=1).registers[0] == 1:
          time.sleep(0.05)

    # wait time
    time.sleep(1)

    # Move to [obj_x, obj_y, 0.3, 0, 1.5708, obj_theta]
    print "Send a Pose Move command to the robot"
    Pose = [pos_x, pos_y, 0.3, 0, 1.5708, pos_theta]
    Pose_to_send = list(map(lambda j: int(number_to_raw_data(j*1000)), Pose))
    print Pose_to_send

    client.write_registers(10, Pose_to_send)
    client.write_register(101, 1)

    while client.read_holding_registers(150, count=1).registers[0] == 1:
        time.sleep(0.01)

    print "Pose Move command is finished"

    # Move to [0.2, 0, 0.3, 0, 1.5708, 0]
    print "Send a Pose Move command to the robot"
    Pose = [0.2, 0, 0.3, 0, 1.5708, 0]
    Pose_to_send = list(map(lambda j: int(number_to_raw_data(j*1000)), Pose))
    print Pose_to_send

    client.write_registers(10, Pose_to_send)
    client.write_register(101, 1)

    while client.read_holding_registers(150, count=1).registers[0] == 1:
        time.sleep(0.01)

    print "Pose Move command is finished"

    # Move to [0.2, 0, 0.16, 0, 1.5708, 0]
    print "Send a Pose Move command to the robot"
    Pose = [0.2, 0, 0.16, 0, 1.5708, 0]
    Pose_to_send = list(map(lambda j: int(number_to_raw_data(j*1000)), Pose))
    print Pose_to_send

    client.write_registers(10, Pose_to_send)
    client.write_register(101, 1)

    while client.read_holding_registers(150, count=1).registers[0] == 1:
        time.sleep(0.01)

    print "Pose Move command is finished"

    # wait time
    time.sleep(1)

    # Gripper Open
    print "setting Gripper close speed"
    client.write_register(401, 300)

    print "Gripper close"
    client.write_register(510, 12)

    while client.read_holding_registers(150, count=1).registers[0] == 1:
          time.sleep(0.05)

    # wait time
    time.sleep(1)

    # Move to [0.2, 0, 0.3, 0, 1.5708, 0]
    print "Send a Pose Move command to the robot"
    Pose = [0.2, 0, 0.3, 0, 1.5708, 0]
    Pose_to_send = list(map(lambda j: int(number_to_raw_data(j*1000)), Pose))
    print Pose_to_send

    client.write_registers(10, Pose_to_send)
    client.write_register(101, 1)

    while client.read_holding_registers(150, count=1).registers[0] == 1:
        time.sleep(0.01)

def agv_niryo_service(req):

    # Move to [0.2, 0.0, 0.3, 0, 1.5708, 0]
    print "Send a Pose Move command to the robot"
    Pose = [0.2, 0.0, 0.3, 0, 1.5708, 0]
    Pose_to_send = list(map(lambda j: int(number_to_raw_data(j*1000)), Pose))
    print Pose_to_send

    client.write_registers(10, Pose_to_send)
    client.write_register(101, 1)

    while client.read_holding_registers(150, count=1).registers[0] == 1:
        time.sleep(0.01)

    print "Pose Move command is finished"

    camera_capcture = 1

    print "Send command to camera"
    robot_from_camera(camera_capcture)

    return agv_niryoResponse(1)
    

################################ Main ##### #################################

if __name__ == '__main__':
    rospy.init_node('robot_client', anonymous=True)

    # Client connect
    print "--- START"
    client = ModbusTcpClient('169.254.200.200', port=5020)

    client.connect()
    print "Connected to modbus server"

    agv_service = rospy.Service('agv_connect_niryo', agv_niryo, agv_niryo_service)
    print "Ready to niryo robot."
    rospy.spin()

    
