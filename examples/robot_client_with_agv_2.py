#!/usr/bin/env python

import sys
import rospy
import time

from pymodbus.client.sync import ModbusTcpClient
from std_msgs.msg import String
from cv_vision.srv import *
from scl_agv.srv import *

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

def niryo_move_world(x, y, z, rx, ry, rz):
    print "Send a Pose Move command to the robot"
    Pose = [x, y, z, rx, ry, rz]
    Pose_to_send = list(map(lambda j: int(number_to_raw_data(j*1000)), Pose))
    print Pose_to_send

    client.write_registers(10, Pose_to_send)
    client.write_register(101, 1)

    while client.read_holding_registers(150, count=1).registers[0] == 1:
        time.sleep(0.01)

    print "Pose Move command is finished"

def robot_from_camera(camera_capcture):
    rospy.wait_for_service('robot_vision')
    global pos_x
    global pos_y
    global pos_z
    global pos_rx
    global pos_ry
    global pos_rz
    try:
        vision2robot = rospy.ServiceProxy('robot_vision', robot2vision)
        response_cam = vision2robot(camera_capcture)
        object_num = response_cam.num
        pos_x      = response_cam.robot_x
        pos_y      = response_cam.robot_y
        pos_z      = response_cam.robot_z
        pos_rx  = response_cam.rotation_x
        pos_ry  = response_cam.rotation_y
        pos_rz  = response_cam.rotation_z
        if object_num > 0:
           Robot_motion()       
    except rospy.ServiceException, e:
        print "Service call faild: %s" %e

def Robot_motion():
    niryo_move_world(pos_x - 0.1, pos_y, pos_z, 0, 0, -pos_rz)
    
    # wait time
    time.sleep(1)

    niryo_move_world(pos_x, pos_y, pos_z, 0, 0, -pos_rz)

    # wait time
    time.sleep(1)

    niryo_move_world(pos_x - 0.1, pos_y, pos_z, 0, 0, -pos_rz)

    print "learning ON"
    client.write_register(300,1)

    # wait time
    time.sleep(1)

def agv_niryo_service(req):
    if req.agv_command == 1:
       print "learning off"
       client.write_register(300,0)

       niryo_move_world(0.22, 0, 0.2, 0, 0, 0)
       camera_capcture = 1
       print "Send command to camera"
       robot_from_camera(camera_capcture)

    elif req.agv_command == 2:
       print "learning off"
       client.write_register(300,0)

       niryo_move_world(0.22, 0, 0.2, 0, 0, 0)
       camera_capcture = 2
       print "Send command to camera"
       robot_from_camera(camera_capcture)
    else:
       camera_capcture = 0
       print "please check agv_command"
    return agv_niryoResponse(camera_capcture)
    

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
