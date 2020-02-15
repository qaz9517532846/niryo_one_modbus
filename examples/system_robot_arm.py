#!/usr/bin/env python

from pymodbus.client.sync import ModbusTcpClient
from std_msgs.msg import String
import rospy
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

def callback(data):
    if data.data == 'commmand to Robot arm' :
       rospy.loginfo('%s', 'I recieved command from AGV')
       rospy.Subscriber('obj_pose', String, callback_pose)
       rospy.spin()


def callback_pose(data_pose):
    rospy.loginfo('obj_pose = %s', data_pose.data)
    obj_pose = data_pose.data.split(',', 2)

    global pos_x
    global pos_y
    global pos_theta

    pos_x = float(obj_pose[0])
    pos_y = float(obj_pose[1])
    pos_theta =  float(obj_pose[2]) 

    for i in range(2):
        hello_str = str('not vision system start')
        rospy.loginfo(hello_str)
        send_command.publish(hello_str)
        rate.sleep()
    
    Robot_motion()

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
    Pose = [pos_x, pos_y, 0.16, 0, 1.5708, pos_theta]
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

    # wait time
    time.sleep(3)

    for i in range(2):
           hello_str = str('vision system start')
           rospy.loginfo(hello_str)
           send_command.publish(hello_str)
           rate.sleep()

    


################################ Main ######################################

if __name__ == '__main__':
    rospy.init_node('Robot_arm', anonymous=True)

    # Send command to Vision System
    global send_command
    send_command = rospy.Publisher('send_to_vision', String, queue_size=1000)
    rate = rospy.Rate(1) # 1hz

    for i in range(3):
        hello_str = str('vision system start')
        rospy.loginfo(hello_str)
        send_command.publish(hello_str)
        rate.sleep()

    # Client connect
    print "--- START"
    client = ModbusTcpClient('169.254.200.200', port=5020)

    client.connect()
    print "Connected to modbus server"

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

    time.sleep(1) # wait 1 sec
    rate = rospy.Rate(100) # 100hz

    # recieved data from vision system
    rospy.Subscriber('string_command', String, callback)
    rospy.spin()
