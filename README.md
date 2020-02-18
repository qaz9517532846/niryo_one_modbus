# niryo_one_modbus
Control niryo robot arm using Python in ROS. 

Niryo robot arm example code - test IO.

$ rosrun niryo_one_modbus client_modbus_test.py

Niryo robot arm example code - Comtrol motion(joint).

$ rosrun niryo_one_modbus client_move_command.py

Niryo robot arm example code - Comtrol motion(world).

$ rosrun niryo_one_modbus client_pose_move_command.py

Niryo robot arm example code - test server.

$ rosrun niryo_one_modbus modbus_server_test.py

Niryo robot joint move to [0, 0, 0, 0, -1.5708, 0]

$ rosrun niryo_one_modbus move_home.py

Niryo robot calibration and learning off.

$ rosrun niryo_one_modbus robot_calibration.py

Niryo robot control with IDS camera using ros service.

$ rosrun niryo_one_modbus robot_client.py

Niryo robot control with IDS camera and agv using ros service.

$ rosrun niryo_one_modbus robot_client_with_agv.py

Niryo robot control with IDS camera and agv using ros service.

$ rosrun niryo_one_modbus robot_client_with_agv_2.py

Niryo robot calibration and learning on.

$ rosrun niryo_one_modbus robot_learning_on.py

Niryo robot arm example code - Comtrol motion(joint).

$ rosrun niryo_one_modbus robot_test.py

Niryo robot control with IDS camera using ros topic.

$ rosrun niryo_one_modbus system_robot_arm.py
