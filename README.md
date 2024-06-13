https://github.com/elephantrobotics/myagv_ros

roscore
python gpio_run.py
roslaunch myagv_odometry myagv_active.launch
rosrun rosserial_python serial_node.py
rosrun rosserial_python arduino_send.py

roslaunch myagv_navigtation navigation_active.launch
rosrun myagv_odometry camera_sub.py
rosrun myagv_odometry send_goals.py
