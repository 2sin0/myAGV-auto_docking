https://github.com/elephantrobotics/myagv_ros

1. roscore
2. python gpio_run.py
3. roslaunch myagv_odometry myagv_active.launch
4. rosrun rosserial_python serial_node.py
5. rosrun rosserial_python arduino_send.py
6. roslaunch myagv_navigtation navigation_active.launch
7. rosrun myagv_odometry camera_sub.py
8. rosrun myagv_odometry send_goals.py
