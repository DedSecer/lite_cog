#!/bin/bash
source ~/.bashrc
sudo -S pkill qnx2ros << EOF 
'
EOF
sudo systemctl kill transfer
sudo systemctl stop transfer
sudo pkill qnx2ros
sudo pkill ros2qnx
sudo pkill nx2app
sudo pkill sensor_checker
sudo pkill static_transfor 
sudo pkill lslidar_driver_
sudo pkill move_base 
sudo pkill map_server
sudo pkill rviz 
sudo pkill roslaunch
