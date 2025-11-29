#!/bin/bash
sleep 10s
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

gnome-terminal -x bash -c "source /home/ysc/lite_cog/transfer/devel/setup.bash; roslaunch message_transformer message_transformer.launch; read -p 'Press any key to exit...'"

sleep 10s
gnome-terminal -x bash -c "source /home/ysc/lite_cog/drivers/leishen_ws/devel/setup.bash; roslaunch lslidar_driver lslidar_c16.launch ; read -p 'Press any key to exit...'"

sleep 5s
gnome-terminal -x bash -c "source /home/ysc/lite_cog/nav/devel/setup.bash; roslaunch hdl_localization local_rslidar_imu.launch ; read -p 'Press any key to exit...'"







