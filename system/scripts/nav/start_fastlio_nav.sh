#!/bin/bash
# Fast-LIO 定位导航启动脚本
# 用法: ./start_fastlio_nav.sh [map_name] [init_x] [init_y] [init_yaw]

MAP_NAME=${1:-"lite3"}
INIT_X=${2:-"0.0"}
INIT_Y=${3:-"0.0"}
INIT_YAW=${4:-"0.0"}

echo "=========================================="
echo "  Fast-LIO 定位导航启动脚本"
echo "=========================================="
echo "地图: $MAP_NAME"
echo "初始位置: x=$INIT_X, y=$INIT_Y, yaw=$INIT_YAW"
echo "=========================================="

# 启动 Fast-LIO 定位（在新终端）
gnome-terminal --title="Fast-LIO Localization" -- bash -c "
source /home/ysc/lite_cog/slam/devel/setup.bash
roslaunch faster_lio localization_c16.launch rviz:=false
read -p 'Press any key to exit...'
"

sleep 3

# 启动导航系统（在新终端）
gnome-terminal --title="FastLIO Navigation" -- bash -c "
source /home/ysc/lite_cog/nav/devel/setup.bash
roslaunch fastlio_local_nav fastlio_local_nav.launch \
    map_name:=$MAP_NAME \
    init_x:=$INIT_X \
    init_y:=$INIT_Y \
    init_yaw:=$INIT_YAW \
    open_rviz:=true
read -p 'Press any key to exit...'
"

echo "启动完成！"
