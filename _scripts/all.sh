#!/bin/bash

# clean old log
rosclean purge -y

# 判断仿真还是实飞。默认实飞模式
ISFLIGHT=1

parameters=$(getopt -o s --long sim -n "$0" -- "$@")
[ $? != 0 ] && exit 1
eval set -- "$parameters"   # 将$parameters设置为位置参数
while true ; do             # 循环解析位置参数
    case "$1" in
        -s|--sim) ISFLIGHT=0; shift;;      # 不带参数的选项-a或--longa
        --) shift; break;;       # 开始解析非选项类型的参数，break后，它们都保留在$@中
        *) echo "wrong"; exit 1;;
    esac
done

if [ $ISFLIGHT = 0 ]
then
   echo "Simulation Mode !!!"
else
   echo "Real Flight Mode !!!"
fi





# Workspace path
SH_DIR=$(dirname $0)
WS_DIR="${SH_DIR}/.."

# RflySim仿真参数
UE4IP="192.168.110.145"
# 硬件在环仿真和实飞为true，软件在环仿真为false
USE_PIX=false

roscore &
sleep 2s


if [ $ISFLIGHT = 0 ]
then
    # mavros
    gnome-terminal -x bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch offboard_pkg mavros_sim.launch use_pix:=${USE_PIX};exec bash"
    sleep 2s

else
    # mavros
    gnome-terminal -x bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0:115200";exec bash"
    sleep 2s
fi

# Rviz可视化，可选
gnome-terminal -x bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch cmpcc rviz_view.launch;exec bash"
sleep 3s

# 规划+MPCC控制
gnome-terminal -x bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch cmpcc fly.launch;exec bash"
sleep 3s

# mavros控制汇总
gnome-terminal -x bash -c "source ${WS_DIR}/devel/setup.bash;rosrun offboard_pkg main_node;exec bash"
sleep 3s



# gnome-terminal -x bash -c "rosbag record /d400/imu /d400/color/camera_info /d400/aligned_depth_to_color/image_raw /d400/aligned_depth_to_color/camera_info /d400/color/image_raw /camera/odom/sample /mavros/odometry/out /mavros/local_position/odom /MSF/odom/local /MSF/odom/global /mavros/setpoint_velocity/cmd_vel /mapLoc/pose -o /home/nvidia/record;exec bash"
# sleep 5s