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

mav_id=1
mav_num=2




# Workspace path
SH_DIR=$(dirname $0)
WS_DIR=$(realpath "${SH_DIR}/..")
echo "Workspace Directory: ${WS_DIR}"

# RflySim仿真参数
UE4IP="192.168.110.145"
# 硬件在环仿真和实飞为true，软件在环仿真为false
USE_PIX=false

roscore &
sleep 2s


if [ $ISFLIGHT = 0 ]
then
    # 载入相机和滤波器参数
    gnome-terminal --tab -t "Load Camera & Filter Param" -- bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch params load_param_sim.launch;exec bash"

    # mavros
    gnome-terminal --tab -t "Mavros" -- bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch offboard_pkg mavros_sim.launch use_pix:=${USE_PIX};exec bash"
    sleep 2s

    # RflySim 接口
    gnome-terminal --tab -t "RflySim Obj interface" -- bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch rflysim_ros_pkg obj.launch ip:=${UE4IP};exec bash"
    sleep 0.5s
    gnome-terminal --tab -t "RflySim Image interface" -- bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch rflysim_ros_pkg rgb_newprotocol_cpp.launch;exec bash"

    # 仿真气球运动
    gnome-terminal --tab -t "Sim Balloon Motion" -- bash -c "source ${WS_DIR}/devel/setup.bash;rosrun path sim_balloon_node _balloon_motion_file:=${WS_DIR}/src/params/balloon_motion.yaml;exec bash"
    sleep 0.5s

    # 目标检测
    gnome-terminal --tab -t "Detection in Sim" -- bash -c "source ${WS_DIR}/devel/setup.bash;rosrun detection color_det;exec bash"
    sleep 1s
else
    # 载入相机和滤波器参数
    gnome-terminal --tab -t "Load Camera & Filter Param" -- bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch params load_param_real.launch;exec bash"

    # mavros
    gnome-terminal --tab -t "Mavros" -- bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0:57600";exec bash"
    sleep 2s

    ## DDS
    gnome-terminal -x bash -c "source ${HOME}/CMPCC/devel/setup.bash; roslaunch bs_assis bs_dds.launch  mav_id:=${mav_id} mav_num:=${mav_num}; exec bash"
    sleep 2s

    # 目标检测
    gnome-terminal -x bash -c "source ${WS_DIR}/devel/setup.bash; roslaunch yolo action4.launch; exec bash"
    sleep 10s
    gnome-terminal -x bash -c "source ${WS_DIR}/devel/setup.bash; roslaunch yolo det_yolo.launch; exec bash"
    sleep 10s
fi

# # 目标运动估计
# gnome-terminal --tab -t "MCL" -- bash -c "source ${WS_DIR}/devel/setup.bash;rosrun mcl mcl_node;exec bash"
# sleep 1s

# Rviz可视化，可选
gnome-terminal --tab -t "Rviz" -- bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch cmpcc rviz_view.launch;exec bash"
sleep 2s

# 路径、轨迹规划 + MPCC控制
gnome-terminal --tab -t "Path Finding" -- bash -c "source ${WS_DIR}/devel/setup.bash;rosrun path path_finding_node;exec bash"
sleep 1s
gnome-terminal --tab -t "Path Planning" -- bash -c "source ${WS_DIR}/devel/setup.bash;rosrun path path_planning_node;exec bash"
sleep 1s
gnome-terminal --tab -t "MPCC" -- bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch cmpcc pva_chasing.launch;exec bash"
sleep 2s

# 拦截控制
gnome-terminal --tab -t "Attack" -- bash -c "source ${WS_DIR}/devel/setup.bash;rosrun offboard_pkg attack.py;exec bash"
sleep 1s

# mavros控制汇总
gnome-terminal --tab -t "Offboard Main Node" -- bash -c "source ${WS_DIR}/devel/setup.bash;rosrun offboard_pkg main_node _ISFLIGHT:=${ISFLIGHT} _MODE:=all;exec bash"
sleep 3s

# 接收激光雷达检测
gnome-terminal --tab -t "Receive Balloon From Lidar" -- bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch path receive_balloon.launch;exec bash"
sleep 0.5s


# 录制bag，保存到/home/nvidia/record/
gnome-terminal --tab -t "ROSbag Record" -- bash -c "rosbag record --split --size=512 /attack_cmd /camera/image_raw/compressed /balloons/masspoint /diagnostics /drone_1/balloons/masspoint /drone_1/mavros/local_position/pose /drone_1/mavros/local_position/velocity_local /drone_2/balloons/masspoint /drone_2/mavros/local_position/pose /drone_2/mavros/local_position/velocity_local /camera/image_raw/compressed /mavros/state /mavros/global_position/global /mavros/imu/data /mavros/local_position/pose /mavros/local_position/velocity_local /mavros/rc/in /mavros/setpoint_raw/local /mavros/setpoint_position/local /mavros/setpoint_velocity/cmd_vel /mavros/home_position/home /tracker/pos_image /path_points  -o /home/nvidia/record/all;exec bash"
sleep 5s