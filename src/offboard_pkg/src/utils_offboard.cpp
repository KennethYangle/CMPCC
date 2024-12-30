#include "utils_offboard.h"

// 偏航角控制函数
double Utils::yaw_control(double target_yaw, double feb_yaw, double kp_yaw, double sat_yaw) {
    // 计算目标偏航角与反馈偏航角之间的差值
    double dlt_yaw = minAngleDiff(target_yaw, feb_yaw);
    // 计算控制命令，并应用饱和函数
    double cmd_yaw = Saturation(kp_yaw * dlt_yaw, sat_yaw);
    return cmd_yaw;
}

// 计算两个角度之间的最小差值
double Utils::minAngleDiff(double a, double b) {
    double diff = a - b;
    if (diff < 0) {
        diff += 2 * M_PI; // M_PI是pi的宏定义，需要包含<cmath>
    }
    if (diff < M_PI) {
        return diff;
    } else {
        return diff - 2 * M_PI;
    }
}

// 饱和函数，确保值在上下界之间
double Utils::Saturation(double a, double up) {
    return std::max(-up, std::min(a, up));
}
