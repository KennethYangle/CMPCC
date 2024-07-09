#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <cmath>

class Utils {
public:
    double yaw_control(double target_yaw, double feb_yaw, double kp_yaw, double sat_yaw);
    double minAngleDiff(double a, double b);
    double Saturation(double a, double up);
};

#endif // UTILS_H