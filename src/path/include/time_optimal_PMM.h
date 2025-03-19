#ifndef TIME_OPTIMAL_PMM_H
#define TIME_OPTIMAL_PMM_H

#include <iostream>
#include <cmath>
#include <stdexcept>
#include <tuple>
#include <matplotlibcpp.h>
#include <Eigen/Core>
#include <geometry_msgs/Vector3.h>
#define EPS 1e-4

namespace plt = matplotlibcpp;
using namespace geometry_msgs;



class TimeOptimalPMM {
public:
    TimeOptimalPMM(double x0, double v0, double xT, double vT, double umax, double umin, double vmax = std::numeric_limits<double>::infinity(), double vmin = -std::numeric_limits<double>::infinity())
        : x0(x0), v0(v0), xT(xT), vT(vT), umax(umax), umin(umin), vmax(vmax), vmin(vmin), original_umax(umax), original_umin(umin) {
            alpha = 1.0;
        }

    std::tuple<double, double, double, int> compute_times();
    void plot_trajectory();

// private:
    double x0, v0, xT, vT, umax, umin, vmax, vmin, original_umax, original_umin, alpha, t1_, t2_, T_;
    int case_idx_;
    std::tuple<double, double, double> compute_case1_2();
    std::tuple<double, double, double> compute_case3();
    std::tuple<double, double, double> compute_case4_5();
    std::tuple<double, double, double> compute_case6();
    double find_alpha(double T_target, double tol, int max_iter);
};


class TimeOptimalPMM3D {
public:
    int num_points = 1000;
    double T;
    TimeOptimalPMM pmm_x, pmm_y, pmm_z;
    TimeOptimalPMM3D(Vector3 x0, Vector3 v0, Vector3 xT, Vector3 vT, Vector3 umax, Vector3 umin, Vector3 vmax, Vector3 vmin) : pmm_x(x0.x, v0.x, xT.x, vT.x, umax.x, umin.x, vmax.x, vmin.x), 
                    pmm_y(x0.y, v0.y, xT.y, vT.y, umax.y, umin.y, vmax.y, vmin.y),
                    pmm_z(x0.z, v0.z, xT.z, vT.z, umax.z, umin.z, vmax.z, vmin.z) {}
    std::tuple<Vector3, Vector3, double, Vector3> compute_times();
    void plot_combined_trajectory();
    void plot_initial_trajectory();
    void plot_trajectory();
    bool isEqualFloat(double a, double b);
};

#endif // TIME_OPTIMAL_PMM_H