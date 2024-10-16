#include "time_optimal_PMM.h"


std::tuple<double, double, double, int> TimeOptimalPMM::compute_times() {
    auto cases = { &TimeOptimalPMM::compute_case1_2, &TimeOptimalPMM::compute_case4_5 };
    double best_T = std::numeric_limits<double>::infinity();
    std::tuple<double, double, double, int> best_times;

    for (int case_idx = 0; case_idx < cases.size(); ++case_idx) {
        auto case_func = cases.begin();
        std::advance(case_func, case_idx);
        try {
            auto [t1, t2, T] = (this->**case_func)();
            if (0 <= t1 && t1 <= t2 && t2 <= T && T < best_T && T > 0) {
                best_T = T;
                best_times = std::make_tuple(t1, t2, best_T, case_idx);
            }
        } catch (const std::exception&) {
            continue;
        }
    }

    // Not find
    if (best_T == std::numeric_limits<double>::infinity()) {
        t1_ = 0;
        t2_ = 0;
        T_ = std::max((xT-x0)/vmax, (x0-xT)/vmin);
        case_idx_ = 2;
        best_times = std::make_tuple(t1_, t2_, T_, case_idx_);
        return best_times;
    }

    t1_ = std::get<0>(best_times);
    t2_ = std::get<1>(best_times);
    T_  = std::get<2>(best_times);
    case_idx_ = std::get<3>(best_times);
    return best_times;
}

void TimeOptimalPMM::plot_trajectory() {
    std::vector<double> t_all, x_all, v_all, u_all;
    int num_points = 100;

    if (case_idx_ == 2) {
        double v_line = (xT - x0) / T_;
        for (int i = 0; i < num_points; ++i) {
            t_all.push_back(T_ * i / (num_points - 1));
            x_all.push_back(x0 + t_all[i] * v_line);
            v_all.push_back(v_line);
            u_all.push_back(0);
        }
    }

    else {
        // Phase 1, [0, t1_]
        for (int i = 0; i < num_points; ++i) {
            t_all.push_back(t1_ * i / (num_points - 1));
            u_all.push_back( case_idx_==0?umax:-umin );
        }
        // Phase 2, [t1_, t2_]
        for (int i = 0; i < num_points; ++i) {
            t_all.push_back(t1_ + (t2_ - t1_) * i / (num_points - 1));
            u_all.push_back( 0 );
        }
        // Phase 3, [t2_, T_]
        for (int i = 0; i < num_points; ++i) {
            t_all.push_back(t2_ + (T_ - t2_) * i / (num_points - 1));
            u_all.push_back( case_idx_==0?-umin:umax );
        }

        // calc position and velocity
        x_all.push_back(x0);
        v_all.push_back(v0);
        for (int i = 1; i < t_all.size(); ++i) {
            double dt = t_all[i] - t_all[i-1];
            v_all.push_back(v_all[i-1] + u_all[i-1] * dt);
            x_all.push_back(x_all[i-1] + v_all[i-1] * dt + 0.5 * u_all[i-1] * dt * dt);
        }
    }

    plt::figure_size(1200, 900);

    plt::subplot(3, 1, 1);
    plt::plot(t_all, x_all, { {"label", "Position"} });
    plt::xlabel("Time");
    plt::ylabel("Position");
    plt::title("Time-Optimal Trajectory: Position vs Time");
    plt::grid(true);
    plt::legend();

    plt::subplot(3, 1, 2);
    plt::plot(t_all, v_all, { {"label", "Velocity"}, {"color", "red"} });
    plt::xlabel("Time");
    plt::ylabel("Velocity");
    plt::title("Time-Optimal Trajectory: Velocity vs Time");
    plt::grid(true);
    plt::legend();

    plt::subplot(3, 1, 3);
    plt::plot(t_all, u_all, { {"label", "Acceleration"}, {"color", "green"} });
    plt::xlabel("Time");
    plt::ylabel("Acceleration");
    plt::title("Time-Optimal Trajectory: Acceleration vs Time");
    plt::grid(true);
    plt::legend();

    plt::tight_layout();
    plt::show();
}

std::tuple<double, double, double> TimeOptimalPMM::compute_case1_2() {
    double sqrt_term = std::sqrt((umax + umin) * (v0 * v0 * umin + vT * vT * umax - 2 * umax * umin * x0 + 2 * umax * umin * xT));
    double t1_plus = (-v0 * (umax + umin) + sqrt_term) / (umax * (umax + umin));
    double t1_minus = (-v0 * (umax + umin) - sqrt_term) / (umax * (umax + umin));
    double t1 = (t1_minus > 0) ? t1_minus : t1_plus;
    double t2 = t1;
    double T = ((umax + umin) * t1 + (v0 - vT)) / umin;
    // std::cout << T << std::endl;

    if (vmax != std::numeric_limits<double>::infinity() && (v0 + umax * t1) > vmax) {
        return compute_case3();
    }
    return std::make_tuple(t1, t2, T);
}

std::tuple<double, double, double> TimeOptimalPMM::compute_case3() {
    double t1 = (vmax - v0) / umax;
    double t2 = (umin * v0 * v0 - 2 * umin * v0 * vmax - umax * vmax * vmax + umin * vmax * vmax + umax * vT * vT - 2 * umax * umin * x0 + 2 * umax * umin * xT) / (2 * umax * umin * vmax);
    double T = (v0 - vT + umax * t1) / umin + t2;
    return std::make_tuple(t1, t2, T);
}

std::tuple<double, double, double> TimeOptimalPMM::compute_case4_5() {
    double sqrt_term = std::sqrt((umax + umin) * (v0 * v0 * umax + vT * vT * umin + 2 * umin * umax * x0 - 2 * umin * umax * xT));
    double t1_plus = (v0 * (umax + umin) + sqrt_term) / (umin * (umax + umin));
    double t1_minus = (v0 * (umax + umin) - sqrt_term) / (umin * (umax + umin));
    double t1 = (t1_minus > 0) ? t1_minus : t1_plus;
    double t2 = t1;
    double T = ((umax + umin) * t1 + (vT - v0)) / umax;
    // std::cout << T << std::endl;

    if (vmin != -std::numeric_limits<double>::infinity() && (v0 - umin * t1) < -vmin) {
        return compute_case6();
    }
    return std::make_tuple(t1, t2, T);
}

std::tuple<double, double, double> TimeOptimalPMM::compute_case6() {
    double t1 = (v0 + vmin) / umin;
    double t2 = (umax * v0 * v0 + 2 * umax * v0 * vmin + umax * vmin * vmin - umin * vmin * vmin + umin * vT * vT + 2 * umax * umin * x0 - 2 * umax * umin * xT) / (2 * umax * umin * vmin);
    double T = (vT - v0 + umin * t1) / umax + t2;
    return std::make_tuple(t1, t2, T);
}

double TimeOptimalPMM::find_alpha(double T_target, double tol = 1e-2, int max_iter = 20) {
    double alpha_min = 0.0;
    double alpha_max = 1.0;

    for (int iter = 0; iter < max_iter; ++iter) {
        double alpha_mid = (alpha_min + alpha_max) / 2.0;
        umax = original_umax * alpha_mid;
        umin = original_umin * alpha_mid;
        // std::cout << "alpha_mid: " << alpha_mid << std::endl;

        auto [_, __, T, case_idx_] = compute_times();
        if (case_idx_ < 1.5) {      // case_idx_ == 0 or case_idx_ == 1
            std::cout << T << std::endl;
            if (T > T_target)
                alpha_min = alpha_mid;
            else
                alpha_max = alpha_mid;
        }
        else {
            alpha_min = alpha_mid;
        }

        if (fabs(T_ - T_target) < tol) {
            alpha = (alpha_min + alpha_max) / 2.0;
            return alpha;
        }

    }

    case_idx_ = 2;
    T_ = T_target;
    return -1;
}

std::tuple<Vector3, Vector3, double, Vector3> TimeOptimalPMM3D::compute_times() {
    // Calculate time for each axis
    auto [t1_x, t2_x, T_x, _x] = pmm_x.compute_times();
    auto [t1_y, t2_y, T_y, _y] = pmm_y.compute_times();
    auto [t1_z, t2_z, T_z, _z] = pmm_z.compute_times();

    // Determine the maximum minimum time
    T = std::max({T_x, T_y, T_z});
    std::cout << "T: " << T << ", T_x: " << T_x << ", T_y: " << T_y << ", T_z: " << T_z << std::endl;

    // Find the alpha value for the remaining two axes
    double alpha_x = 1.0, alpha_y = 1.0, alpha_z = 1.0;
    if (T_x < T)
        alpha_x = pmm_x.find_alpha(T);
    std::cout << "alpha_x: " << pmm_x.alpha << ", t1: " << pmm_x.t1_ << ", t2: " << pmm_x.t2_ << ", T: " << pmm_x.T_ << std::endl;

    if (T_y < T)
        alpha_y = pmm_y.find_alpha(T);
    std::cout << "alpha_y: " << pmm_y.alpha << ", t1: " << pmm_y.t1_ << ", t2: " << pmm_y.t2_ << ", T: " << pmm_y.T_ << std::endl;

    if (T_z < T)
        alpha_z = pmm_z.find_alpha(T);
    std::cout << "alpha_z: " << pmm_z.alpha << ", t1: " << pmm_z.t1_ << ", t2: " << pmm_z.t2_ << ", T: " << pmm_z.T_ << std::endl;

    Vector3 t1_vec, t2_vec, case_idx_vec;
    t1_vec.x = pmm_x.t1_; t1_vec.y = pmm_y.t1_; t1_vec.z = pmm_z.t1_;
    t2_vec.x = pmm_x.t2_; t2_vec.y = pmm_y.t2_; t2_vec.z = pmm_z.t2_;
    case_idx_vec.x = pmm_x.case_idx_; case_idx_vec.y = pmm_y.case_idx_; case_idx_vec.z = pmm_z.case_idx_;
    return std::make_tuple(t1_vec, t2_vec, T, case_idx_vec);
}

void TimeOptimalPMM3D::plot_trajectory() {
    // plot each axis
    pmm_x.plot_trajectory();
    pmm_y.plot_trajectory();
    pmm_z.plot_trajectory();

    // plot 3D trajectory
    std::vector<double> t_all;
    std::vector<double> x_all, y_all, z_all;

    double v_line_x = (pmm_x.xT - pmm_x.x0) / pmm_x.T_;
    double v_line_y = (pmm_y.xT - pmm_y.x0) / pmm_y.T_;
    double v_line_z = (pmm_z.xT - pmm_z.x0) / pmm_z.T_;
    double x = pmm_x.x0, y = pmm_y.x0, z = pmm_z.x0;
    double vx = pmm_x.v0, vy = pmm_y.v0, vz = pmm_z.v0;
    double ux = 0, uy = 0, uz = 0;
    // 采1000个点循环
    for (int tt = 0; tt < num_points; tt++) {
        double theta_ = T * tt / (num_points - 1);
        double dt = T / num_points;
        Eigen::Vector3d thetaPoint, thetaDot, thetaDotDot;

        // x轴
        if (isEqualFloat(pmm_x.case_idx_, 2)) {
            thetaPoint[0] = pmm_x.x0 + theta_ * v_line_x;
            thetaDot[0] = v_line_x;
            thetaDotDot[0] = 0;
        }
        else {
            x = x + vx * dt + 0.5 * ux * dt * dt;
            vx = vx + ux * dt;
            if (isEqualFloat(pmm_x.case_idx_, 0)) {
                //                        phase 1,                          phase 2, phase 3
                ux = theta_ < pmm_x.t1_ ? pmm_x.umax : (theta_ < pmm_x.t2_ ? 0 : -pmm_x.umin);
            }
            else {
                ux = theta_ < pmm_x.t1_ ? -pmm_x.umin : (theta_ < pmm_x.t2_ ? 0 : pmm_x.umax);
            }
            thetaPoint[0] = x;
            thetaDot[0] = vx;
            thetaDotDot[0] = ux;
        }
        // y轴
        if (isEqualFloat(pmm_y.case_idx_, 2)) {
            thetaPoint[1] = pmm_y.x0 + theta_ * v_line_y;
            thetaDot[1] = v_line_y;
            thetaDotDot[1] = 0;
        }
        else {
            y = y + vy * dt + 0.5 * uy * dt * dt;
            vy = vy + uy * dt;
            if (isEqualFloat(pmm_y.case_idx_, 0)) {
                uy = theta_ < pmm_y.t1_ ? pmm_y.umax : (theta_ < pmm_y.t2_ ? 0 : -pmm_y.umin);
            }
            else {
                uy = theta_ < pmm_y.t1_ ? -pmm_y.umin : (theta_ < pmm_y.t2_ ? 0 : pmm_y.umax);
            }
            thetaPoint[1] = y;
            thetaDot[1] = vy;
            thetaDotDot[1] = uy;
        }
        // z轴
        if (isEqualFloat(pmm_z.case_idx_, 2)) {
            thetaPoint[2] = pmm_z.x0 + theta_ * v_line_z;
            thetaDot[2] = v_line_z;
            thetaDotDot[2] = 0;
        }
        else {
            z = z + vz * dt + 0.5 * uz * dt * dt;
            vz = vz + uz * dt;
            if (isEqualFloat(pmm_z.case_idx_, 0)) {
                uz = theta_ < pmm_z.t1_ ? pmm_z.umax : (theta_ < pmm_z.t2_ ? 0 : -pmm_z.umin);
            }
            else {
                uz = theta_ < pmm_z.t1_ ? -pmm_z.umin : (theta_ < pmm_z.t2_ ? 0 : pmm_z.umax);
            }
            thetaPoint[2] = z;
            thetaDot[2] = vz;
            thetaDotDot[2] = uz;
        }

        t_all.push_back(theta_);
        x_all.push_back(thetaPoint[0]);
        y_all.push_back(thetaPoint[1]);
        z_all.push_back(thetaPoint[2]);
    }

    plt::plot3(x_all, y_all, z_all);
    plt::xlabel("x label");
    plt::ylabel("y label");
    plt::set_zlabel("z label"); // set_zlabel rather than just zlabel, in accordance with the Axes3D method
    plt::legend();
    plt::show();
}

// return true if a == b in double, else false
bool TimeOptimalPMM3D::isEqualFloat(double a, double b) {
    return fabs(a - b) < EPS ? true : false;
}