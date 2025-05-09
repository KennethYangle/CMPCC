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
    } else {
        // Phase 1, [0, t1_]
        for (int i = 0; i < num_points; ++i) {
            t_all.push_back(t1_ * i / (num_points - 1));
            u_all.push_back(case_idx_ == 0 ? umax : -umin);
        }
        // Phase 2, [t1_, t2_]
        for (int i = 0; i < num_points; ++i) {
            t_all.push_back(t1_ + (t2_ - t1_) * i / (num_points - 1));
            u_all.push_back(0);
        }
        // Phase 3, [t2_, T_]
        for (int i = 0; i < num_points; ++i) {
            t_all.push_back(t2_ + (T_ - t2_) * i / (num_points - 1));
            u_all.push_back(case_idx_ == 0 ? -umin : umax);
        }

        // calc position and velocity
        x_all.push_back(x0);
        v_all.push_back(v0);
        for (int i = 1; i < t_all.size(); ++i) {
            double dt = t_all[i] - t_all[i - 1];
            v_all.push_back(v_all[i - 1] + u_all[i - 1] * dt);
            x_all.push_back(x_all[i - 1] + v_all[i - 1] * dt + 0.5 * u_all[i - 1] * dt * dt);
        }
    }

    plt::figure_size(400, 700);

    // --- Position Plot ---
    plt::subplot(3, 1, 1);
    plt::plot(t_all, x_all);
    plt::ylabel("p (m)"); //keep consist label

    // --- Velocity Plot ---
    plt::subplot(3, 1, 2);
    plt::plot(t_all, v_all);
    plt::ylabel("v (m/s)"); //keep consist label

    // Add velocity bounds
    plt::axhline(vmax, 0.0, 1.0, {{"color", "black"}, {"linestyle", "--"}});
    plt::axhline(-vmin, 0.0, 1.0, {{"color", "red"}, {"linestyle", "--"}});
    plt::fill_between(t_all, std::vector<double>(t_all.size(), vmax), std::vector<double>(t_all.size(), 1.5*vmax), {{"alpha", "0.2"}, {"color", "peachpuff"}});
    plt::fill_between(t_all, std::vector<double>(t_all.size(), -vmin), std::vector<double>(t_all.size(), -1.5*vmin), {{"alpha", "0.2"}, {"color", "lightsteelblue"}});

    // --- Acceleration Plot ---
    plt::subplot(3, 1, 3);
    plt::plot(t_all, u_all);
    plt::xlabel("t (s)"); // xlabel only on the bottom plot
    plt::ylabel("u (m/s$^2$)"); //keep consist label

    // Add acceleration bounds
    plt::axhline(umax, 0.0, 1.0, {{"color", "black"}, {"linestyle", "--"}});
    plt::axhline(-umin, 0.0, 1.0, {{"color", "red"}, {"linestyle", "--"}});
    plt::fill_between(t_all, std::vector<double>(t_all.size(), umax), std::vector<double>(t_all.size(), 1.5*umax), {{"alpha", "0.2"}, {"color", "peachpuff"}});
    plt::fill_between(t_all, std::vector<double>(t_all.size(), -umin), std::vector<double>(t_all.size(), -1.5*umin), {{"alpha", "0.2"}, {"color", "lightsteelblue"}});

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
            // std::cout << T << std::endl;
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
    // std::cout << "T: " << T << ", T_x: " << T_x << ", T_y: " << T_y << ", T_z: " << T_z << std::endl;

    // Find the alpha value for the remaining two axes
    double alpha_x = 1.0, alpha_y = 1.0, alpha_z = 1.0;
    if (T_x < T)
        alpha_x = pmm_x.find_alpha(T);
    // std::cout << "alpha_x: " << pmm_x.alpha << ", t1: " << pmm_x.t1_ << ", t2: " << pmm_x.t2_ << ", T: " << pmm_x.T_ << std::endl;

    if (T_y < T)
        alpha_y = pmm_y.find_alpha(T);
    // std::cout << "alpha_y: " << pmm_y.alpha << ", t1: " << pmm_y.t1_ << ", t2: " << pmm_y.t2_ << ", T: " << pmm_y.T_ << std::endl;

    if (T_z < T)
        alpha_z = pmm_z.find_alpha(T);
    // std::cout << "alpha_z: " << pmm_z.alpha << ", t1: " << pmm_z.t1_ << ", t2: " << pmm_z.t2_ << ", T: " << pmm_z.T_ << std::endl;

    Vector3 t1_vec, t2_vec, case_idx_vec;
    t1_vec.x = pmm_x.t1_; t1_vec.y = pmm_y.t1_; t1_vec.z = pmm_z.t1_;
    t2_vec.x = pmm_x.t2_; t2_vec.y = pmm_y.t2_; t2_vec.z = pmm_z.t2_;
    case_idx_vec.x = pmm_x.case_idx_; case_idx_vec.y = pmm_y.case_idx_; case_idx_vec.z = pmm_z.case_idx_;
    return std::make_tuple(t1_vec, t2_vec, T, case_idx_vec);
}

#include "time_optimal_PMM.h" // Make sure this header includes <cmath>, <tuple>, <vector>, <limits>, <algorithm>, and matplotlibcpp.h
#include <iostream>

void TimeOptimalPMM3D::plot_combined_trajectory() {
    // --- 1. Calculate Trajectories (Similar to plot_trajectory, but storing data) ---

    std::vector<double> t_all_x, x_all_x, v_all_x, u_all_x;
    std::vector<double> t_all_y, x_all_y, v_all_y, u_all_y;
    std::vector<double> t_all_z, x_all_z, v_all_z, u_all_z;

    // Helper function to generate trajectory data for a single axis
    auto generate_axis_data = [&](TimeOptimalPMM& pmm, std::vector<double>& t_all, std::vector<double>& x_all,
                                  std::vector<double>& v_all, std::vector<double>& u_all) {
        double v_line = (pmm.xT - pmm.x0) / pmm.T_;
        double x = pmm.x0;
        double v = pmm.v0;
        double u = 0;
        t_all.clear(); x_all.clear(); v_all.clear(); u_all.clear(); // Clear vectors

        if (pmm.case_idx_ == 2) {
            for (int i = 0; i < num_points; ++i) {
                double t = pmm.T_ * i / (num_points - 1);
                t_all.push_back(t);
                x_all.push_back(pmm.x0 + t * v_line);
                v_all.push_back(v_line);
                u_all.push_back(0);
            }
        } else {
            // Phase 1
            for (int i = 0; i < num_points; ++i) {
                double t = pmm.t1_ * i / (num_points - 1);
                t_all.push_back(t);
                u_all.push_back(pmm.case_idx_ == 0 ? pmm.umax : -pmm.umin);
            }
            // Phase 2
            for (int i = 0; i < num_points; ++i) {
                double t = pmm.t1_ + (pmm.t2_ - pmm.t1_) * i / (num_points - 1);
                t_all.push_back(t);
                u_all.push_back(0);
            }

            // Phase 3
             for (int i = 0; i < num_points; ++i) {
                double t = pmm.t2_ + (pmm.T_ - pmm.t2_) * i / (num_points - 1);
                t_all.push_back(t);
                u_all.push_back(pmm.case_idx_ == 0 ? -pmm.umin : pmm.umax);
            }


            x_all.push_back(pmm.x0);
            v_all.push_back(pmm.v0);
            for (int i = 1; i < t_all.size(); ++i) {
                double dt = t_all[i] - t_all[i - 1];
                v = v_all[i - 1] + u_all[i - 1] * dt;
                x = x_all[i - 1] + v_all[i-1] * dt + 0.5 * u_all[i-1] * dt*dt;
                v_all.push_back(v);
                x_all.push_back(x);
            }
        }
    };

    // Generate data for each axis
    generate_axis_data(pmm_x, t_all_x, x_all_x, v_all_x, u_all_x);
    generate_axis_data(pmm_y, t_all_y, x_all_y, v_all_y, u_all_y);
    generate_axis_data(pmm_z, t_all_z, x_all_z, v_all_z, u_all_z);


    // --- 2. Create 3x3 Subplot ---

    plt::figure_size(1200, 800); // Adjust as needed

    // --- X-Axis Plots ---
    plt::subplot(3, 3, 1);
    plt::plot(t_all_x, x_all_x);
    plt::ylabel("p (m)");

    plt::subplot(3, 3, 4);
    plt::plot(t_all_x, v_all_x);
    plt::ylabel("v (m/s)");
    plt::axhline(pmm_x.vmax, 0.0, 1.0, {{"color", "black"}, {"linestyle", "--"}});
    plt::axhline(-pmm_x.vmin, 0.0, 1.0, {{"color", "red"}, {"linestyle", "--"}});
    // Add fill_between for velocity bounds (X-axis)
    plt::fill_between(t_all_x, std::vector<double>(t_all_x.size(), pmm_x.vmax), std::vector<double>(t_all_x.size(), 1.5*pmm_x.vmax), {{"alpha", "0.2"}, {"color", "peachpuff"}});
    plt::fill_between(t_all_x, std::vector<double>(t_all_x.size(), -pmm_x.vmin), std::vector<double>(t_all_x.size(), -1.5*pmm_x.vmin), {{"alpha", "0.2"}, {"color", "lightsteelblue"}});


    plt::subplot(3, 3, 7);
    plt::plot(t_all_x, u_all_x);
    plt::ylabel("u (m/s$^2$)");
    plt::xlabel("t (s)");
    plt::axhline(pmm_x.umax, 0.0, 1.0, {{"color", "black"}, {"linestyle", "--"}});
    plt::axhline(-pmm_x.umin, 0.0, 1.0, {{"color", "red"}, {"linestyle", "--"}});
    // Add fill_between for acceleration bounds (X-axis)
    plt::fill_between(t_all_x, std::vector<double>(t_all_x.size(), pmm_x.umax), std::vector<double>(t_all_x.size(), 1.5*pmm_x.umax), {{"alpha", "0.2"}, {"color", "peachpuff"}});
    plt::fill_between(t_all_x, std::vector<double>(t_all_x.size(), -pmm_x.umin), std::vector<double>(t_all_x.size(), -1.5*pmm_x.umin), {{"alpha", "0.2"}, {"color", "lightsteelblue"}});



    // --- Y-Axis Plots ---
    plt::subplot(3, 3, 2);
    plt::plot(t_all_y, x_all_y);
    // plt::ylabel("$p_y$ (m)");

    plt::subplot(3, 3, 5);
    plt::plot(t_all_y, v_all_y);
    // plt::ylabel("$v_y$ (m/s)");
    plt::axhline(pmm_y.vmax, 0.0, 1.0, {{"color", "black"}, {"linestyle", "--"}});
    plt::axhline(-pmm_y.vmin, 0.0, 1.0, {{"color", "red"}, {"linestyle", "--"}});
    // Add fill_between for velocity bounds (Y-axis)
    plt::fill_between(t_all_y, std::vector<double>(t_all_y.size(), pmm_y.vmax), std::vector<double>(t_all_y.size(), 1.5*pmm_y.vmax), {{"alpha", "0.2"}, {"color", "peachpuff"}});
    plt::fill_between(t_all_y, std::vector<double>(t_all_y.size(), -pmm_y.vmin), std::vector<double>(t_all_y.size(), -1.5*pmm_y.vmin), {{"alpha", "0.2"}, {"color", "lightsteelblue"}});



    plt::subplot(3, 3, 8);
    plt::plot(t_all_y, u_all_y);
    // plt::ylabel("$u_y$ (m/s$^2$)");
    plt::xlabel("t (s)");
    plt::axhline(pmm_y.umax, 0.0, 1.0, {{"color", "black"}, {"linestyle", "--"}});
    plt::axhline(-pmm_y.umin, 0.0, 1.0, {{"color", "red"}, {"linestyle", "--"}});
     // Add fill_between for acceleration bounds (Y-axis)
    plt::fill_between(t_all_y, std::vector<double>(t_all_y.size(), pmm_y.umax), std::vector<double>(t_all_y.size(), 1.5*pmm_y.umax), {{"alpha", "0.2"}, {"color", "peachpuff"}});
    plt::fill_between(t_all_y, std::vector<double>(t_all_y.size(), -pmm_y.umin), std::vector<double>(t_all_y.size(), -1.5*pmm_y.umin), {{"alpha", "0.2"}, {"color", "lightsteelblue"}});


    // --- Z-Axis Plots ---
    plt::subplot(3, 3, 3);
    plt::plot(t_all_z, x_all_z);
    // plt::ylabel("$p_z$ (m)");

    plt::subplot(3, 3, 6);
    plt::plot(t_all_z, v_all_z);
    // plt::ylabel("$v_z$ (m/s)");
    plt::axhline(pmm_z.vmax, 0.0, 1.0, {{"color", "black"}, {"linestyle", "--"}});
    plt::axhline(-pmm_z.vmin, 0.0, 1.0, {{"color", "red"}, {"linestyle", "--"}});
    // Add fill_between for velocity bounds (Z-axis)
    plt::fill_between(t_all_z, std::vector<double>(t_all_z.size(), pmm_z.vmax), std::vector<double>(t_all_z.size(), 1.5*pmm_z.vmax), {{"alpha", "0.2"}, {"color", "peachpuff"}});
    plt::fill_between(t_all_z, std::vector<double>(t_all_z.size(), -pmm_z.vmin), std::vector<double>(t_all_z.size(), -1.5*pmm_z.vmin), {{"alpha", "0.2"}, {"color", "lightsteelblue"}});


    plt::subplot(3, 3, 9);
    plt::plot(t_all_z, u_all_z);
    // plt::ylabel("$u_z$ (m/s$^2$)");
    plt::xlabel("t (s)");
    plt::axhline(pmm_z.umax, 0.0, 1.0, {{"color", "black"}, {"linestyle", "--"}});
    plt::axhline(-pmm_z.umin, 0.0, 1.0, {{"color", "red"}, {"linestyle", "--"}});
    // Add fill_between for acceleration bounds (Z-axis)
    plt::fill_between(t_all_z, std::vector<double>(t_all_z.size(), pmm_z.umax), std::vector<double>(t_all_z.size(), 1.5*pmm_z.umax), {{"alpha", "0.2"}, {"color", "peachpuff"}});
    plt::fill_between(t_all_z, std::vector<double>(t_all_z.size(), -pmm_z.umin), std::vector<double>(t_all_z.size(), -1.5*pmm_z.umin), {{"alpha", "0.2"}, {"color", "lightsteelblue"}});


    plt::tight_layout(); // Prevent overlapping labels
    plt::show();
}

void TimeOptimalPMM3D::plot_initial_trajectory() {
    // Calculate time for each axis
    auto [t1_x, t2_x, T_x, _x] = pmm_x.compute_times();
    auto [t1_y, t2_y, T_y, _y] = pmm_y.compute_times();
    auto [t1_z, t2_z, T_z, _z] = pmm_z.compute_times();

    // // plot each axis
    // pmm_x.plot_trajectory();
    // pmm_y.plot_trajectory();
    // pmm_z.plot_trajectory();

    // plot 3x3 combined trajectory
    plot_combined_trajectory();
}

void TimeOptimalPMM3D::plot_trajectory() {
    // // plot each axis
    // pmm_x.plot_trajectory();
    // pmm_y.plot_trajectory();
    // pmm_z.plot_trajectory();

    // plot 3x3 combined trajectory
    plot_combined_trajectory();

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
    plt::xlabel("x (m)");
    plt::ylabel("y (m)");
    plt::set_zlabel("z (m)"); // set_zlabel rather than just zlabel, in accordance with the Axes3D method
    plt::show();
}

// return true if a == b in double, else false
bool TimeOptimalPMM3D::isEqualFloat(double a, double b) {
    return fabs(a - b) < EPS ? true : false;
}