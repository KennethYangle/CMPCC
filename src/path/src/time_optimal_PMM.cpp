#include <iostream>
#include <cmath>
#include <stdexcept>
#include <tuple>
#include <matplotlibcpp.h>

namespace plt = matplotlibcpp;

class TimeOptimalPMM {
public:
    TimeOptimalPMM(double x0, double v0, double xT, double vT, double umax, double umin, double vmax = std::numeric_limits<double>::infinity(), double vmin = -std::numeric_limits<double>::infinity())
        : x0(x0), v0(v0), xT(xT), vT(vT), umax(umax), umin(umin), vmax(vmax), vmin(vmin), original_umax(umax), original_umin(umin) {}

    std::tuple<double, double, double, int> compute_times() {
        auto cases = { &TimeOptimalPMM::compute_case1_2, &TimeOptimalPMM::compute_case4_5 };
        double best_T = std::numeric_limits<double>::infinity();
        std::tuple<double, double, double, int> best_times;

        for (int case_idx = 0; case_idx < cases.size(); ++case_idx) {
            auto case_func = cases.begin();
            std::advance(case_func, case_idx);
            try {
                auto [t1, t2, T] = (this->**case_func)();
                if (0 <= t1 && t1 <= t2 && t2 <= T && T < best_T) {
                    best_T = T;
                    best_times = std::make_tuple(t1, t2, T, case_idx);
                }
            } catch (const std::exception&) {
                continue;
            }
        }

        if (std::get<2>(best_times) == std::numeric_limits<double>::infinity()) {
            throw std::runtime_error("No valid time solution found for the given conditions.");
        }

        return best_times;
    }

    void plot_trajectory() {
        auto [t1, t2, T, case_idx] = compute_times();

        std::vector<double> t_all, x_all, v_all, u_all;

        auto add_phase = [&](double t_start, double t_end, auto x_func, auto v_func, auto u_func) {
            int num_points = 100;
            std::vector<double> t_phase(num_points), x_phase(num_points), v_phase(num_points), u_phase(num_points);
            for (int i = 0; i < num_points; ++i) {
                double t = t_start + (t_end - t_start) * i / (num_points - 1);
                t_phase[i] = t;
                x_phase[i] = x_func(t);
                v_phase[i] = v_func(t);
                u_phase[i] = u_func(t);
            }
            t_all.insert(t_all.end(), t_phase.begin(), t_phase.end());
            x_all.insert(x_all.end(), x_phase.begin(), x_phase.end());
            v_all.insert(v_all.end(), v_phase.begin(), v_phase.end());
            u_all.insert(u_all.end(), u_phase.begin(), u_phase.end());
        };

        if (case_idx == 0) {
            add_phase(0, t1,
                [&](double t) { return x0 + v0 * t + 0.5 * umax * t * t; },
                [&](double t) { return v0 + umax * t; },
                [&](double) { return umax; });
            add_phase(t1, t2,
                [&](double t) { return x0 + v0 * t1 + 0.5 * umax * t1 * t1 + (v0 + umax * t1) * (t - t1); },
                [&](double) { return v0 + umax * t1; },
                [&](double) { return 0; });
            add_phase(t2, T,
                [&](double t) { return xT - vT * (T - t) - 0.5 * umin * (T - t) * (T - t); },
                [&](double t) { return vT + umin * (T - t); },
                [&](double) { return -umin; });
        } else if (case_idx == 1) {
            add_phase(0, t1,
                [&](double t) { return x0 + v0 * t - 0.5 * umin * t * t; },
                [&](double t) { return v0 - umin * t; },
                [&](double) { return -umin; });
            add_phase(t1, t2,
                [&](double t) { return x0 + v0 * t1 - 0.5 * umin * t1 * t1 + (v0 - umin * t1) * (t - t1); },
                [&](double) { return v0 - umin * t1; },
                [&](double) { return 0; });
            add_phase(t2, T,
                [&](double t) { return xT - vT * (T - t) + 0.5 * umax * (T - t) * (T - t); },
                [&](double t) { return vT - umax * (T - t); },
                [&](double) { return umax; });
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

// private:
    double x0, v0, xT, vT, umax, umin, vmax, vmin, original_umax, original_umin;

    std::tuple<double, double, double> compute_case1_2() {
        double sqrt_term = std::sqrt((umax + umin) * (v0 * v0 * umin + vT * vT * umax - 2 * umax * umin * x0 + 2 * umax * umin * xT));
        double t1_plus = (-v0 * (umax + umin) + sqrt_term) / (umax * (umax + umin));
        double t1_minus = (-v0 * (umax + umin) - sqrt_term) / (umax * (umax + umin));
        double t1 = (t1_minus > 0) ? t1_minus : t1_plus;
        double t2 = t1;
        double T = ((umax + umin) * t1 + (v0 - vT)) / umin;

        if (vmax != std::numeric_limits<double>::infinity() && (v0 + umax * t1) > vmax) {
            return compute_case3();
        }
        return std::make_tuple(t1, t2, T);
    }

    std::tuple<double, double, double> compute_case3() {
        double t1 = (vmax - v0) / umax;
        double t2 = (umin * v0 * v0 - 2 * umin * v0 * vmax - umax * vmax * vmax + umin * vmax * vmax + umax * vT * vT - 2 * umax * umin * x0 + 2 * umax * umin * xT) / (2 * umax * umin * vmax);
        double T = (v0 - vT + umax * t1) / umin + t2;
        return std::make_tuple(t1, t2, T);
    }

    std::tuple<double, double, double> compute_case4_5() {
        double sqrt_term = std::sqrt((umax + umin) * (v0 * v0 * umax + vT * vT * umin + 2 * umin * umax * x0 - 2 * umin * umax * xT));
        double t1_plus = (v0 * (umax + umin) + sqrt_term) / (umin * (umax + umin));
        double t1_minus = (v0 * (umax + umin) - sqrt_term) / (umin * (umax + umin));
        double t1 = (t1_minus > 0) ? t1_minus : t1_plus;
        double t2 = t1;
        double T = ((umax + umin) * t1 - (v0 - vT)) / umax;

        if (vmin != -std::numeric_limits<double>::infinity() && (v0 - umin * t1) < vmin) {
            return compute_case6();
        }
        return std::make_tuple(t1, t2, T);
    }

    std::tuple<double, double, double> compute_case6() {
        double t1 = (v0 + vmin) / umin;
        double t2 = (umax * v0 * v0 + 2 * umax * v0 * vmin + umax * vmin * vmin - umin * vmin * vmin + umin * vT * vT + 2 * umax * umin * x0 - 2 * umax * umin * xT) / (2 * umax * umin * vmin);
        double T = (vT - v0 + umin * t1) / umax + t2;
        return std::make_tuple(t1, t2, T);
    }
};

double find_alpha(TimeOptimalPMM& pmm, double T_target, double tol = 1e-3, int max_iter = 100) {
    double alpha_min = 0.0;
    double alpha_max = 1.0;

    for (int iter = 0; iter < max_iter; ++iter) {
        double alpha_mid = (alpha_min + alpha_max) / 2.0;
        pmm.umax = pmm.original_umax * alpha_mid;
        pmm.umin = pmm.original_umin * alpha_mid;

        try {
            auto [_, __, T, _0] = pmm.compute_times();
            std::cout << T << std::endl;
            if (T > T_target)
                alpha_min = alpha_mid;
            else
                alpha_max = alpha_mid;
        } catch (const std::exception&) {
            alpha_min = alpha_mid;
        }

        if (alpha_max - alpha_min < tol) {
            return (alpha_min + alpha_max) / 2.0;
        }

    }

    throw std::runtime_error("Alpha did not converge within the maximum number of iterations.");
}

int main() {
    try {
        // Initial conditions for x, y, z axes
        TimeOptimalPMM pmm_x(0, 0, -8, 3, 12, 12, 7.5, 7.5);
        TimeOptimalPMM pmm_y(0, 0, 7, 6, 12, 12, 7.5, 7.5);
        TimeOptimalPMM pmm_z(0, 0, 2, 0, 2.5, 15, 5, 5);

        // Calculate time for each axis
        std::vector<TimeOptimalPMM> axes = { pmm_x, pmm_y, pmm_z };

        auto [t1_x, t2_x, T_x, _x] = pmm_x.compute_times();
        auto [t1_y, t2_y, T_y, _y] = pmm_y.compute_times();
        auto [t1_z, t2_z, T_z, _z] = pmm_z.compute_times();

        // Determine the maximum minimum time
        double T = std::max({T_x, T_y, T_z});
        std::cout << "T: " << T << ", T_x: " << T_x << ", T_y: " << T_y << ", T_z: " << T_z << std::endl;

        // Find the alpha value for the remaining two axes
        double alpha_x = 1.0, alpha_y = 1.0, alpha_z = 1.0;
        if (T_x < T)
            alpha_x = find_alpha(axes[0], T);
        std::cout << "alpha_x: " << alpha_x << std::endl;

        if (T_y < T)
            alpha_y = find_alpha(axes[1], T);
        std::cout << "alpha_y: " << alpha_y << std::endl;

        if (T_z < T)
            alpha_z = find_alpha(axes[2], T);
        std::cout << "alpha_z: " << alpha_z << std::endl;

        // Plot trajectory for each axis
        for (auto& axis : axes) {
            axis.plot_trajectory();
        }

    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
