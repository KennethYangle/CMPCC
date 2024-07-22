import math
import matplotlib.pyplot as plt
import numpy as np

class TimeOptimalPMM:
    def __init__(self, x0, v0, xT, vT, umax, umin, vmax=None, vmin=None):
        self.x0 = x0
        self.v0 = v0
        self.xT = xT
        self.vT = vT
        self.umax = umax
        self.umin = umin
        self.vmax = vmax
        self.vmin = vmin
        self.original_umax = umax
        self.original_umin = umin

    def compute_case1_2(self):
        sqrt_term = math.sqrt(
            (self.umax + self.umin) * (self.v0**2 * self.umin + self.vT**2 * self.umax - 2 * self.umax * self.umin * self.x0 + 2 * self.umax * self.umin * self.xT)
        )
        t1_plus = (-self.v0 * (self.umax + self.umin) + sqrt_term) / (self.umax * (self.umax + self.umin))
        t1_minus = (-self.v0 * (self.umax + self.umin) - sqrt_term) / (self.umax * (self.umax + self.umin))
        t1 = t1_minus if t1_minus > 0 else t1_plus
        t2 = t1
        T = ((self.umax + self.umin) * t1 + (self.v0 - self.vT)) / self.umin
        
        if self.vmax is not None and (self.v0 + self.umax * t1) > self.vmax:
            return self.compute_case3()
        
        return t1, t2, T

    def compute_case3(self):
        t1 = (self.vmax - self.v0) / self.umax
        t2 = (self.umin * self.v0**2 - 2 * self.umin * self.v0 * self.vmax - self.umax * self.vmax**2 + self.umin * self.vmax**2 + self.umax * self.vT**2 - 2 * self.umax * self.umin * self.x0 + 2 * self.umax * self.umin * self.xT) / (2 * self.umax * self.umin * self.vmax)
        T = (self.v0 - self.vT + self.umax * t1) / self.umin + t2
        return t1, t2, T

    def compute_case4_5(self):
        sqrt_term = math.sqrt(
            (self.umax + self.umin) * (self.v0**2 * self.umax + self.vT**2 * self.umin + 2 * self.umin * self.umax * self.x0 - 2 * self.umin * self.umax * self.xT)
        )
        t1_plus = (self.v0 * (self.umax + self.umin) + sqrt_term) / (self.umin * (self.umax + self.umin))
        t1_minus = (self.v0 * (self.umax + self.umin) - sqrt_term) / (self.umin * (self.umax + self.umin))
        t1 = t1_minus if t1_minus > 0 else t1_plus
        t2 = t1
        T = ((self.umax + self.umin) * t1 + (self.vT - self.v0)) / self.umax
        
        if self.vmin is not None and (self.v0 - self.umin * t1) < -self.vmin:
            return self.compute_case6()
        
        return t1, t2, T

    def compute_case6(self):
        t1 = (self.v0 + self.vmin) / self.umin
        t2 = (self.umax * self.v0**2 + 2 * self.umax * self.v0 * self.vmin + self.umax * self.vmin**2 - self.umin * self.vmin**2 + self.umin * self.vT**2 + 2 * self.umax * self.umin * self.x0 - 2 * self.umax * self.umin * self.xT) / (2 * self.umax * self.umin * self.vmin)
        T = (self.vT - self.v0 + self.umin * t1) / self.umax + t2
        return t1, t2, T

    def compute_times(self):
        cases = [
            self.compute_case1_2,
            self.compute_case4_5
        ]

        best_T = float('inf')
        best_times = None
        
        for case_idx in range(len(cases)):
            case = cases[case_idx]
            try:
                t1, t2, T = case()
                if 0 <= t1 <= t2 <= T and T < best_T:
                    best_T = T
                    best_times = (t1, t2, T, case_idx)
            except (ValueError, ZeroDivisionError, FloatingPointError):
                continue
        
        if best_times is None:
            raise ValueError("No valid time solution found for the given conditions.")
        
        return best_times

    def plot_trajectory(self):
        t1, t2, T, case_idx = self.compute_times()
        
        # Create time arrays for each phase
        t_phase1 = np.linspace(0, t1, num=100)
        t_phase2 = np.linspace(t1, t2, num=100)
        t_phase3 = np.linspace(t2, T, num=100)
        
        # Calculate position and velocity for each phase
        if case_idx == 0:
            x_phase1 = self.x0 + self.v0 * t_phase1 + 0.5 * self.umax * t_phase1**2
            v_phase1 = self.v0 + self.umax * t_phase1
            u_phase1 = np.full_like(t_phase1, self.umax)
            
            x_phase2 = x_phase1[-1] + v_phase1[-1] * (t_phase2 - t1)
            v_phase2 = np.full_like(t_phase2, v_phase1[-1])
            u_phase2 = np.full_like(t_phase2, 0)
            
            x_phase3 = x_phase2[-1] + v_phase2[-1] * (t_phase3 - t2) - 0.5 * self.umin * (t_phase3 - t2)**2
            v_phase3 = v_phase2[-1] - self.umin * (t_phase3 - t2)
            u_phase3 = np.full_like(t_phase3, -self.umin)
        elif case_idx == 1:
            x_phase1 = self.x0 + self.v0 * t_phase1 - 0.5 * self.umin * t_phase1**2
            v_phase1 = self.v0 - self.umin * t_phase1
            u_phase1 = np.full_like(t_phase1, -self.umin)
            
            x_phase2 = x_phase1[-1] + v_phase1[-1] * (t_phase2 - t1)
            v_phase2 = np.full_like(t_phase2, v_phase1[-1])
            u_phase2 = np.full_like(t_phase2, 0)
            
            x_phase3 = x_phase2[-1] + v_phase2[-1] * (t_phase3 - t2) + 0.5 * self.umax * (t_phase3 - t2)**2
            v_phase3 = v_phase2[-1] + self.umax * (t_phase3 - t2)
            u_phase3 = np.full_like(t_phase3, self.umax)
        
        # Concatenate all phases
        t_all = np.concatenate((t_phase1, t_phase2, t_phase3))
        x_all = np.concatenate((x_phase1, x_phase2, x_phase3))
        v_all = np.concatenate((v_phase1, v_phase2, v_phase3))
        u_all = np.concatenate((u_phase1, u_phase2, u_phase3))
        
        # Plotting
        plt.figure(figsize=(12, 9))
        
        # Position plot
        plt.subplot(3, 1, 1)
        plt.plot(t_all, x_all, label="Position")
        plt.xlabel("Time")
        plt.ylabel("Position")
        plt.title("Time-Optimal Trajectory: Position vs Time")
        plt.grid(True)
        plt.legend()
        
        # Velocity plot
        plt.subplot(3, 1, 2)
        plt.plot(t_all, v_all, label="Velocity", color="red")
        plt.xlabel("Time")
        plt.ylabel("Velocity")
        plt.title("Time-Optimal Trajectory: Velocity vs Time")
        plt.grid(True)
        plt.legend()
        
        # acceleration plot
        plt.subplot(3, 1, 3)
        plt.plot(t_all, u_all, label="Acceleration", color="green")
        plt.xlabel("Time")
        plt.ylabel("Acceleration")
        plt.title("Time-Optimal Trajectory: Acceleration vs Time")
        plt.grid(True)
        plt.legend()

        plt.tight_layout()
        plt.show()

def find_alpha(t_optimal_pmm, T_target, tolerance=1e-2):
    alpha_low, alpha_high = 0.0, 1.0
    alpha = (alpha_low + alpha_high) / 2.0
    
    while alpha_high - alpha_low > tolerance:
        t_optimal_pmm.umax = alpha * t_optimal_pmm.original_umax
        t_optimal_pmm.umin = alpha * t_optimal_pmm.original_umin
        try:
            _, _, T, _ = t_optimal_pmm.compute_times()
            print(T)
        except ValueError:
            alpha_low = alpha
        else:
            if T > T_target:
                alpha_low = alpha
            else:
                alpha_high = alpha
        
        alpha = (alpha_low + alpha_high) / 2.0
    
    return alpha

# Example usage:
x0, y0, z0 = 0, 0, 0
v0_x, v0_y, v0_z = 0, 0, 0
xT, yT, zT = -8, 7, 2
vT_x, vT_y, vT_z = 3, 6, 0
umax_x, umin_x = 12, 12
umax_y, umin_y = 12, 12
umax_z, umin_z = 2.5, 15
vmax_x, vmin_x = 7.5, 7.5
vmax_y, vmin_y = 7.5, 7.5
vmax_z, vmin_z = 5, 5

# Calculate time for each axis
pmm_x = TimeOptimalPMM(x0, v0_x, xT, vT_x, umax_x, umin_x, vmax_x, vmin_x)
pmm_y = TimeOptimalPMM(y0, v0_y, yT, vT_y, umax_y, umin_y, vmax_y, vmin_y)
pmm_z = TimeOptimalPMM(z0, v0_z, zT, vT_z, umax_z, umin_z, vmax_z, vmin_z)

t1_x, t2_x, T_x, _ = pmm_x.compute_times()
t1_y, t2_y, T_y, _ = pmm_y.compute_times()
t1_z, t2_z, T_z, _ = pmm_z.compute_times()

# Determine the maximum minimum time
T = max(T_x, T_y, T_z)
print("T: {}, T_x: {}, T_y: {}, T_z: {}".format(T, T_x, T_y, T_z))

# Find the alpha value for the remaining two axes
if T_x < T:
    alpha_x = find_alpha(pmm_x, T)
else:
    alpha_x = 1.0
print("alpha_x:", alpha_x)

if T_y < T:
    alpha_y = find_alpha(pmm_y, T)
else:
    alpha_y = 1.0
print("alpha_y:", alpha_y)

if T_z < T:
    alpha_z = find_alpha(pmm_z, T)
else:
    alpha_z = 1.0
print("alpha_z:", alpha_z)

# Plot trajectories
pmm_x.plot_trajectory()
pmm_y.plot_trajectory()
pmm_z.plot_trajectory()
