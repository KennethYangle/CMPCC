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
        best_case_idx = None
        
        for case_idx in range(len(cases)):
            case = cases[case_idx]
            try:
                t1, t2, T = case()
                if 0 <= t1 <= t2 <= T and T < best_T:
                    best_T = T
                    best_times = (t1, t2, T, case_idx)
            except ValueError:
                continue
        
        if best_times is None:
            raise ValueError("No valid time solution found for the given conditions.")
        
        print(best_times)
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


# Example usage:
test = 8
if test == 1:
    x0 = 0  # initial position
    v0 = 0  # initial velocity
    xT = 2  # target position
    vT = 0  # target velocity
    umax = 2  # maximum acceleration
    umin = 2  # minimum acceleration (should be positive)
    vmax = 2  # maximum velocity
    vmin = 2 # minimum velocity (should be positive)
elif test == 2:
    x0 = 0; v0 = 0; xT = 2; vT = 0; umax = 2; umin = 1; vmax = 1; vmin = 1
elif test == 3:
    x0 = 0; v0 = 2; xT = -2; vT = 2; umax = 2; umin = 1; vmax = 3; vmin = 3
elif test == 4:
    x0 = 0; v0 = 0; xT = -2; vT = 0; umax = 2; umin = 1; vmax = 1; vmin = 1
elif test == 5:
    a = 0.492188
    x0 = 6.6; v0 = 1; xT = 12.5; vT = 0.3; umax = 12*a; umin = 12*a; vmax = 10; vmin = 10
elif test == 6:
    x0 = 11.56; v0 = 0.1; xT = 7.84; vT = 0.1; umax = 12; umin = 12; vmax = 10; vmin = 10
elif test == 7:
    x0 = 9.79; v0 = 0.5; xT = 14.37; vT = 1; umax = 3; umin = 7; vmax = 5; vmin = 5
elif test == 8:
    a = 0.69
    x0 = -6.4; v0 = -4.8; xT = -5; vT = 0.; umax = 12*a; umin = 12*a; vmax = 10; vmin = 10


time_optimal_pmm = TimeOptimalPMM(x0, v0, xT, vT, umax, umin, vmax, vmin)
time_optimal_pmm.plot_trajectory()
