#!/usr/bin/env python
#coding=utf-8

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from scipy.optimize import minimize

# 定义贝塞尔曲线的函数
def bezier_curve(cp, t):
    B0 = (1 - t)**3
    B1 = 3 * (1 - t)**2 * t
    B2 = 3 * (1 - t) * t**2
    B3 = t**3
    curve = np.outer(B0, cp[0]) + np.outer(B1, cp[1]) + np.outer(B2, cp[2]) + np.outer(B3, cp[3])
    return curve

def piecewise_bezier(cps, t):
    idx = int(t - 1e-5)
    t -= idx
    curve_point = bezier_curve(cps[idx], t)[0]
    return curve_point

# 计算给定点到贝塞尔曲线上每一点的欧氏距离
def point_to_curve_distance(point, cp, t):
    curve_point = piecewise_bezier(cp, t)
    return np.linalg.norm(curve_point - point)

# 遍历找到曲线上距离给定点最近的点
def find_nearest_point_on_curve(cp, target_point, num_samples=1000):
    t_values = np.linspace(0, cp.shape[0], num_samples*cp.shape[0]+1)
    distances = np.array([point_to_curve_distance(target_point, cp, t) for t in t_values])
    min_index = np.argmin(distances)
    return t_values[min_index], distances[min_index], piecewise_bezier(cp, t_values[min_index])


# 计算贝塞尔曲线的长度
def bezier_length(cp):
    t = np.linspace(0, 1, 100)
    curve = bezier_curve(cp, t)
    return np.sum(np.sqrt(np.sum(np.diff(curve, axis=0)**2, axis=1)))

# 目标函数，最小化贝塞尔曲线的总长度
def objective(x, points):
    n = len(points)
    control_points = x.reshape((n-1, 4, 3))
    L = 0
    for i in range(n-1):
        cp = control_points[i]
        L += bezier_length(cp)
    return L

# 约束函数，确保位置和速度连续
def constraints(x, points):
    n = len(points)
    control_points = x.reshape((n-1, 4, 3))
    cons = []

    # 位置约束
    for i in range(n-1):
        cons.append({'type': 'eq', 'fun': lambda x, i=i: x[i*12:i*12+3] - points[i]})
        cons.append({'type': 'eq', 'fun': lambda x, i=i: x[i*12+9:i*12+12] - points[i+1]})

    return cons

# 软约束的惩罚函数
def penalty(x, points):
    n = len(points)
    control_points = x.reshape((n-1, 4, 3))
    penalty_value = 0

    # 速度连续性和加速度约束
    for i in range(1, n-1):
        cp_prev = control_points[i-1]
        cp_curr = control_points[i]
        velocity_diff = 3 * (cp_prev[3] - cp_prev[2]) - 3 * (cp_curr[1] - cp_curr[0])
        penalty_value += np.sum(velocity_diff**2)  # 惩罚速度连续性的不满足情况

    # 加速度约束
    max_a = 6
    for i in range(n-1):
        cp = control_points[i]
        acceleration_0 = 6 * cp[0] - 12 * cp[1] + 6 * cp[2]
        acceleration_1 = 6 * cp[1] - 12 * cp[2] + 6 * cp[3]
        penalty_value += np.sum(np.maximum(0, np.linalg.norm(acceleration_0) - max_a)**2)
        penalty_value += np.sum(np.maximum(0, np.linalg.norm(acceleration_1) - max_a)**2)

    return penalty_value

# 初始化控制点
def initialize_control_points(points):
    n = len(points)
    control_points = np.zeros((n-1, 4, 3))
    for i in range(n-1):
        control_points[i, 0] = points[i]
        control_points[i, 3] = points[i+1]
        if i == 0:
            control_points[i, 1] = points[i] + np.array([1, 0, 0])
            control_points[i, 2] = points[i+1] - np.array([1, 0, 0])
        else:
            control_points[i, 1] = points[i] + np.array([1, 0, 0])
            control_points[i, 2] = points[i+1] - np.array([1, 0, 0])
    return control_points

# 修改绘制函数，添加最近点的绘制
def plot_bezier_curves(points, control_points_opt, target_point, nearest_point):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for i in range(control_points_opt.shape[0]):
        cp = control_points_opt[i]
        ax.plot(cp[:, 0], cp[:, 1], cp[:, 2], 'go-', label='Control Points')
        t = np.linspace(0, 1, 100)
        curve = bezier_curve(cp, t)
        ax.plot(curve[:, 0], curve[:, 1], curve[:, 2], 'b-', linewidth=2, label='Bezier Curve')

    ax.scatter(target_point[0], target_point[1], target_point[2], c='y', marker='*', label='Target Point')
    ax.scatter(nearest_point[0], nearest_point[1], nearest_point[2], c='r', marker='o', label='Nearest Point on Curve')
    ax.legend()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Bezier Curve with Nearest Point to Target')
    plt.show()

# main函数中添加目标点，并调用新的绘图函数
def main():
    # points = np.array([[0, 0, 0], [1, 2, 1], [3, 3, 4], [4, 1, 2], [5, 4, 5]])
    points = np.array([[0, 0, 0], [3, 2, 1], [6, 5, 1], [10, 4, 0]])
    # target_point = np.array([2, 2, 2])  # 这里可以替换为任意目标点的坐标
    target_point = np.array([1, 2, 2])
    control_points = initialize_control_points(points)
    x0 = control_points.flatten()

    cons = constraints(x0, points)
    penalty_weight = 1e3  # 惩罚函数权重

    # 最小化目标函数加上惩罚项
    def objective_with_penalty(x, points):
        return objective(x, points) + penalty_weight * penalty(x, points)

    result = minimize(objective_with_penalty, x0, args=(points,), constraints=cons, method='SLSQP', options={'disp': True, 'ftol': 1e-4, 'maxiter': 1000})

    control_points_opt = result.x.reshape((len(points)-1, 4, 3))
    if result.success:
        control_points_opt = result.x.reshape((len(points)-1, 4, 3))
        print("Optimized control points:", control_points_opt)
        
        # 找到最近的点
        nearest_t, dis, nearest_point_on_curve = find_nearest_point_on_curve(control_points_opt, target_point)
        print(nearest_t, dis, nearest_point_on_curve)
        
        # 绘制曲线和最近点
        plot_bezier_curves(points, control_points_opt, target_point, nearest_point_on_curve)

if __name__ == "__main__":
    main()
