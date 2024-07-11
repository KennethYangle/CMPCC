#!/usr/bin/env python
#coding=utf-8

import rospy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from scipy.optimize import minimize
from geometry_msgs.msg import Point
from quadrotor_msgs.msg import PiecewiseBezier
from swarm_msgs.msg import MassPoints

class PathPlanner:
    def __init__(self):
        rospy.init_node('planning_path_node', anonymous=True)
        self.refer_path_cps_pub = rospy.Publisher('refer_path_cps', PiecewiseBezier, queue_size=2)
        self.path_points_sub = rospy.Subscriber('path_points', MassPoints, self.path_points_callback, queue_size=1)
        self.points = None
        self.points_vel = None
        self.control_points = None

    def path_points_callback(self, msg):
        self.points = np.array([[pt.position.x, pt.position.y, pt.position.z] for pt in msg.points])
        self.points_vel = np.array([[pt.velocity.x, pt.velocity.y, pt.velocity.z] for pt in msg.points])
        # print(self.points)
        self.plan_path()

    def bezier_curve(self, cp, t):
        B0 = (1 - t)**3
        B1 = 3 * (1 - t)**2 * t
        B2 = 3 * (1 - t) * t**2
        B3 = t**3
        curve = np.outer(B0, cp[0]) + np.outer(B1, cp[1]) + np.outer(B2, cp[2]) + np.outer(B3, cp[3])
        return curve

    def bezier_length(self, cp):
        t = np.linspace(0, 1, 100)
        curve = self.bezier_curve(cp, t)
        return np.sum(np.sqrt(np.sum(np.diff(curve, axis=0)**2, axis=1)))

    def objective(self, x, points):
        n = len(points)
        control_points = x.reshape((n-1, 4, 3))
        L = 0
        for i in range(n-1):
            cp = control_points[i]
            L += self.bezier_length(cp)
        return L

    def constraints(self, x, points):
        n = len(points)
        control_points = x.reshape((n-1, 4, 3))
        cons = []

        # 位置约束
        for i in range(n-1):
            cons.append({'type': 'eq', 'fun': lambda x, i=i: x[i*12:i*12+3] - points[i]})
            cons.append({'type': 'eq', 'fun': lambda x, i=i: x[i*12+9:i*12+12] - points[i+1]})

        return cons

    def penalty(self, x, points):
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

    def initialize_control_points(self, points, points_vel):
        n = len(points)
        control_points = np.zeros((n-1, 4, 3))
        for i in range(n-1):
            control_points[i, 0] = points[i]
            control_points[i, 3] = points[i+1]
            if i == 0:
                control_points[i, 1] = points[i] + points_vel[i]
                control_points[i, 2] = points[i+1] - np.array([1., 0., 0.])
            else:
                control_points[i, 1] = points[i] + np.array([1., 0., 0.])
                control_points[i, 2] = points[i+1] - np.array([1., 0., 0.])
        return control_points

    def plot_bezier_curves(self, points, control_points_opt):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        for i in range(control_points_opt.shape[0]):
            cp = control_points_opt[i]
            ax.plot(cp[:, 0], cp[:, 1], cp[:, 2], 'go-', label='Control Points {}'.format(i+1))
            t = np.linspace(0, 1, 100)
            curve = self.bezier_curve(cp, t)
            ax.plot(curve[:, 0], curve[:, 1], curve[:, 2], 'b-', linewidth=2, label='Bezier Curve {}'.format(i+1))
        ax.plot(points[:, 0], points[:, 1], points[:, 2], 'r*', label='Points')
        ax.legend()
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Optimized 3D Bezier Curves')
        plt.show()

    def create_path_msg(self, control_points_opt):
        refer_path_cps_cmd = PiecewiseBezier()
        refer_path_cps_cmd.header.stamp = rospy.Time.now()
        refer_path_cps_cmd.num_order = 3
        refer_path_cps_cmd.num_segment = control_points_opt.shape[0]

        for i in range(refer_path_cps_cmd.num_segment):
            cp = control_points_opt[i]
            refer_path_cps_cmd.K.append(cp.shape[0])

            for j in range(refer_path_cps_cmd.K[i]):
                pt = Point()
                pt.x = cp[j, 0]
                pt.y = cp[j, 1]
                pt.z = cp[j, 2]
                refer_path_cps_cmd.pts.append(pt)

        refer_path_cps_cmd.K_max = max(refer_path_cps_cmd.K)

        return refer_path_cps_cmd

    def plan_path(self):
        if self.points is None:
            return
        
        control_points = self.initialize_control_points(self.points, self.points_vel)
        x0 = control_points.flatten()

        cons = self.constraints(x0, self.points)
        penalty_weight = 1e3  # 惩罚函数权重

        # 最小化目标函数加上惩罚项
        def objective_with_penalty(x, points):
            return self.objective(x, points) + penalty_weight * self.penalty(x, points)

        result = minimize(objective_with_penalty, x0, args=(self.points,), constraints=cons, method='SLSQP', options={'disp': True, 'ftol': 1e-4, 'maxiter': 1000})

        control_points_opt = result.x.reshape((len(self.points)-1, 4, 3))
        refer_path_cps_cmd = self.create_path_msg(control_points_opt)
        if result.success:
            self.refer_path_cps_pub.publish(refer_path_cps_cmd)
        # if result.success:
        #     control_points_opt = result.x.reshape((len(self.points)-1, 4, 3))
        #     print("Optimized control points:\n", control_points_opt)
        #     self.plot_bezier_curves(self.points, control_points_opt)
        # else:
        #     print("Optimization failed:", result.message)

if __name__ == "__main__":
    planner = PathPlanner()
    rospy.spin()
