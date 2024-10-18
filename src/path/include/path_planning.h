#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <swarm_msgs/MassPoints.h>
#include <swarm_msgs/TimeOptimalPMMPieces.h>
#include <swarm_msgs/TimeOptimalPMMParam.h>
#include <geometry_msgs/Vector3.h>
#include "time_optimal_PMM.h"


class PathPlanningNode {
public:
    PathPlanningNode() {
        refer_path_cps_pub = nh.advertise<swarm_msgs::TimeOptimalPMMPieces>("refer_path_params", 2);
        path_points_sub = nh.subscribe("path_points", 1, &PathPlanningNode::pathPointsCallback, this);

        // Load constraints from YAML file
        loadConstraints();
    }

    void pathPointsCallback(const swarm_msgs::MassPoints::ConstPtr& msg);


private:
    geometry_msgs::Vector3 p1, p2, p3, v1, v2, v_dir, v, current_velocity;
    double current_V;
    ros::NodeHandle nh;
    ros::Publisher refer_path_cps_pub;
    ros::Subscriber path_points_sub;
    // Constraint parameters
    std::vector<double> v_max, v_min, u_max, u_min;

    void loadConstraints();
};



// a + b
geometry_msgs::Vector3 add_Vector3(const geometry_msgs::Vector3& a, const geometry_msgs::Vector3& b) {
    geometry_msgs::Vector3 result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;
    return result;
}

// a - b
geometry_msgs::Vector3 subtract_Vector3(const geometry_msgs::Vector3& a, const geometry_msgs::Vector3& b) {
    geometry_msgs::Vector3 result;
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    result.z = a.z - b.z;
    return result;
}

// normalize(a)
geometry_msgs::Vector3 normalize_Vector3(geometry_msgs::Vector3 a) {
    // 计算模
    double norm = sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    
    // 如果模长为0，返回原地向量(0,0,0)避免除以0的错误
    if (norm == 0) {
        return geometry_msgs::Vector3();
    } else {
        // 归一化向量
        geometry_msgs::Vector3 unit_vector;
        unit_vector.x = a.x / norm;
        unit_vector.y = a.y / norm;
        unit_vector.z = a.z / norm;
        return unit_vector;
    }
}

// norm(a)
inline double norm_Vector3(geometry_msgs::Vector3 a) {
    // 计算模
    return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

// scalar * v
geometry_msgs::Vector3 scalar_multiply_Vector3(const geometry_msgs::Vector3& v, double scalar) {
    geometry_msgs::Vector3 result;
    result.x = v.x * scalar;
    result.y = v.y * scalar;
    result.z = v.z * scalar;
    return result;
}


#endif // PATH_PLANNING_H