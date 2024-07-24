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
    ros::NodeHandle nh;
    ros::Publisher refer_path_cps_pub;
    ros::Subscriber path_points_sub;
    // Constraint parameters
    std::vector<double> v_max, v_min, u_max, u_min;

    void loadConstraints();
};


#endif // PATH_PLANNING_H