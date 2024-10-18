#include "path_planning.h"


void PathPlanningNode::pathPointsCallback(const swarm_msgs::MassPoints::ConstPtr& msg) {
    swarm_msgs::TimeOptimalPMMPieces optimal_paths_msg;
    optimal_paths_msg.header.stamp = ros::Time::now();
    optimal_paths_msg.num_segment = msg->points.size() - 1; // Number of segments is one less than number of points
    optimal_paths_msg.T_tatal = 0.0;

    current_velocity = msg->points[0].velocity;
    current_V = norm_Vector3(current_velocity);
    current_V = std::max(2.0, current_V);   // trick

    // 1. Construct basic information about optimal_paths_msg
    for (size_t i = 0; i < msg->points.size() - 1; ++i) {
        const auto& pt_start = msg->points[i];
        const auto& pt_end = msg->points[i + 1];

        // Extract position and velocity
        geometry_msgs::Vector3 x0 = pt_start.position;
        geometry_msgs::Vector3 v0 = pt_start.velocity;
        geometry_msgs::Vector3 xT = pt_end.position;
        geometry_msgs::Vector3 vT = pt_end.velocity;

        // Populate TimeOptimalPMMParam message
        swarm_msgs::TimeOptimalPMMParam path_param;
        path_param.trajectory_id = i;
        path_param.x0 = x0;
        path_param.v0 = v0;
        path_param.xT = xT;
        path_param.vT = vT;

        // Add path_param to optimal_paths_msg
        optimal_paths_msg.pieces.push_back(path_param);
    }

    // 2. Smooth the speed of the intermediate point
    for (size_t i = 0; i < msg->points.size() - 2; ++i) {
        p1 = msg->points[i].position;
        p2 = msg->points[i+1].position;
        p3 = msg->points[i+2].position;

        v1 = normalize_Vector3(subtract_Vector3(p2, p1));
        v2 = normalize_Vector3(subtract_Vector3(p3, p2));
        v_dir = normalize_Vector3(add_Vector3(v1, v2));
        v = scalar_multiply_Vector3(v_dir, current_V);

        optimal_paths_msg.pieces[i].vT = v;
        optimal_paths_msg.pieces[i+1].v0 = v;
    }

    // 3. Construct the trajectory parameters of optimal_paths_msg
    for (size_t i = 0; i < optimal_paths_msg.pieces.size(); ++i) {
        // Assuming constant constraints for simplicity
        geometry_msgs::Vector3 umax, umin, vmax, vmin;
        umax.x = u_max[0]; umax.y = u_max[1]; umax.z = u_max[2];
        umin.x = u_min[0]; umin.y = u_min[1]; umin.z = u_min[2];
        vmax.x = v_max[0]; vmax.y = v_max[1]; vmax.z = v_max[2];
        vmin.x = v_min[0]; vmin.y = v_min[1]; vmin.z = v_min[2];

        // Create TimeOptimalPMM instance and compute times
        TimeOptimalPMM3D pmm3d(optimal_paths_msg.pieces[i].x0, 
                               optimal_paths_msg.pieces[i].v0, 
                               optimal_paths_msg.pieces[i].xT, 
                               optimal_paths_msg.pieces[i].vT, 
                               umax, umin, vmax, vmin);
        std::cout << "x0: " << optimal_paths_msg.pieces[i].x0 << "\nxT: " << optimal_paths_msg.pieces[i].xT << "\nv0: " << optimal_paths_msg.pieces[i].v0 << "\nvT: " << optimal_paths_msg.pieces[i].vT << std::endl;
        auto [t1, t2, T, case_idx] = pmm3d.compute_times();
        // Synchronized u
        umax.x = pmm3d.pmm_x.umax; umax.y = pmm3d.pmm_y.umax; umax.z = pmm3d.pmm_z.umax;
        umin.x = pmm3d.pmm_x.umin; umin.y = pmm3d.pmm_y.umin; umin.z = pmm3d.pmm_z.umin;

        // Update TimeOptimalPMMParam message
        optimal_paths_msg.pieces[i].umax = umax;
        optimal_paths_msg.pieces[i].umin = umin;
        optimal_paths_msg.pieces[i].t1 = t1;
        optimal_paths_msg.pieces[i].t2 = t2;
        optimal_paths_msg.pieces[i].T = T;
        optimal_paths_msg.pieces[i].case_idx = case_idx;
        optimal_paths_msg.T_tatal += T;
    }

    // 4. Publish
    refer_path_cps_pub.publish(optimal_paths_msg);
}

// Run at construction
void PathPlanningNode::loadConstraints() {
    try {
        // Load YAML file
        std::string path = ros::package::getPath("params") + "/path_constraint.yaml";
        YAML::Node node = YAML::LoadFile(path);

        // Read velocity constraints
        v_max = node["v_max"].as<std::vector<double>>();
        v_min = node["v_min"].as<std::vector<double>>();

        // Read acceleration constraints
        u_max = node["u_max"].as<std::vector<double>>();
        u_min = node["u_min"].as<std::vector<double>>();

        ROS_INFO_STREAM("Loaded constraints from YAML file.");
    } catch (const YAML::Exception& e) {
        ROS_ERROR_STREAM("Failed to load constraints: " << e.what());
        ros::shutdown();
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "path_planning_node");
    PathPlanningNode node;
    ros::spin();
    return 0;
}
