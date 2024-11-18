#include "mcl_filter.h"

// Create a set of particles
vector<Robot> p;

cv::Mat imgCallback;
ros::Subscriber img_show_sub, expect_target_sub, rc_sub;
std::vector<ros::Subscriber> pose_subs, img_subs, target_pose_subs;
ros::Publisher pub_center_points, pub_target_center_points, pub_interceptors, pub_target_marker, pub_center_marker, pub_feature_marker, pub_expect_target_marker;
ros::Time last_img_stamp(0.);

void mav_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if (ch8 == 0) return;

    mav_pos(0) = msg->pose.position.x;
    mav_pos(1) = msg->pose.position.y;
    mav_pos(2) = msg->pose.position.z;
    mav_quad = msg->pose.orientation;
    tf::Quaternion RQ2;
    tf::quaternionMsgToTF(mav_quad, RQ2);
    tf::Matrix3x3(RQ2).getRPY(mav_roll, mav_pitch, mav_yaw);
    tf::quaternionMsgToEigen(mav_quad, mav_quad_eigen);
    mav_R = mav_quad_eigen.toRotationMatrix();

    // timestamp = msg->header.stamp;
    // ROS_DEBUG_STREAM("timestamp: " << (timestamp - timestamp_begin).toSec());
    // ROS_DEBUG_STREAM("mav_pos: " << mav_pos);
}

void mav_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    get_vel = true;
    mav_vel(0) = msg->twist.linear.x;
    mav_vel(1) = msg->twist.linear.y;
    mav_vel(2) = msg->twist.linear.z;
}

void mav_img_cb(const swarm_msgs::BoundingBoxes::ConstPtr &msg)
{
    // if (ch8 == 0) return;

    // ROS_INFO_STREAM("Enter mav_img_cb." << mav_id);
    // Hold on short-term loss
    int sphere_num = msg->bounding_boxes.size();
    if (sphere_num > 0)
        image_failed_cnt = 0;
    else
        image_failed_cnt += 1;
    if (image_failed_cnt <= image_failed_max_cnt && image_failed_cnt > 0)
        return;

    // Measurement
    Vector3d p_s_c = Vector3d(0., 0., 0.), p_s;    // 论文中{}^{\rm{s}}{\bf{p}}为相机系下向量，这里分别计算相机系和世界系下表示
    p_s_Vec.clear();
    for (int t=0; t<sphere_num; t++) {
        double msg_x = (msg->bounding_boxes[t].xmin + msg->bounding_boxes[t].xmax) / 2;
        double msg_y = (msg->bounding_boxes[t].ymin + msg->bounding_boxes[t].ymax) / 2;
        p_s_c = Vector3d(msg_x - img0(0), msg_y - img0(1), img_f);
        p_s_c.normalize();
        p_s = mav_R * R_cb * p_s_c;
        p_s_Vec.push_back(p_s);
    }

    double dt = (msg->header.stamp - last_img_stamp).toSec();
    last_img_stamp = msg->header.stamp;

    if (dt > 1e5) dt = 0.;
    // ROS_INFO_STREAM("dt: " << dt);

    // ROS_INFO_STREAM("Enter target_prediction.");
    for (int i = 0; i < p.size(); i++) {
        p[i].target_prediction(dt); // Prediction Step
    }
    // ROS_INFO_STREAM("Enter target_update.");
    for (int i = 0; i < p.size(); i++) {
        p[i].target_update();       // Update Step
    }
    // ROS_INFO_STREAM("Enter target_resampling.");
    target_resampling();            // Resampling Step
    // ROS_INFO_STREAM("Enter target_clustering.");
    target_clustering();            // Resampling Step: Clustering
    // ROS_INFO_STREAM("Enter publishBarycenter.");
    publishBarycenter();
    publishTargetBarycenter();
    // ROS_INFO_STREAM("Enter target_balance.");
    target_balance();               // Resampling Step: Balance

    // TODO: 根据时间戳寻找最接近图像时刻的位姿，计算权重
}

void rcin_cb(const mavros_msgs::RCIn::ConstPtr &msg)
{
    // "A", attack
    if (msg->channels[6] < 1300) {
        ch7 = 0;
    }
    else if (msg->channels[6] < 1700) {
        ch7 = 1;
    }
    else {
        ch7 = 2;
    }

    // "O", offboard
    if (msg->channels[7] < 1300) {
        ch8 = 0;
    }
    else if (msg->channels[7] < 1700) {
        ch8 = 1;
    }
    else {
        ch8 = 2;
    }
}

void mcl_cb(const ros::TimerEvent& event)
{
    
}

void target_resampling()
{
    vector<Robot> p2;
    int index = gen_real_random() * particle_num;
    double beta = 0.0;
    double mq = 0;
    for (int i = 0; i < particle_num; i++) {
        if (mq < p[i].q) {
            mq = p[i].q;
        }
    }

    for (int i = 0; i < particle_num; i++) {
        beta += gen_real_random() * 2.0 * mq;
        while (beta > p[index].q) {
            beta -= p[index].q;
            index = (index + 1) % particle_num;
        }
        p2.push_back(p[index]);
    }
    p.assign(p2.begin(), p2.end());
}

void target_clustering()
{
    int k = target_pos_hat.size(); // 聚类中心的数量

    for (int iter = 0; iter < max_iterations; ++iter) {
        // 分配步骤
        for (size_t i = 0; i < particle_num; ++i) {
            double minDist = std::numeric_limits<double>::max();
            int closestCenter = -1;
            for (int j = 0; j < k; ++j) {
                double dist = distance(p[i].position, target_pos_hat[j]);
                if (dist < minDist) {
                    minDist = dist;
                    closestCenter = j;
                }
            }
            p[i].cluster = closestCenter;
        }

        // 更新步骤
        std::vector<Eigen::Vector3d> newCenters(k, Eigen::Vector3d(0, 0, 0));
        std::vector<double> weights(k, 0.0);
        for (size_t i = 0; i < particle_num; ++i) {
            newCenters[p[i].cluster] += p[i].position * p[i].q;
            weights[p[i].cluster] += p[i].q;
        }

        for (int j = 0; j < k; ++j) {
            if (weights[j] > 0) {
                newCenters[j] /= weights[j];
            }
        }

        double maxShift = 0.0;
        for (int j = 0; j < k; ++j) {
            double shift = distance(target_pos_hat[j], newCenters[j]);
            maxShift = std::max(maxShift, shift);
        }

        target_pos_hat.assign(newCenters.begin(), newCenters.end());

        if (maxShift < 1e-3) {
            break; // 聚类中心变化小于阈值时停止
        }
    }
}

void target_balance()
{
    int k = target_pos_hat.size(); // 聚类中心的数量
    int targetPerCluster = particle_num / k; // 目标每个聚类的粒子数量
    std::vector<std::vector<Robot>> clusters(k);

    for (int i=0; i<particle_num; i++) {
        clusters[p[i].cluster].push_back(p[i]);
    }

    // 平衡每个聚类的粒子数量
    for (auto& cluster : clusters) {
        if (cluster.size() > targetPerCluster) {
            // 如果聚类中的粒子过多，则随机去除多余的粒子
            std::shuffle(cluster.begin(), cluster.end(), gen);
            cluster.resize(targetPerCluster);
        } else while (cluster.size() < targetPerCluster) {
            // 如果聚类中的粒子过少，则复制现有粒子
            Robot p = cluster[std::uniform_int_distribution<>(0, cluster.size() - 1)(gen)];
            cluster.push_back(p);
        }
    }

    p.clear();
    for (const auto& cluster : clusters) {
        p.insert(p.end(), cluster.begin(), cluster.end());
    }
}

void publishBarycenter()
{
    swarm_msgs::CenterPoints cps;
    geometry_msgs::Point cp;
    cps.header.stamp = ros::Time::now();
    int k = target_pos_hat.size(); // 聚类中心的数量

    for (int i=0; i<k; i++) {
        cp.x = target_pos_hat[i](0);
        cp.y = target_pos_hat[i](1);
        cp.z = target_pos_hat[i](2);
        cps.points.push_back(cp);
    }

    pub_center_points.publish(cps);
}

void publishTargetBarycenter()
{
    pub_target_center_points.publish(target_cps);
}


void visualization_cb(const ros::TimerEvent& event)
{
    double dt = (event.current_real - event.last_real).toSec();
    if (dt > 1e5) dt = 0.;

    publishInterceptors();
    publishTargetMarkers();
    publishCenterMarkers();
    publishDetectedFeatures();
}

// Draw interceptors' pose arrow
void publishInterceptors()
{
    mav_pos_Array.header.stamp = ros::Time::now();
    mav_pos_Array.header.frame_id = "world";
    
    pub_interceptors.publish(mav_pos_Array);
}

// Draw target particles
void publishTargetMarkers()
{
    visualization_msgs::MarkerArray markers;//定义MarkerArray对象
	for(int i = 0; i < particle_num; i++)
	{
		visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        // marker.header.stamp = ros::Time::now();
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.ns = "Target points";
		marker.id = i; //用来标记同一帧不同的对象，如果后面的帧的对象少于前面帧的对象，那么少的id将在rviz中残留，所以需要后续的实时更新程序
        marker.pose.position.x = p[i].position(0);
        marker.pose.position.y = p[i].position(1);
        marker.pose.position.z = p[i].position(2);
		marker.pose.orientation.w = 1.0;
        marker.scale.x = marker.scale.y = marker.scale.z = max(p[i].q, 0.2);
		marker.color.a = 1.0;
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0 - 0.3*p[i].cluster;
        // marker.lifetime = ros::Duration(0.2);
        markers.markers.push_back(marker);
	}
    pub_target_marker.publish(markers);
}


// Draw target particles on sphere
void publishCenterMarkers()
{
    visualization_msgs::MarkerArray markers;//定义MarkerArray对象
    // target_pos_hat
    int k = target_pos_hat.size(); // 聚类中心的数量
	for(int i = 0; i < k; i++)
	{
		visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        // marker.header.stamp = ros::Time::now();
		marker.type = visualization_msgs::Marker::CUBE;
		marker.ns = "Estimate targets position";
		marker.id = i;
        marker.pose.position.x = target_pos_hat[i](0);
        marker.pose.position.y = target_pos_hat[i](1);
        marker.pose.position.z = target_pos_hat[i](2);
		marker.pose.orientation.w = 1.0;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.3;
		marker.color.a = 1.0;
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 0.3 * k;
        // marker.lifetime = ros::Duration(0.2);
        markers.markers.push_back(marker);
	}
    pub_center_marker.publish(markers);
}

//Draw the direction of the detected features
void publishDetectedFeatures()
{
    visualization_msgs::MarkerArray markers;//定义MarkerArray对象

    for (int t=0; t<p_s_Vec.size(); t++) {
        visualization_msgs::Marker marker_arrow;
        marker_arrow.header.frame_id = "world";
        // marker_arrow.header.stamp = ros::Time::now();
        marker_arrow.type = visualization_msgs::Marker::ARROW;
        marker_arrow.ns = "Feature arrow";
        marker_arrow.id = t;
        marker_arrow.scale.x = 0.1;
        marker_arrow.scale.y = 0.16;
        marker_arrow.scale.z = 0.2;
        marker_arrow.color.a = 1.0;
        marker_arrow.color.r = 0.3;
        marker_arrow.color.g = 1.0;
        marker_arrow.color.b = 0.3;
        marker_arrow.lifetime = ros::Duration(0.2);

        geometry_msgs::Point pt1, pt2;
        pt1.x = mav_pos(0);   // start point
        pt1.y = mav_pos(1);
        pt1.z = mav_pos(2);
        // end point
        pt2.x = mav_pos(0) + p_s_Vec[t](0) * 1.5;
        pt2.y = mav_pos(1) + p_s_Vec[t](1) * 1.5;
        pt2.z = mav_pos(2) + p_s_Vec[t](2) * 1.5;

        marker_arrow.points.push_back(pt1);
        marker_arrow.points.push_back(pt2);

        markers.markers.push_back(marker_arrow);
    }
    pub_feature_marker.publish(markers);
}

// 获取ROS参数，预设vector大小
void get_ROS_param(ros::NodeHandle& nh)
{
    nh.param<int>("/target_num", target_num, 1);
    nh.param<int>("/particle_num", particle_num, 1000);
    particle_num *= target_num;
    target_pos.reserve(target_num);
    target_vel.reserve(target_num);
    target_pose_subs = std::vector<ros::Subscriber>(target_num);
    target_cps.points = std::vector<geometry_msgs::Point>(target_num);
    ROS_INFO_STREAM_ONCE("particle_num: " << particle_num);

    nh.param<double>("/mcl_params/alpha_1", params["alpha_1"], 0.0);
    nh.param<double>("/mcl_params/alpha_2", params["alpha_2"], 0.0);
    nh.param<double>("/mcl_params/alpha_3", params["alpha_3"], 0.0);
    nh.param<double>("/mcl_params/alpha_4", params["alpha_4"], 0.0);
    ROS_INFO_STREAM_ONCE("alpha_1: " << params["alpha_1"] << ", alpha_2: " << params["alpha_2"] << ", alpha_3: " << params["alpha_3"] << ", alpha_4: " << params["alpha_4"]);
    nh.param<double>("/mcl_params/init_pos_conv", params["init_pos_conv"], 1.0);
    nh.param<double>("/mcl_params/init_vel_conv", params["init_vel_conv"], 1.0);
    nh.param<double>("/mcl_params/sigma_p", params["sigma_p"], 1.0);
    ROS_INFO_STREAM_ONCE("init_pos_conv: " << params["init_pos_conv"] << ", init_vel_conv: " << params["init_vel_conv"] << ", sigma_p: " << params["sigma_p"]);
    
    nh.getParam("target", target_params);
    for (size_t i=0; i<target_num; i++) {
        Vector3d param_target_pos(target_params[i]["p_x"], target_params[i]["p_y"], target_params[i]["p_z"]);
        target_pos.push_back(param_target_pos);
        ROS_INFO_STREAM("target_pos: " << target_pos[i]);

        Vector3d param_target_vel(target_params[i]["v_x"], target_params[i]["v_y"], target_params[i]["v_z"]);
        target_vel.push_back(param_target_vel);
        ROS_INFO_STREAM("target_vel: " << target_vel[i]);
    }
    target_pos_hat.assign(target_pos.begin(), target_pos.end());

    nh.param<double>("/camera/img_width", img_width, 0.0);
    nh.param<double>("/camera/img_height", img_height, 0.0);
    img0 = Vector2d(img_width/2, img_height/2);
    nh.param<double>("/camera/img_f", img_f, 0.0);
    nh.param<double>("/camera/img_distance", img_distance, 20.0);
    ROS_INFO_STREAM_ONCE("img_width: " << img_width << ", img_height: " << img_height << ", img_f: " << img_f << ", img_distance: " << img_distance);
    nh.param<bool>("/Debug/is_show_image", is_show_image, false);
    R_cb << 0, 0, 1, -1, 0, 0, 0, -1, 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mcl_node");
    ros::NodeHandle nh;
    
    get_ROS_param(nh);

    // 初始化粒子
    for (int i = 0; i < particle_num; i++) {
        p.push_back( Robot(target_pos, target_vel, params) );
        // ROS_INFO_STREAM("particle[" << i << "]_init_pos: " << p[i].position);
    }

    // Subscriber
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
                ("mavros/local_position/pose", 10, mav_pose_cb);   //50hz
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
                ("mavros/local_position/velocity_local", 10, mav_vel_cb);//50hz
    ros::Subscriber img_sub = nh.subscribe<swarm_msgs::BoundingBoxes>
                ("tracker/pos_image", 10, mav_img_cb);          //23hz
    rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 2, rcin_cb);

    // Timer
    ros::Timer mcl_timer = nh.createTimer(ros::Duration(1.0/30), mcl_cb);
    ros::Timer visualization_timer = nh.createTimer(ros::Duration(1.0/15), visualization_cb);

    // Publisher
    pub_center_points = nh.advertise<swarm_msgs::CenterPoints>("/center_points", 1, true);
    pub_target_center_points = nh.advertise<swarm_msgs::CenterPoints>("/target/center_points", 1, true);
    pub_interceptors = nh.advertise<geometry_msgs::PoseArray>("/interceptors_pose", 1, true);
    pub_target_marker = nh.advertise<visualization_msgs::MarkerArray>("/target_marker", 1, true);
    pub_center_marker = nh.advertise<visualization_msgs::MarkerArray>("/center_marker", 1, true);
    pub_feature_marker = nh.advertise<visualization_msgs::MarkerArray>("/feature_marker", 1, true);
    pub_expect_target_marker = nh.advertise<visualization_msgs::MarkerArray>("/expect_target_marker", 1, true);
    ROS_INFO_STREAM("Create Timer & Publisher finish.");


    // ros::spin();
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
