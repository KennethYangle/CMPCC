#include "map.h"
#include <ros/package.h>
#include <ros/ros.h>

using namespace std;
using namespace Eigen;
namespace ft{
    Map::Map(){}

    void Map::setPathPts(const quadrotor_msgs::PiecewiseBezier::ConstPtr& msg){
        control_points.clear();
        theta_sample.clear();
        pos_sample.clear();

        num_order = msg->num_order;
        num_segment = msg->num_segment;
        K_data_my = msg->K;
        K_max = msg->K_max;
        thetaMax = num_segment * 10.0;

        int idx = 0;
        for (int i=0; i<num_segment; i++) {
            Eigen::MatrixXd piece_cpts = MatrixXd::Zero(K_max, 3);
            for (int j=0; j<K_data_my[i]; j++) {
                piece_cpts(j, 0) = msg->pts[idx].x;
                piece_cpts(j, 1) = msg->pts[idx].y;
                piece_cpts(j, 2) = msg->pts[idx].z;
                idx++;
            }
            control_points.push_back(piece_cpts);
        }

        Eigen::Vector3d thetaPoint;
        for (double theta=0; theta<thetaMax; theta+=0.001){
            getGlobalCommand(theta, thetaPoint);
            theta_sample.push_back(theta);
            pos_sample.push_back(thetaPoint);
        }
    }

    double Map::findNearestTheta(double theta, Eigen::Vector3d & position){
        int index = 0;
        int left = 0;
        int right = theta_sample.size()-1;
        double distance = 0;
        double nearestTheta = 0;
        double distanceMin = 1000;
        double error = 10;
        // 二分法找传入的theta对应的下标
        while(fabs(error) > 0.005){
            error = theta_sample[index] - theta;
            if(error > 0){
                right = index;
                index = (index+left)/2;
            }
            else{
                left = index;
                index = (index+right)/2;
            }
        }
        // 依次以0.1, 0.01, 0.001为间隔找最接近position的theta
        for (int i=((index-1000)>0? index-1000:0); i<(index+1000<theta_sample.size()? index+1000:theta_sample.size()-1); i+=100){
            distance = (position - pos_sample[i]).squaredNorm();
            if(distance < distanceMin){
                distanceMin = distance;
                nearestTheta = theta_sample[i];
                index = i;
            }
        }
        for (int i=((index-100)>0? index-100:0); i<(index+100<theta_sample.size()? index+100:theta_sample.size()-1); i+=10){
            distance = (position - pos_sample[i]).squaredNorm();
            if(distance < distanceMin){
                distanceMin = distance;
                nearestTheta = theta_sample[i];
                index = i;
            }
        }
        for (int i=((index-10)>0? index-10:0); i<(index+10<theta_sample.size()? index+10:theta_sample.size()-1); i+=1){
            distance = (position - pos_sample[i]).squaredNorm();
            if(distance < distanceMin){
                distanceMin = distance;
                nearestTheta = theta_sample[i];
            }
        }
        return nearestTheta;
    };

    // 通过p找theta
    double Map::findNearestTheta(Eigen::Vector3d & position){
        int index;
        double distance = 0;
        double nearestTheta = 0;
        double distanceMin = 10;
        // sampling method for finding the nearest point on the trajectory
        // 依次以0.1, 0.01, 0.001为间隔找最接近position的theta
        for (int i=0; i<theta_sample.size(); i+=100){
            distance = (position - pos_sample[i]).squaredNorm();
            if(distance < distanceMin){
                distanceMin = distance;
                nearestTheta = theta_sample[i];
                index = i;
            }
        }
        for (int i=((index-100)>0? index-100:0); i<(index+100<theta_sample.size()? index+100:theta_sample.size()-1); i+=10){
            distance = (position - pos_sample[i]).squaredNorm();
            if(distance < distanceMin){
                distanceMin = distance;
                nearestTheta = theta_sample[i];
                index = i;
            }
        }
        for (int i=((index-10)>0? index-10:0); i<(index+10<theta_sample.size()? index+10:theta_sample.size()-1); i+=1){
            distance = (position - pos_sample[i]).squaredNorm();
            if(distance < distanceMin){
                distanceMin = distance;
                nearestTheta = theta_sample[i];
            }
        }
        // cout << "position: " << position(0) << ", " << position(1) << ", " << position(2) << ".   nearestTheta: " << nearestTheta << endl;
        return nearestTheta;
    }

    // 计算贝塞尔基础函数，3阶，第i个点
    double binom[4] = {1,3,3,1};
    inline double bezierBasis(int i, double t) {
        return pow(1 - t, 3 - i) * pow(t, i) * binom[i];
    }

    // 计算贝塞尔基础函数的导数，3阶，第i个点
    double bezierBasisDerivative(int i, double t) {
        if (i==0) return -3*(1-t)*(1-t);
        if (i==1) return 3*(1-t)*(1-3*t);
        if (i==2) return 3*t*(2-3*t);
        if (i==3) return 3*t*t;
    }

    // 3阶贝塞尔曲线位置计算函数
    Vector3d getBezierPos(double t, Eigen::MatrixXd cps){
        assert(cps.rows() == 4 && cps.cols() == 3 && "Control points matrix must be 4x3.");

        Vector3d pos(0, 0, 0);
        for (int i = 0; i < cps.rows(); ++i) {
            double basis = bezierBasis(i, t);
            pos += cps.row(i) * basis;
        }
        return pos;
    }

    // 3阶贝塞尔曲线速度计算函数
    Vector3d getBezierVel(double t, const Eigen::MatrixXd& cps) {
        assert(cps.rows() == 4 && cps.cols() == 3 && "Control points matrix must be 4x3.");

        Vector3d vel(0, 0, 0);
        for (int i = 0; i < cps.rows(); ++i) {
            double basisDeriv = bezierBasisDerivative(i, t);
            vel += cps.row(i) * basisDeriv;
        }
        return vel / 3.0;
    }

    // 3阶贝塞尔曲线加速度计算函数
    Eigen::Vector3d getBezierAcc(double t, const Eigen::MatrixXd& cps) {
        assert(cps.rows() == 4 && cps.cols() == 3 && "Control points matrix must be 4x3.");

        // 计算加速度的每个分量
        Eigen::Vector3d acc(0, 0, 0);
        acc = (6-6*t) * cps.row(0) + (18*t-12) * cps.row(1) + (6-18*t) * cps.row(2) + 6*t * cps.row(3);
        return acc;
    }

    void Map::getGlobalCommand(double t, Vector3d & position){
        t = std::min(t, thetaMax) / 10;     // 保护不越界

        int idx = int(t);   // 位于第几段
        t -= idx;           // [0, 1)
        // cout<< t << endl;

        position = getBezierPos(t, control_points[idx]);
    }

    void Map::getGlobalCommand(double t, Vector3d & position, Vector3d & velocity){
        t = std::min(t, thetaMax) / 10;     // 保护不越界

        int idx = int(t);   // 位于第几段
        t -= idx;           // [0, 1)

        position = getBezierPos(t, control_points[idx]);
        velocity = getBezierVel(t, control_points[idx]); 
    }
    
    void Map::getGlobalCommand(double t, Vector3d & position, Vector3d & velocity, Vector3d & acceleration)
    {   
        t = std::min(t, thetaMax) / 10;     // 保护不越界

        int idx = int(t);   // 位于第几段
        t -= idx;           // [0, 1)

        position = getBezierPos(t, control_points[idx]);
        velocity = getBezierVel(t, control_points[idx]);
        acceleration = getBezierAcc(t, control_points[idx]);
    }

    double Map::getYaw(double t){
        t = std::min(t, thetaMax) / 10;     // 保护不越界
        int idx = int(t);   // 位于第几段
        t -= idx;           // [0, 1)

        Vector3d velocity = getBezierVel(t, control_points[idx]); 
        return std::atan2(velocity[1], velocity[0]);
    }

} //namespace ft
