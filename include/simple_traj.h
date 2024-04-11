#ifndef _SIMPLE_TRAJ_H
#define _SIMPLE_TRAJ_H
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>
using namespace std;
class SimpleTraj
{
private:
    std::vector<Eigen::Vector3d> way_points_;
    Eigen::Vector3d euler_;
    bool c_plus_; //航向旋转方向
    double target_yaw_, curr_yaw_;
    double max_vel_, max_acc_, max_omega_;
    double speedUpLen_, totalLen_;
    double speedUpTime_, totalTime_, rotatingTime_;
public:
    SimpleTraj();
    void setParams(double max_vel, double max_acc, double max_omega);
    void setWayPoints(std::vector<Eigen::Vector3d>& way_points, Eigen::Vector2d yaws);
    bool getPVA(vector<Eigen::Vector3d>& pva, double t);
};

SimpleTraj::SimpleTraj()
{
    max_vel_ = 0.1;
    max_acc_ = 0.1;
    max_omega_ = 0.1;
}

void SimpleTraj::setParams(double max_vel, double max_acc, double max_omega=0.1) 
{
    max_vel_ = max_vel;
    max_acc_ = max_acc;
    max_omega_ = max_omega;
    speedUpTime_ = max_vel_/max_acc_;
}

void SimpleTraj::setWayPoints(std::vector<Eigen::Vector3d>& way_points, Eigen::Vector2d yaws)
{
    curr_yaw_ = yaws[0];
    target_yaw_ = yaws[1];
    way_points_ = way_points;
    Eigen::Vector3d delta_xyz = way_points[1] - way_points[0];
    totalLen_ = delta_xyz.norm()+1e-14;
    euler_[0] = delta_xyz[0]/totalLen_;
    euler_[1] = delta_xyz[1]/totalLen_;
    euler_[2] = delta_xyz[2]/totalLen_;
    speedUpLen_ = 0.5*max_acc_*speedUpTime_*speedUpTime_;
    if(2*speedUpLen_ > totalLen_) {
        totalTime_ = 2 * sqrt(totalLen_/max_acc_);
    }
    else {
        totalTime_ = (totalLen_-speedUpLen_*2)/max_vel_ + 2*speedUpTime_;
    }
    // plan yaw
    //角度差与最短要旋转的角度差
    double diff_yaw = target_yaw_ - curr_yaw_, rotating_angle = 0;
    c_plus_ = true;
    //选择最短路旋转
    if(abs(diff_yaw)<2*M_PI-abs(diff_yaw)) {
        rotating_angle = diff_yaw;
    }
    else {
        rotating_angle = diff_yaw>0?-2*M_PI+abs(diff_yaw):2*M_PI-abs(diff_yaw);
    }
    std::cout<<"curr_yaw_: "<<curr_yaw_<<" target_yaw_: "<<target_yaw_<<endl;
    std::cout<<"diff_yaw: "<<diff_yaw<<" rotating_angle: "<<rotating_angle<<endl;
    std::cout<<"c_plus"<<c_plus_<<endl;
    c_plus_ = rotating_angle>0;
    rotatingTime_ = abs(rotating_angle) / max_omega_;  //旋转时长
}

/**
 * 返回是否完成
*/
bool SimpleTraj::getPVA(vector<Eigen::Vector3d>& pva, double t)
{   
    pva.resize(4);
    //Yaw
    if(t<rotatingTime_) {
        if(c_plus_) pva[3] = Eigen::Vector3d(curr_yaw_+max_omega_*t, max_omega_, 0);
        else pva[3] = Eigen::Vector3d(curr_yaw_-max_omega_*t, -max_omega_, 0);
    }
    else if(t>=rotatingTime_) {
        pva[3] = Eigen::Vector3d(target_yaw_, 0, 0);
    }
    //xyz
    if(t>totalTime_) {
        pva[0] = way_points_[1];
        pva[1].setZero();
        pva[2].setZero();
    }
    else {
        if(2*speedUpLen_ > totalLen_) {  //不能加到最大速度
            double speedupTime = totalTime_ / 2;
            if(t < speedupTime) {
                pva[0] = way_points_[0] + 0.5*max_acc_*euler_*t*t;
                pva[1] = max_acc_*euler_*t;
                pva[2] = max_acc_*euler_;
            }
            else {
                pva[0] = way_points_[0] + (totalLen_ - 0.5*max_acc_*(totalTime_-t)*(totalTime_-t))*euler_;
                pva[1] = max_acc_*(totalTime_-t)*euler_;
                pva[2] = -max_acc_*euler_;
            }
        }
        else {
            if(t < speedUpTime_) {
                pva[0] = way_points_[0] + 0.5*max_acc_*euler_*t*t;
                pva[1] = max_acc_*euler_*t;
                pva[2] = max_acc_*euler_;
            }
            else if(t<=totalTime_-speedUpTime_) {
                pva[0] = way_points_[0] + (speedUpLen_ + (t-speedUpTime_)*max_vel_)*euler_;
                pva[1] = max_vel_*euler_;
                pva[2].setZero();
            }
            else {
                pva[0] = way_points_[0] + (totalLen_ - 0.5*max_acc_*(totalTime_-t)*(totalTime_-t))*euler_;
                pva[1] = max_acc_*(totalTime_-t)*euler_;
                pva[2] = -max_acc_*euler_;
            }
        }
    }
    if(t>totalTime_ && t>rotatingTime_) return true;
    return false;
}
#endif