#ifndef UAV_CONTROLLER_HPP
#define UAV_CONTROLLER_HPP
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "params.h"

class Controller {
public:
    struct State
    {
        Eigen::Vector3d x;
        Eigen::Vector3d v;
        Eigen::Vector3d acc;
        Eigen::Matrix3d R;
        Eigen::Vector3d angle_zxy;
        Eigen::Vector3d angle_dot_zxy;
        // Eigen::Vector3d omega;
        Eigen::Vector4d  motor_rpm;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
public:
    Controller(Params params);
    void hybridController(const Eigen::Vector4d& forces);
    void controller2D(Eigen::Vector3d y_des, Eigen::Vector3d z_des);
    void controllerSE3(std::vector<Eigen::Vector3d> pva_des);
    void setState(const std::vector<Eigen::Vector3d>& pva, Eigen::Matrix3d R,Eigen::Vector3d angle_zxy, Eigen::Vector3d angle_dot_zxy);
    Eigen::Vector4d getRpm() { return w_; }
private:
    double c_t_, c_m_, d_;
    double g_, m_;
    Eigen::Vector4d w_;
    State state_;
    Params params_;
};

#endif