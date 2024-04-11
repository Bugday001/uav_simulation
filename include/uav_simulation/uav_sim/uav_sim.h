#ifndef UAV_SIM_
#define UAV_SIM_
#include <iostream>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class Quadrotor
{
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
    Quadrotor();
    void position_dynamical();
    void attitude_dynamical();
    void attitude_kinematical();
    void position_kinematical();
    void getTuaF();
    void setInput(const Eigen::Vector4d &u);
    void setExternalForce(const Eigen::Vector3d &external_force);
    void step(double t);
    double getMass() { return m; }
    double getGravity() { return g; }
    double getCT() { return c_T; }
    double getPhi() { return phi; }
    double getPhi_dot() { return phi_dot_; }
    void updateState();
    void initState(const Eigen::Vector3d &init_state);
    State getState();

private:
    double c_T, c_M;  //拉力系数，力矩系数
    double dt_;
    double d;  //螺旋桨离中心距离
    double f;
    double g;
    double m;
    double psi, theta, phi;
    double Omega;
    double p_dot_, q_dot_, r_dot_;
    double w1, w2, w3, w4;
    double I_xx, I_yy, I_zz;
    double tau_x, tau_y, tau_z;
    double q, p, r;
    double J_RP;
    double phi_dot_, theta_dot_, psi_dot_;
    Quadrotor::State state_;
    Eigen::Matrix3d R_y_pi;
    Eigen::Vector3d external_force_;
};

#endif