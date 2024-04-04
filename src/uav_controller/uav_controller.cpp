#include "uav_controller/uav_controller.h"
#include "iostream"

Controller::Controller(Params params) {
    params_ = params;
    c_t_ = 1.105e-05;
    c_m_ = 1.779e-07*2;
    d_ = 0.225;
    g_ = 9.8;
    m_ = 1.4;
}

void Controller::hybridController(const Eigen::Vector4d& forces) {
    double k1 = 1/(4 * c_m_ * c_t_ * d_);
    double kf = c_m_ * d_ * forces(0);
    double kx = sqrt(2) * c_m_ * forces(1);
    double ky = sqrt(2) * c_m_ * forces(2);
    double kz = c_t_ * d_ * forces(3);
    w_(0) = sqrt(std::max(k1 * (kf + kx -ky - kz), 0.0));
    w_(1) = sqrt(std::max(k1 * (kf - kx +ky - kz), 0.0));
    w_(2) = sqrt(std::max(k1 * (kf - kx -ky + kz), 0.0));
    w_(3) = sqrt(std::max(k1 * (kf + kx +ky + kz), 0.0));
}

void Controller::controller2D(Eigen::Vector3d y_des, Eigen::Vector3d z_des) {
    const double k_dz = 2.5, k_pz = 8;
    const double k_dy = 8, k_py = 15;
    const double k_dphi = 20, k_pphi = 100; 
    double I_xx = 0.0211;
    Eigen::Vector4d forces;
    forces.setZero();
    // z-dim control
    forces(0) = m_ * (g_ + z_des(2) + k_dz * (z_des(1) - state_.v(2)) + k_pz * (z_des(0) - state_.x(2)));
    // y-dim control
    double phi_c = -(y_des(2) + k_dy * (y_des(1) - state_.v(1)) + k_py * (y_des(0) - state_.x(1))) / g_;
    phi_c = std::max(-0.4, std::min(phi_c, 0.4));
    forces(1) =  I_xx * (k_pphi * (phi_c - state_.angle_zxy(1)) + k_dphi * (0 - state_.angle_dot_zxy(1)));
    hybridController(forces);
}

void Controller::controllerSE3(std::vector<Eigen::Vector3d> pva_des) {
    Eigen::Vector4d forces;
    //
    Eigen::Vector3d position_err = pva_des[0] - state_.x;
    Eigen::Vector3d velocity_err = pva_des[1] - state_.v;  //世界系下的速度误差
    // 世界坐标系的z轴方向向量
    Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());
	// 计算期望加速度，先算出期望升力F，然后除以质量m
    // F = -kp*ep - kv*ev + m*g*zw + m*r_ddot_T
    Eigen::Vector3d F = (position_err.cwiseProduct(params_.k_p) + velocity_err.cwiseProduct(params_.k_v)) +
                        m_ * g_ * e_3 + m_ * pva_des[2];
    //点乘z_b得到u1
    forces(0) = F.dot(state_.R.col(2).normalized());
    // 以下计算u2 u3 u4 Eigen::Vector3d b1_des;
    Eigen::Vector3d angle_zxy = state_.angle_zxy;
    Eigen::Vector3d z_b_des = F / F.norm();
    Eigen::Vector3d x_c_des(cos(angle_zxy(0)), sin(angle_zxy(0)), 0);
    Eigen::Vector3d y_c_des = z_b_des.cross(x_c_des).normalized();
    Eigen::Matrix3d R_c_b = Eigen::Matrix3d::Zero();  //期望的旋转矩阵
    R_c_b.col(0) = x_c_des;
    R_c_b.col(1) = y_c_des;
    R_c_b.col(2) = z_b_des;
    //姿态误差，R_c_b就是R_des
    Eigen::Matrix3d R = state_.R;
    Eigen::Matrix3d angle_err_matrix = 0.5 * (R_c_b.transpose() * R - R.transpose() * R_c_b);
    Eigen::Vector3d angle_err(angle_err_matrix(2,1), angle_err_matrix(0,2), angle_err_matrix(1,0));
    
    // TODO: 角速度误差，机体坐标系下实际角速度和期望角速度的差
    Eigen::Vector3d angle_dot_xyz(state_.angle_dot_zxy(1), state_.angle_dot_zxy(2), state_.angle_dot_zxy(0));
    Eigen::Vector3d angle_dot_err = angle_dot_xyz;// - pva_des[2];
    forces.tail(3) = -angle_err.cwiseProduct(params_.k_R) - angle_dot_err.cwiseProduct(params_.k_w)
                    + angle_dot_xyz.cross(angle_dot_xyz);;
    hybridController(forces);
}

void Controller::setState(const std::vector<Eigen::Vector3d>& pva, Eigen::Matrix3d R, 
                            Eigen::Vector3d angle_zxy, Eigen::Vector3d angle_dot_zxy) {
    state_.x = pva[0];
    state_.v = pva[1];
    state_.R = R;
    state_.acc = pva[2];
    state_.angle_zxy = angle_zxy;  // 机体系
    state_.angle_dot_zxy = angle_dot_zxy;
}