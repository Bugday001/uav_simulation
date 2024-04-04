#include "uav_simulation/uav_sim/uav_sim.h"
Quadrotor::Quadrotor() {
    m = 1.4; //kg
    g = 9.8; //m/s^2
    phi = 0.0; //rad
    theta = 0.0; //rad
    psi = 0.0; //rad
    x = 0.0;
    y = 0.0;
    z = 0.0;
    v_x = 0.0; //m/s
    v_y = 0.0; //m/s
    v_z = 0.0; //m/s
    c_T = 1.105e-05;
    c_M = 1.779e-07*2;
    d = 0.225;
    I_xx = 0.0211;      // 四旋翼x轴转动惯量(kg·m^2)
    I_yy = 0.0219;      // 四旋翼y轴转动惯量(kg·m^2)
    I_zz = 0.0366;      // 四旋翼z轴转动惯量(kg·m^2)
    J_RP = 0.0001287;   // 整个电机转子和螺旋桨绕转轴的总转动惯量(kg·m^2)
    R_y_pi << -1, 0, 0, 0, 1, 0, 0, 0, -1;
}

/**
 * 1.作用：本函数为四旋翼的位置动力学微分方程组，通过四旋翼所受拉力、姿态角
 *         计算得到四旋翼飞行器的速度
 * 2.函数输入：
 *   m：四旋翼飞行器质量(kg)
 *   g：重力加速度(m/s^2)
 *   phi：滚转角(rad)
 *   theta：俯仰角(rad)
 *   psi：偏航角(rad)
 *   f：螺旋桨产生的总拉力(N)
 * 3.函数输出：
 *   v_x_dot：地球坐标系下沿x轴的速度的导数
 *   v_y_dot：地球坐标系下沿y轴的速度的导数
 *   v_z_dot：地球坐标系下沿z轴的速度的导数
 * 4.四旋翼构型为“X”型，螺旋桨序号如下所示：
 *            3↓   1↑
 *              \ /
 *              / \
 *            2↑   4↓
 *   其中，↑表示螺旋桨逆时针旋转；↓表示螺旋桨顺时针旋转。
*/
void Quadrotor::position_dynamical() {
    v_x_dot = f * (1/m) * (cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi));
    v_y_dot = f * (1/m) * (sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi));
    v_z_dot = -g + f * (1/m) * cos(phi) * cos(theta);
    v_x += v_x_dot * dt_;
    v_y += v_y_dot * dt_;
    v_z += v_z_dot * dt_;
    
}

/**
 *
 * 1.作用：本函数为四旋翼的姿态动力学微分方程组，通过四旋翼所受力矩和螺旋
 *         桨转速计算得到四旋翼的机体角速度
 * 2.函数输入：
 *   wi：四个螺旋桨的转速(rad/s)
 *   tau_x：x轴反扭力矩（机体轴）(N·m)
 *   tau_y：y轴反扭力矩（机体轴）(N·m)
 *   tau_z：z轴反扭力矩（机体轴）(N·m)
 *   I_xx：四旋翼x轴转动惯量(kg·m^2)
 *   I_yy：四旋翼y轴转动惯量(kg·m^2)
 *   I_zz：四旋翼z轴转动惯量(kg·m^2)
 *   J_RP：整个电机转子和螺旋桨绕转轴的总转动惯量(kg·m^2)
 *   p：四旋翼x轴角速度（机体轴)(rad/s)
 *   q：四旋翼y轴角速度（机体轴)(rad/s)
 *   r：四旋翼z轴角速度（机体轴)(rad/s)
 * 3.函数输出：
 *   p_dot：四旋翼x轴角加速度（机体轴)
 *   q_dot：四旋翼y轴角加速度（机体轴)
 *   r_dot：四旋翼z轴角加速度（机体轴)
 * 4.四旋翼构型为“X”型，螺旋桨序号如下所示：
 *            3↓   1↑
 *              \ /
 *              / \
 *            2↑   4↓
 *   其中，↑表示螺旋桨逆时针旋转；↓表示螺旋桨顺时针旋转。
*/
void Quadrotor::attitude_dynamical() {
    Omega = -w1 + w2 - w3 + w4;
    p_dot_ = (1/I_xx) * (tau_x + q * r * (I_yy - I_zz) - J_RP * q * Omega);
    q_dot_ = (1/I_yy) * (tau_y + p * r * (I_zz - I_xx) + J_RP * p * Omega);
    r_dot_ = (1/I_zz) * (tau_z + p * q * (I_xx - I_yy) );
    p += p_dot_ * dt_;
    q += q_dot_ * dt_;
    r += r_dot_ * dt_;
}

/**
 * 位置运动学微分方程组，通过四旋翼的速度计算得到四旋翼的位置
*/
void Quadrotor::position_kinematical() {
    x += v_x * dt_;
    y += v_y * dt_;
    z += v_z * dt_;
}

/**
 *  * 函数描述
 * 1.作用：本函数为四旋翼的姿态运动学微分方程组，通过四旋翼的机体角速度
 *         计算得到四旋翼的姿态角
 * 2.函数输入：
 *   p：四旋翼x轴角速度（机体轴)(rad/s)
 *   q：四旋翼y轴角速度（机体轴)(rad/s)
 *   r：四旋翼z轴角速度（机体轴)(rad/s)
 *   phi：滚转角(rad)
 *   theta：俯仰角(rad)
 * 3.函数输出：
 *   phi_dot：滚转角速度
 *   theta_dot：俯仰角速度
 *   psi_dot：偏航角速度
 * 4.四旋翼构型为“X”型，螺旋桨序号如下所示：
 *            3↓   1↑
 *              \ /
 *              / \
 *            2↑   4↓
 *   其中，↑表示螺旋桨逆时针旋转；↓表示螺旋桨顺时针旋转。
*/
void Quadrotor::attitude_kinematical() {
    phi_dot_ = p + q * tan(theta) * sin(phi) + r * tan(theta) * cos(phi);
    theta_dot_ = q * cos(phi) - r * sin(phi);
    psi_dot_ = (q * sin(phi) + r * cos(phi)) / cos(theta);
    phi += phi_dot_ * dt_;
    theta += theta_dot_ * dt_;
    psi += psi_dot_ * dt_;
}

/**
 *  * 函数描述
 * 1.作用：本函数用来计算螺旋桨旋转产生的总拉力和反扭力矩。
 * 2.函数输入：
 *   wi：四个螺旋桨的转速(rad/s)
 *   c_T：螺旋桨拉力系数
 *   c_M：螺旋桨力矩系数
 *   d：机体中心和任一电机的距离(m)
 * 3.函数输出：
 *   f：螺旋桨拉力（机体轴）
 *   tau_x：x轴反扭力矩（机体轴）
 *   tau_y：y轴反扭力矩（机体轴）
 *   tau_z：z轴反扭力矩（机体轴）
 * 4.四旋翼构型为“X”型，螺旋桨序号如下所示：
 *            3↓   1↑
 *              \ /
 *              / \
 *            2↑   4↓
 *   其中，↑表示螺旋桨逆时针旋转；↓表示螺旋桨顺时针旋转。
*/
void Quadrotor::getTuaF() {
    f = c_T * (w1*w1 + w2*w2 + w3*w3 + w4*w4);
    tau_x = d * c_T * (sqrt(2)/2) * (w1*w1 - w2*w2 - w3*w3 + w4*w4);
    tau_y = d * c_T * (sqrt(2)/2) * (-w1*w1 + w2*w2 - w3*w3 + w4*w4);
    tau_z = c_M * (-w1*w1 + -w2*w2 + w3*w3 + w4*w4);
}   

void Quadrotor::setInput(const Eigen::Vector4d &u) {
    state_.motor_rpm = u;
    w1 = u(0);
    w2 = u(1);
    w3 = u(2);
    w4 = u(3);
}

void Quadrotor::step(double t) {
    dt_ = t;
    getTuaF();
    position_dynamical();
    attitude_dynamical();
    position_kinematical();
    attitude_kinematical();
    updateState();
}

void Quadrotor::updateState() {
    state_.x(0) = x;
    state_.x(1) = y;
    state_.x(2) = z;
    state_.v(0) = v_x;
    state_.v(1) = v_y;
    state_.v(2) = v_z;
    state_.acc(0) = v_x_dot;
    state_.acc(1) = v_y_dot;
    state_.acc(2) = v_z_dot;
    state_.angle_zxy(0) = psi;
    state_.angle_zxy(1) = phi;
    state_.angle_zxy(2) = theta;
    state_.angle_dot_zxy(0) = r;
    state_.angle_dot_zxy(1) = p;
    state_.angle_dot_zxy(2) = q;
    state_.R << cos(theta)*cos(psi), cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(theta), cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi),
                cos(theta)*sin(psi), sin(psi)*sin(phi)*sin(phi)+cos(psi)*cos(phi), sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi),
                -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta);
}

/**
 *        z↑   ➚
 *         |  /x
 *         | /
 * y        |/       
 * <————————
*/
Quadrotor::State Quadrotor::getState() {
    return state_;
}

