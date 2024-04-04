#include "uav_simulation/uav_sim/uav_sim_node.h"


QuadrotorSimulator::QuadrotorSimulator(): Node("quadrotor_simulator_so3") {
    getParams();
    my_ctrl = std::make_shared<Controller>(params_);
    quad_state_ = quad_.getState();
    m = quad_.getMass();
    g = quad_.getGravity();
    kf = quad_.getCT();

    simple_traj_.setParams(2.5, 2.5);
    Eigen::Vector3d start_point(0, 0, 0);
    Eigen::Vector3d end_point(0, 2, 1);
    std::vector<Eigen::Vector3d> way_points;
    way_points.push_back(start_point);
    way_points.push_back(end_point);
    simple_traj_.setWayPoints(way_points);
    planner_start_time_ = -1;

    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    ctrl_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("ctrl_cmd_", 10, 
                std::bind(&QuadrotorSimulator::ctrl_callback, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, 
                std::bind(&QuadrotorSimulator::goal_callback, this, std::placeholders::_1));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&QuadrotorSimulator::rospubTimerCallback, this));
}


void QuadrotorSimulator::getParams() {
    // Extrinsics
    std::vector<double> k_p_default{8, 8, 11.2};
    std::vector<double> k_v_default{2.5, 2.5, 3.5};
    // center of gravity to imu
    std::vector<double> k_p, k_v, k_R, k_w;
    declare_param(this, "se3/k_p", k_p, k_p_default);
    declare_param(this, "se3/k_v", k_v, k_v_default);
    declare_param(this, "se3/k_R", k_R, k_p_default);
    declare_param(this, "se3/k_w", k_w, k_v_default);
    params_.k_p = Eigen::Vector3d(k_p[0], k_p[1], k_p[2]);
    params_.k_v = Eigen::Vector3d(k_v[0], k_v[1], k_v[2]);
    params_.k_R = Eigen::Vector3d(k_R[0], k_R[1], k_R[2]);
    params_.k_w = Eigen::Vector3d(k_w[0], k_w[1], k_w[2]);
    std::cout<<"k_v: "<<params_.k_v.transpose()<<std::endl;
    std::cout<<"k_v: "<<params_.k_p.transpose()<<std::endl;
    std::cout<<"k_R: "<<params_.k_R.transpose()<<std::endl;
    std::cout<<"k_w: "<<params_.k_w.transpose()<<std::endl;
}

void QuadrotorSimulator::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    Eigen::Vector3d goal_point(msg->pose.position.x, msg->pose.position.y, 1);
    std::vector<Eigen::Vector3d> way_points;
    way_points.push_back(quad_state_.x);
    way_points.push_back(goal_point);
    simple_traj_.setWayPoints(way_points);
    planner_start_time_ = -1;
}

void QuadrotorSimulator::ctrl_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    static double last_time = -1;
    double cur_time = this->get_clock()->now().seconds();
    if(last_time == -1) {
        last_time = cur_time; //this->get_clock()->now().seconds();
        return;
    }
    Eigen::Vector4d w_cmd;
    w_cmd << msg->data[0], msg->data[1], msg->data[2], msg->data[3];
    quad_.setInput(w_cmd);
    quad_.step(cur_time - last_time);
}

void QuadrotorSimulator::rospubTimerCallback() {
    quad_state_ = quad_.getState();
    testCtrl();
    static int cnt = 0;
    if(cnt>2) {
        publishOdometry();
        publishTF();
        cnt = 0;
    }
    cnt++;
}

void QuadrotorSimulator::publishOdometry() {
    auto msg = nav_msgs::msg::Odometry();
    // Fill the message fields with your data
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "world";
    msg.pose.pose.position.x = quad_state_.x(0);
    msg.pose.pose.position.y = quad_state_.x(1);
    msg.pose.pose.position.z = quad_state_.x(2);
    Eigen::Quaterniond quaternion(quad_state_.R);
    quaternion.normalize();
    msg.pose.pose.orientation.x = quaternion.x();
    msg.pose.pose.orientation.y = quaternion.y();
    msg.pose.pose.orientation.z = quaternion.z();
    msg.pose.pose.orientation.w = quaternion.w();
    msg.twist.twist.linear.x = quad_state_.v(0);
    msg.twist.twist.linear.y = quad_state_.v(1);
    msg.twist.twist.linear.z = quad_state_.v(2);
    publisher_->publish(msg);
    // std::cout << quad_state_.x(2) << ", " << quad_state_.v(2) << std::endl;
}
void QuadrotorSimulator::testCtrl() {
    std::vector<Eigen::Vector3d> pva(3, Eigen::Vector3d());
    double cur_sec = this->now().seconds();
    if(planner_start_time_==-1) {
        planner_start_time_ = cur_sec;
    }
    if(simple_traj_.getPVA(pva, cur_sec-planner_start_time_)) {
        // RCLCPP_INFO(this->get_logger(), "Planner finished!");
    }
    std::vector<Eigen::Vector3d> cur_pva;
    cur_pva.push_back(quad_state_.x);
    cur_pva.push_back(quad_state_.v);
    cur_pva.push_back(quad_state_.acc);
    my_ctrl->setState(cur_pva, quad_state_.R, quad_state_.angle_zxy, quad_state_.angle_dot_zxy);
    double dt = 0.01;
    // my_ctrl->controller2D(Eigen::Vector3d(pva[0](1), pva[1](1), pva[2](1)), Eigen::Vector3d(pva[0](2), pva[1](2), pva[2](2)));
    my_ctrl->controllerSE3(pva);
    Eigen::Vector4d motor_rpm = my_ctrl->getRpm();//Eigen::Vector4d(rpm, rpm*1.02, rpm, rpm*1.02);
    quad_.setInput(motor_rpm);
    quad_.step(dt);
    //End for tes
}

void QuadrotorSimulator::publishTF()
  {
    geometry_msgs::msg::TransformStamped transform;
    double seconds = this->now().seconds();
    transform.header.stamp = rclcpp::Time(static_cast<uint64_t>(seconds * 1e9));
    transform.header.frame_id = "world";
    transform.child_frame_id = "base_link";

    transform.transform.translation.x = quad_state_.x(0);
    transform.transform.translation.y = quad_state_.x(1);
    transform.transform.translation.z = quad_state_.x(2);
    Eigen::Quaterniond quaternion(quad_state_.R);
    quaternion.normalize();
    transform.transform.rotation.x = quaternion.x();
    transform.transform.rotation.y = quaternion.y();
    transform.transform.rotation.z = quaternion.z();
    transform.transform.rotation.w = quaternion.w();

    // 广播坐标变换信息
    tf_broadcaster_->sendTransform(transform);
  }

 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<QuadrotorSimulator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}