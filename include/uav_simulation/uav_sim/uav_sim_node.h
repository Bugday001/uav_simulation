#ifndef UAV_SIM_NODE_H
#define UAV_SIM_NODE_H
#include "uav_simulation/uav_sim/uav_sim.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>
#include <vector>
#include "uav_controller/uav_controller.h"
#include "simple_traj.h"
#include "params.h"

class QuadrotorSimulator : public rclcpp::Node
{
public:
    QuadrotorSimulator();
 
private:
    void getParams();
    void publishOdometry();
    void rospubTimerCallback();
    void publishTF();
    void ctrl_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void publishPlannerTarget(const std::vector<Eigen::Vector3d>& pva);
    void testCtrl();
    void staticCtrl();
    double m, kf, g;
    Quadrotor quad_;
    Quadrotor::State quad_state_;
    std::shared_ptr<Controller> my_ctrl;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_, plan_odom_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ctrl_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    SimpleTraj simple_traj_;
    double planner_start_time_;
    Params params_;
};

template <typename T>
struct identity { typedef T type; };

template <typename T>
void declare_param(rclcpp::Node* node, const std::string param_name, T& param, const typename identity<T>::type& default_value) {
    node->declare_parameter(param_name, default_value);
    node->get_parameter(param_name, param);
}

#endif