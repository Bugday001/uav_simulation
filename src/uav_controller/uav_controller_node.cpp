#include "rclcpp/rclcpp.hpp"
#include "uav_controller/uav_controller.h"
#include "std_msgs/msg/float64_multi_array.hpp"
 
class Float64Publisher : public rclcpp::Node
{
public:
    Float64Publisher()
    : Node("float64_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("ctrl_cmd_", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Float64Publisher::publish_message, this));
    }
 
private:
    void publish_message()
    {
        auto message = std_msgs::msg::Float64MultiArray();
        message.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        message.layout.dim[0].size = 4;
        message.layout.dim[0].stride = 4;
        message.layout.dim[0].label = "array";
        double rpm = 557.142;
        message.data = {rpm, rpm, rpm, rpm};  // Example float64 array
 
        publisher_->publish(message);
    }
 
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    size_t count_;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Float64Publisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}