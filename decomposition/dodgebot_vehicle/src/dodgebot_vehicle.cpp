#include <rclcpp/rclcpp.hpp>

#include <behavior_interface/msg/shoot.hpp>

class DodgebotVehicle : public rclcpp::Node {
public:
    DodgebotVehicle(const rclcpp::NodeOptions & options) : Node("dodgebot_vehicle"){
        // double max_vel = this->declare_parameter("control.trans_vel", 2.0);
        // double max_omega = this->declare_parameter("control.rot_vel", 3.0);
        // double aim_sens = this->declare_parameter("control.stick_sens", 1.57);
        // double deadzone = this->declare_parameter("control.deadzone", 0.05);
        std::string shoot_topic = this->declare_parameter("shoot_topic", "shoot");
        shoot_pub_ = this->create_publisher<behavior_interface::msg::Shoot>(shoot_topic, 10);
    }
private:
    rclcpp::Publisher<behavior_interface::msg::Shoot>::SharedPtr shoot_pub_;

};


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(DodgebotVehicle)