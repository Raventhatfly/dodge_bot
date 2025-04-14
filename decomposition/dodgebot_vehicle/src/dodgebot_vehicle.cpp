#include <rclcpp/rclcpp.hpp>

#include <behavior_interface/msg/shoot.hpp>
#include <behavior_interface/msg/aim.hpp>

#define PUB_RATE 15 // ms
class DodgebotVehicle : public rclcpp::Node {
public:
    DodgebotVehicle(const rclcpp::NodeOptions & options) : Node("dodgebot_vehicle"){
        // double max_vel = this->declare_parameter("control.trans_vel", 2.0);
        // double max_omega = this->declare_parameter("control.rot_vel", 3.0);
        // double aim_sens = this->declare_parameter("control.stick_sens", 1.57);
        // double deadzone = this->declare_parameter("control.deadzone", 0.05);
        yaw_ = this->declare_parameter("control.yaw_init", 0.0);
        pitch_ = this->declare_parameter("control.pitch_init", 0.0);
        std::string shoot_topic = this->declare_parameter("shoot_topic", "shoot");
        std::string aim_topic = this->declare_parameter("aim_topic", "aim");
        aim_pub_ = this->create_publisher<behavior_interface::msg::Aim>(aim_topic, 10);
        shoot_pub_ = this->create_publisher<behavior_interface::msg::Shoot>(shoot_topic, 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(PUB_RATE), [this](){
                timer_callback();
            });

        RCLCPP_INFO(this->get_logger(), "DbusVehicle initialized.");
    }
private:
    void timer_callback() {
        auto aim_msg = std::make_shared<behavior_interface::msg::Aim>();
        auto shoot_msg = std::make_shared<behavior_interface::msg::Shoot>();
        aim_msg->yaw = 0.0;
        aim_msg->pitch = 0.0;
        aim_pub_->publish(*aim_msg);
        shoot_pub_->publish(*shoot_msg);
    }
    double yaw_, pitch_;
    rclcpp::Publisher<behavior_interface::msg::Shoot>::SharedPtr shoot_pub_;
    rclcpp::Publisher<behavior_interface::msg::Aim>::SharedPtr aim_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(DodgebotVehicle)