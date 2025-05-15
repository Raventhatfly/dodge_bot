#include <rclcpp/rclcpp.hpp>

#include <behavior_interface/msg/shoot.hpp>
#include <behavior_interface/msg/aim.hpp>
#include <operation_interface/msg/wfly_control.hpp>
#include <vision_interface/msg/auto_aim.hpp>
#include <vision_interface/msg/auto_aim_vel.hpp>

#define PUB_RATE 10 // ms
class DodgebotVehicle : public rclcpp::Node {
public:
    DodgebotVehicle(const rclcpp::NodeOptions & options) : Node("dodgebot_vehicle"){
        // double max_vel = this->declare_parameter("control.trans_vel", 2.0);
        // double max_omega = this->declare_parameter("control.rot_vel", 3.0);
        // double aim_sens = this->declare_parameter("control.stick_sens", 1.57);
        // double deadzone = this->declare_parameter("control.deadzone", 0.05);
        yaw_ = this->declare_parameter("control.yaw_init", 0.0);
        pitch_ = this->declare_parameter("control.pitch_init", 0.0);
        yaw_max_ = this->declare_parameter("control.yaw_max", 0.8);
        yaw_min_ = this->declare_parameter("control.yaw_min", -0.9 );
        pitch_max_ = this->declare_parameter("control.pitch_max", 0.4);
        pitch_min_ = this->declare_parameter("control.pitch_min", -0.3);
        remote_enable_ = this->declare_parameter("control.remote_enable", true);
        // vision_enable_ = this->declare_parameter("control.vision_enable", false);
        vision_enable_ = false;
        feed_speed_ = this->declare_parameter("control.feed_speed", 1.0);
        std::string shoot_topic = this->declare_parameter("shoot_topic", "shoot");
        std::string aim_topic = this->declare_parameter("aim_topic", "aim");
        std::string remote_topic = this->declare_parameter("remote_topic", "wfly_control");
        aim_pub_ = this->create_publisher<behavior_interface::msg::Aim>(aim_topic, 10);
        shoot_pub_ = this->create_publisher<behavior_interface::msg::Shoot>(shoot_topic, 30);
        remote_sub_ = this->create_subscription<operation_interface::msg::WflyControl>(
            remote_topic, 30, std::bind(&DodgebotVehicle::remote_callback, this, std::placeholders::_1));
        // vision_sub_ = this->create_subscription<vision_interface::msg::AutoAim>(
        //     "auto_aim", 10, std::bind(&DodgebotVehicle::vision_callback, this, std::placeholders::_1));
        vision_sub_ = this->create_subscription<vision_interface::msg::AutoAimVel>(
            "auto_aim", 10, std::bind(&DodgebotVehicle::vision_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(PUB_RATE), [this](){
                timer_callback();
            });
        
        fric_state_ = false;
        feed_state_ = false;

        RCLCPP_INFO(this->get_logger(), "Dodgebot initialized.");
    }
private:
    void timer_callback() {
        auto aim_msg = std::make_shared<behavior_interface::msg::Aim>();
        auto shoot_msg = std::make_shared<behavior_interface::msg::Shoot>();
        shoot_msg->fric_state = fric_state_;
        shoot_msg->feed_state = feed_state_;
        shoot_msg->feed_speed = feed_speed_;
        if(yaw_ > yaw_max_) yaw_ = yaw_max_;
        if(yaw_ < yaw_min_) yaw_ = yaw_min_;
        if(pitch_ > pitch_max_) pitch_ = pitch_max_;
        if(pitch_ < pitch_min_) pitch_ = pitch_min_;
        aim_msg->yaw = yaw_;
        aim_msg->pitch = pitch_;
        aim_pub_->publish(*aim_msg);
        shoot_pub_->publish(*shoot_msg);
    }

    void remote_callback(const operation_interface::msg::WflyControl::SharedPtr msg) {
        if(remote_enable_){
            // Shooter control
            if(msg->sd == "up"){
                vision_enable_ = false;
            }else{
                vision_enable_ = true;
                return;
            }

            // Shooter control
            if(msg->sa == "up") {
                fric_state_ = false;
                feed_state_ = false;
            }else if(msg->sb == "mid"){
                fric_state_ = true;
                feed_state_ = false;
            }else if(msg->sb == "down"){
                fric_state_ = true;
                feed_state_ = true;
            }
            
            // Yaw and pitch control
            if(msg->sa == "down"){
                vision_enable_ = false;
                yaw_ += 0.01 * msg->ls_y;
                pitch_ += 0.02 * msg->ls_x;
            }  
        }      
    }

    void vision_callback(const vision_interface::msg::AutoAimVel::SharedPtr msg) {
        if(vision_enable_){
            yaw_ = msg->v_yaw;
            pitch_ = pitch_ + 0.05 * msg->v_pitch;
        }
    }

    double yaw_, pitch_;
    double yaw_max_, yaw_min_, pitch_max_, pitch_min_;
    bool remote_enable_, vision_enable_, fric_state_, feed_state_;
    double feed_speed_;
    rclcpp::Publisher<behavior_interface::msg::Shoot>::SharedPtr shoot_pub_;
    rclcpp::Publisher<behavior_interface::msg::Aim>::SharedPtr aim_pub_;
    rclcpp::Subscription<operation_interface::msg::WflyControl>::SharedPtr remote_sub_;
    rclcpp::Subscription<vision_interface::msg::AutoAimVel>::SharedPtr vision_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(DodgebotVehicle)