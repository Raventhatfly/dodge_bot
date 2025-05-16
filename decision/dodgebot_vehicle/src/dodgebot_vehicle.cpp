#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>

#include <behavior_interface/msg/shoot.hpp>
#include <behavior_interface/msg/aim.hpp>
#include <operation_interface/msg/wfly_control.hpp>
#include <operation_interface/msg/io_info.hpp>
#include <vision_interface/msg/auto_aim.hpp>
#include <vision_interface/msg/auto_aim_vel.hpp>

#define PUB_RATE 10 // ms
class DodgebotVehicle : public rclcpp::Node {
public:
    enum GameState {
        INIT,
        EASY,
        HARD,
        DEAD
    };

    DodgebotVehicle(const rclcpp::NodeOptions & options) : Node("dodgebot_vehicle"){
        // double max_vel = this->declare_parameter("control.trans_vel", 2.0);
        // double max_omega = this->declare_parameter("control.rot_vel", 3.0);
        // double aim_sens = this->declare_parameter("control.stick_sens", 1.57);
        // double deadzone = this->declare_parameter("control.deadzone", 0.05);
        yaw_init_ = this->declare_parameter("control.yaw_init", 0.0);
        pitch_init_ = this->declare_parameter("control.pitch_init", 0.0);
        yaw_ = yaw_init_;
        pitch_ = pitch_init_;
        yaw_max_ = this->declare_parameter("control.yaw_max", 0.8);
        yaw_min_ = this->declare_parameter("control.yaw_min", -0.9 );
        pitch_max_ = this->declare_parameter("control.pitch_max", 0.4);
        pitch_min_ = this->declare_parameter("control.pitch_min", -0.3);
        // remote_enable_ = this->declare_parameter("control.remote_enable", true);
        // vision_enable_ = this->declare_parameter("control.vision_enable", false);
        vision_enable_ = false;
        feed_speed_ = this->declare_parameter("control.feed_speed", 1.0);
        std::string shoot_topic = this->declare_parameter("shoot_topic", "shoot");
        std::string aim_topic = this->declare_parameter("aim_topic", "aim");
        std::string remote_topic = this->declare_parameter("remote_topic", "wfly_control");

        // Game Logic Parameters
        max_hit_ = this->declare_parameter("max_hit", 3);

        aim_pub_ = this->create_publisher<behavior_interface::msg::Aim>(aim_topic, 10);
        shoot_pub_ = this->create_publisher<behavior_interface::msg::Shoot>(shoot_topic, 30);
        remote_sub_ = this->create_subscription<operation_interface::msg::WflyControl>(
            remote_topic, 30, std::bind(&DodgebotVehicle::remote_callback, this, std::placeholders::_1));
        // vision_sub_ = this->create_subscription<vision_interface::msg::AutoAim>(
        //     "auto_aim", 10, std::bind(&DodgebotVehicle::vision_callback, this, std::placeholders::_1));
        vision_sub_ = this->create_subscription<vision_interface::msg::AutoAimVel>(
            "auto_aim", 10, std::bind(&DodgebotVehicle::vision_callback, this, std::placeholders::_1));
        io_sub_ = this->create_subscription<operation_interface::msg::IoInfo>(
            "dodgebot_io", 10, std::bind(&DodgebotVehicle::io_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(PUB_RATE), [this](){
                timer_callback();
            });
        
        fric_state_ = false;
        feed_state_ = false;


        times_hit_ = 0;
        prev_armor_button_ = false;
        dodgebot_alive_ = false;
        difficulty_hard_ = false;
        game_state_ = INIT;

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
        if(msg->sa == "down") robot_enable_ = true; else robot_enable_ = false;
        if(robot_enable_){
            if(msg->sd == "down"){
                remote_enable_ = false;
                vision_enable_ = true;
            }else{
                remote_enable_ = true;
                vision_enable_ = false;
            }
        }else{
            remote_enable_ = false;
            vision_enable_ = false;
        }
        

        // Read Game State
        std::string sc = msg->sc;
        if(sc == "up"){
            dodgebot_alive_ = false;
            game_state_ = INIT;
        }else if(!(game_state_ == DEAD)  && sc == "mid"){
            dodgebot_alive_ = true;
            game_state_ = EASY;
            max_hit_ = 1;
        }else if(!(game_state_ == DEAD) && sc == "down"){
            dodgebot_alive_ = true;
            game_state_ = HARD;
            max_hit_ = 2;
        }
        

        if(robot_enable_){
            // Remote Shooter control
            if(!remote_enable_ && !vision_enable_){
                fric_state_ = false;
                feed_state_ = false;
            }
            if(remote_enable_){
                if(msg->sb == "up") {
                    fric_state_ = false;
                    feed_state_ = false;
                }else if(msg->sb == "mid"){
                    fric_state_ = true;
                    feed_state_ = false;
                }else if(msg->sb == "down"){
                    fric_state_ = true;
                    feed_state_ = true;
                }
            }else{   // Vision Shoot Control
                if(dodgebot_alive_){
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
                }else{
                    fric_state_ = false;
                    feed_state_ = false;
                }
                
            }
            
            // Remote Yaw and pitch control
            if(remote_enable_){
                vision_enable_ = false;
                yaw_ += 0.01 * msg->rs_y * 1.4; // 1.4 Requested by Kaiwang
                if(msg->ls_x > 0.01 || msg->ls_x < -0.01)  pitch_ -= 0.02 * msg->ls_x;
            }  
        }else{ // safe mode
            fric_state_ = false;
            feed_state_ = false;
        }      
    }

    void vision_callback(const vision_interface::msg::AutoAimVel::SharedPtr msg) {
        if(vision_enable_ && dodgebot_alive_){
            yaw_ = msg->v_yaw;
            pitch_ = pitch_ + 0.05 * msg->v_pitch;
        }
    }

    void io_callback(const operation_interface::msg::IoInfo::SharedPtr msg) {
        if(msg->armor_button && !prev_armor_button_){
            times_hit_++;
            if (times_hit_ >= max_hit_){
                dodgebot_alive_ = false;
                times_hit_ = 0;
                game_state_ = DEAD;
            }
        }
        prev_armor_button_ = msg->armor_button;
    }

    double yaw_init_, pitch_init_, yaw_, pitch_;
    double yaw_max_, yaw_min_, pitch_max_, pitch_min_;
    bool robot_enable_, remote_enable_, vision_enable_, fric_state_, feed_state_, difficulty_hard_;
    GameState game_state_;
    double feed_speed_;
    int times_hit_; bool prev_armor_button_;
    // Internal state variables
    bool dodgebot_alive_;
    int max_hit_;

    rclcpp::Publisher<behavior_interface::msg::Shoot>::SharedPtr shoot_pub_;
    rclcpp::Publisher<behavior_interface::msg::Aim>::SharedPtr aim_pub_;
    rclcpp::Subscription<operation_interface::msg::WflyControl>::SharedPtr remote_sub_;
    rclcpp::Subscription<vision_interface::msg::AutoAimVel>::SharedPtr vision_sub_;
    rclcpp::Subscription<operation_interface::msg::IoInfo>::SharedPtr io_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(DodgebotVehicle)