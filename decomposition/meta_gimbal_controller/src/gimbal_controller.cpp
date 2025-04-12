#include "gimbal_controller/gimbal_controller.hpp"

#include <control_toolbox/pid_ros.hpp>
#include <limits>
#include <memory>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>

#include "angles/angles.h"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

using ControllerReferenceMsg =
    gimbal_controller::GimbalController::ControllerReferenceMsg;

void reset_controller_reference_msg(
    const std::shared_ptr<ControllerReferenceMsg> &msg,
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & /*node*/) {
    msg->yaw = NaN;
    msg->pitch = NaN;
}

using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;

namespace gimbal_controller {

controller_interface::CallbackReturn GimbalController::on_init() {

    try {
        param_listener_ = std::make_shared<gimbal_controller::ParamListener>(get_node());
    } catch (const std::exception &e) {
        std::cerr << "Exception thrown during controller's init with message: "
                  << e.what() << std::endl;
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GimbalController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
    params_ = param_listener_->get_params();

    // topics QoS
    auto subscribers_qos = rclcpp::SystemDefaultsQoS();
    subscribers_qos.keep_last(1);
    subscribers_qos.best_effort();

    // Reference Subscriber
    ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
        "~/reference", subscribers_qos,
        std::bind(&GimbalController::reference_callback, this, std::placeholders::_1));

    auto msg = std::make_shared<ControllerReferenceMsg>();
    reset_controller_reference_msg(msg, get_node());
    input_ref_.writeFromNonRT(msg);

    try {
        // State publisher
        s_publisher_ = get_node()->create_publisher<ControllerStateMsg>(
            "~/controller_state", rclcpp::SystemDefaultsQoS());
        state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
    } catch (const std::exception &e) {
        std::cerr << "Exception thrown during publisher creation at configure "
                     "stage with message: "
                  << e.what() << std::endl;
        return controller_interface::CallbackReturn::ERROR;
    }

    state_publisher_->lock();
    state_publisher_->msg_.dof_states.resize(4);
    state_publisher_->msg_.dof_states[0].name = "yaw_ref";
    state_publisher_->msg_.dof_states[1].name = "pitch_ref";
    state_publisher_->unlock();

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
GimbalController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type =
        controller_interface::interface_configuration_type::INDIVIDUAL;

    command_interfaces_config.names.reserve(2);
    command_interfaces_config.names.push_back(params_.yaw_gimbal_joint.name + "/" +
                                              HW_IF_POSITION);
    command_interfaces_config.names.push_back(params_.pitch_gimbal_joint.name + "/" +
                                              HW_IF_POSITION);

    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
GimbalController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type =
        controller_interface::interface_configuration_type::INDIVIDUAL;

    // IMU feedback comes from ROS2 topic

    return state_interfaces_config;
}

void GimbalController::reference_callback(
    const std::shared_ptr<ControllerReferenceMsg> msg) {
    input_ref_.writeFromNonRT(msg);
}


std::vector<hardware_interface::CommandInterface>
GimbalController::on_export_reference_interfaces() {
    reference_interfaces_.resize(2, NaN);

    std::vector<hardware_interface::CommandInterface> reference_interfaces;
    reference_interfaces.reserve(reference_interfaces_.size());

    reference_interfaces.emplace_back(get_node()->get_name(),
                                      std::string("yaw/") + HW_IF_POSITION,
                                      &reference_interfaces_[0]);

    reference_interfaces.emplace_back(get_node()->get_name(),
                                      std::string("pitch/") + HW_IF_POSITION,
                                      &reference_interfaces_[1]);

    return reference_interfaces;
}

bool GimbalController::on_set_chained_mode(bool chained_mode) {
    // Always accept switch to/from chained mode
    return true || chained_mode;
}

controller_interface::CallbackReturn
GimbalController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
    // Set default value in command
    reset_controller_reference_msg(*(input_ref_.readFromRT()), get_node());

    reference_interfaces_.assign(reference_interfaces_.size(), NaN);

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GimbalController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type GimbalController::update_reference_from_subscribers() {
    auto current_ref = *(input_ref_.readFromRT()); // A shared_ptr must be allocated
                                                   // immediately to prevent dangling

    if (!std::isnan(current_ref->yaw) && !std::isnan(current_ref->pitch)) {
        reference_interfaces_[0] = current_ref->yaw;
        reference_interfaces_[1] = current_ref->pitch;

        current_ref->yaw = NaN;
        current_ref->pitch = NaN;
    }

    return controller_interface::return_type::OK;
}

controller_interface::return_type
GimbalController::update_and_write_commands(const rclcpp::Time &time,
                                            const rclcpp::Duration &period) {
    double yaw_pos_ref = reference_interfaces_[0]; 
    double pitch_pos_ref = reference_interfaces_[1];                                            

    // Publish state
    if (state_publisher_ && state_publisher_->trylock()) {
        state_publisher_->msg_.header.stamp = time;
        state_publisher_->msg_.dof_states[0].reference = yaw_pos_ref;
        state_publisher_->msg_.dof_states[1].reference = pitch_pos_ref;
        state_publisher_->unlockAndPublish();
    }

    command_interfaces_[0].set_value(yaw_pos_ref);
    command_interfaces_[1].set_value(pitch_pos_ref);

    return controller_interface::return_type::OK;
}

} // namespace gimbal_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(gimbal_controller::GimbalController,
                       controller_interface::ChainableControllerInterface)
