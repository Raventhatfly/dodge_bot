#include <chrono>
#include <cmath>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <cv_bridge/cv_bridge.h>
#include <fmt/format.h>
#include <librealsense2/hpp/rs_frame.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <rcl/time.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <librealsense2/rsutil.h>

#include "irmv_detection/armor.hpp"
#include "irmv_detection/irm_detector.hpp"
#include "irmv_detection/magic_enum.hpp"
#include "irmv_detection/yolo_engine.hpp"

constexpr bool ALLOW_DEBUG_AND_PROFILING = true;

namespace irmv_detection
{
IrmDetector::IrmDetector(const rclcpp::NodeOptions & options)
{
  node_ = std::make_shared<rclcpp::Node>("irmv_detector", options);

  // Declare parameters
  declare_parameters();

  // Initialize YOLO engine
  auto model_path =
    ament_index_cpp::get_package_share_directory("irmv_detection") + "/models/yolo11n-pose.onnx";
  for (int i = 0; i < 3; i++) {
    yolo_engines_[i] =
      std::make_unique<YoloEngine>(model_path, image_input_size_, enable_profiling_);
  }

  RCLCPP_INFO(node_->get_logger(), "YOLOEngine initialized");

  auto camera_info_manager =
    std::make_unique<camera_info_manager::CameraInfoManager>(node_.get(), "mv_camera");
  auto camera_info_url = node_->declare_parameter(
    "camera_info_url", std::string("package://irmv_detection/config/camera_info.yaml"));
  if (!camera_info_manager->validateURL(camera_info_url)) {
    RCLCPP_ERROR(node_->get_logger(), "Invalid camera info URL");
    exit(0);
  }
  camera_info_manager->loadCameraInfo(camera_info_url);
  auto camera_info_msg = camera_info_manager->getCameraInfo();
  pnp_solver_ = std::make_unique<PnPSolver>(camera_info_msg.k, camera_info_msg.d);

  RCLCPP_INFO(node_->get_logger(), "PnPSolver initialized");

  // Handle parameter changes
  param_event_handle_ = node_->add_on_set_parameters_callback(
    std::bind_front(&IrmDetector::param_event_callback, this));

  // Initialize publishers and subscribers
  armors_pub_ = node_->create_publisher<auto_aim_interfaces::msg::Armors>(
    "/detector/armors", rclcpp::SensorDataQoS());

  target_pub_ = node_->create_publisher<auto_aim_interfaces::msg::Target>(
    "/detector/target", rclcpp::SensorDataQoS());

  auto_aim_pub_ = node_->create_publisher<vision_interface::msg::AutoAimVel>(
    "auto_aim", 10);
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  if constexpr (ALLOW_DEBUG_AND_PROFILING) {
    create_debug_publishers();
  }

  // Initialize camera
  Camera::Config camera_config = {
    .image_size = image_input_size_,
    .depth_size = cv::Size(640, 480),
    .image_buffers = {
      yolo_engines_[0]->get_src_image_buffer(), yolo_engines_[1]->get_src_image_buffer(),
      yolo_engines_[2]->get_src_image_buffer()}};
  // camera_ = std::make_unique<VirtualCamera>(
  //   camera_config, "/mnt/d/RMUL23_Vision_data/3v3/Italy_Torino_group/video_28.mp4",
  // std::bind_front(&IrmDetector::message_callback, this), 200);
  
  // camera_ = std::make_unique<
  // camera_ = std::make_unique<MVCamera>(
  // camera_config, std::bind_front(&IrmDetector::message_callback, this));

  camera_ = std::make_unique<RealsenseCamera>(
  camera_config, std::bind_front(&IrmDetector::message_callback, this));
}

void IrmDetector::create_debug_publishers()
{
  if (enable_profiling_) {
    total_latency_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
      "/detector/total_latency", rclcpp::SystemDefaultsQoS());
    inference_latency_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
      "/yolo_engine/inference_latency", rclcpp::SystemDefaultsQoS());
    pnp_latency_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
      "/pnp_solver/pnp_latency", rclcpp::SystemDefaultsQoS());
  }
  if (enable_debug_) {
    binary_img_pub_ = image_transport::create_publisher(
      node_.get(), "/image/binary_image", rmw_qos_profile_sensor_data);
    visualized_img_pub_ = image_transport::create_publisher(
      node_.get(), "/image/visualized_image", rmw_qos_profile_sensor_data);
  }
  if (enable_depth_) {
    depth_img_pub_ = image_transport::create_publisher(
      node_.get(), "/image/depth_image", rmw_qos_profile_sensor_data);
  }
  if (enable_rviz_) {
    armor_marker_.ns = "armor";
    armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
    armor_marker_.scale.x = 0.05;
    armor_marker_.scale.z = 0.125;
    armor_marker_.color.a = 1.0;
    armor_marker_.color.g = 0.5;
    armor_marker_.color.b = 1.0;
    armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

    text_marker_.ns = "classification";
    text_marker_.action = visualization_msgs::msg::Marker::ADD;
    text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker_.scale.z = 0.1;
    text_marker_.color.a = 1.0;
    text_marker_.color.r = 1.0;
    text_marker_.color.g = 1.0;
    text_marker_.color.b = 1.0;
    text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

    marker_array_pub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>("/detector/marker", 10);
  }
}

void IrmDetector::declare_parameters()
{
  auto param_desc = rcl_interfaces::msg::ParameterDescriptor();

  param_desc.description = "Enable debug mode";
  param_desc.additional_constraints = "Must be true or false";
  enable_debug_ = node_->declare_parameter<bool>("debug", false, param_desc);

  param_desc.description = "Enable profiling";
  param_desc.additional_constraints = "Must be true or false";
  enable_profiling_ = node_->declare_parameter<bool>("profiling", false, param_desc);
  if (enable_debug_) enable_profiling_ = true;

  param_desc.description = "Enable Rviz visualization";
  param_desc.additional_constraints = "Must be true or false";
  enable_rviz_ = node_->declare_parameter<bool>("rviz", true, param_desc);
  if (enable_debug_) enable_rviz_ = true;

  param_desc.description = "Enable Depth";
  param_desc.additional_constraints = "Must be true or false";
  enable_depth_ = node_->declare_parameter<bool>("depth", true, param_desc);

  param_desc.description = "Input size of the YOLO model";
  param_desc.additional_constraints = "Must be a list of two integers";
  auto image_input_size = node_->declare_parameter<std::vector<long>>(
    "image_input_size", std::vector<long>{640, 480}, param_desc);
  image_input_size_ = cv::Size(image_input_size[0], image_input_size[1]);

  param_desc.description = "Binary threshold for light extraction";
  param_desc.additional_constraints = "Must be an integer ranging from 0 to 255";
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 255;
  param_desc.integer_range[0].step = 1;
  binary_threshold_ = node_->declare_parameter<int>("binary_threshold", 150, param_desc);

  param_desc.description = "Enemy color, 0 for blue, 1 for red";
  param_desc.additional_constraints = "Must be 0 or 1";
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 1;
  param_desc.integer_range[0].step = 1;
  enemy_color_ = node_->declare_parameter<int>("enemy_color", 0, param_desc);

  light_min_ratio_ = node_->declare_parameter<double>("light.min_ratio", 0.1);
  light_max_ratio_ = node_->declare_parameter<double>("light.max_ratio", 0.4);
  light_max_angle_ = node_->declare_parameter<double>("light.max_angle", 40.0);

  armor_min_small_center_distance_ =
    node_->declare_parameter<double>("armor.min_small_center_distance", 0.8);
  armor_max_small_center_distance_ =
    node_->declare_parameter<double>("armor.max_small_center_distance", 3.2);
  armor_min_large_center_distance_ =
    node_->declare_parameter<double>("armor.min_large_center_distance", 3.2);
  armor_max_large_center_distance_ =
    node_->declare_parameter<double>("armor.max_large_center_distance", 5.5);
}

void IrmDetector::message_callback(Camera::StampedImage & image)
{
  rclcpp::Time extraction_end_time;
  rclcpp::Time pnp_end_time;

  std::vector<YoloEngine::bbox> bboxes = yolo_engines_[image.id]->detect();

  if (pnp_solver_ == nullptr) return;  // This could happen if camera_info topic is not received yet

  if constexpr (ALLOW_DEBUG_AND_PROFILING) {
    if (enable_profiling_) extraction_end_time = node_->now();
  }

  rclcpp::Time time_stamp_ros(image.time_stamp.time_since_epoch().count(), RCL_ROS_TIME);
  std_msgs::msg::Header header;
  header.stamp = time_stamp_ros;
  header.frame_id = "camera_optical_frame";
  auto_aim_interfaces::msg::Armors armors_msg;
  armors_msg.header = header;
  if (enable_rviz_) {
    armor_marker_.header = header;
    text_marker_.header = header;
    armor_marker_.id = 0;
    text_marker_.id = 0;
  }

  float target_x = 0.0;
  float target_y = 0.0;
  float min_dist_to_image_center = 10000.0;

  for (const auto & bbox : bboxes) {
    float min_x = std::max(bbox.xyxy[0], 0.0f);
    float min_y = std::max(bbox.xyxy[1], 0.0f);
    float max_x = std::min(bbox.xyxy[2], static_cast<float>(image.image.cols));
    float max_y = std::min(bbox.xyxy[3], static_cast<float>(image.image.rows));

    if (min_x >= max_x || min_y >= max_y) continue;

    float point[3];
    float pixel[2];
    pixel[0] = (min_x + max_x) / 2;
    pixel[1] = (min_y + max_y) / 2;

    // Iterate nose, eyes, ears
    int visible_count = 0;
    float point_x_sum = 0.0;
    float point_y_sum = 0.0;
    for(int i = 0; i < 5; i++){
      if (bbox.key_points[i].x > 0 && bbox.key_points[i].x < image.image.cols &&
          bbox.key_points[i].y > 0 && bbox.key_points[i].y < image.image.rows &&
          bbox.key_points[i].z > 0.7) {
        visible_count++;
        point_x_sum += bbox.key_points[i].x;
        point_y_sum += bbox.key_points[i].y;
      }
    }
    if(visible_count > 0){
      pixel[0] = point_x_sum / visible_count;
      pixel[1] = point_y_sum / visible_count;
    }

    rs2::depth_frame depth_frame = image.depth.as<rs2::depth_frame>();
    float depth_val = depth_frame.get_distance(pixel[0], pixel[1]);

    rs2_intrinsics intrinsics = camera_->get_intrinsics();
    rs2_deproject_pixel_to_point(point, &intrinsics, pixel, depth_val);

    auto_aim_interfaces::msg::Armor armor_msg;

    // Fill in pose
    armor_msg.pose.position.x = point[0];
    armor_msg.pose.position.y = point[1];
    armor_msg.pose.position.z = point[2];

    tf2::Quaternion tf2_quat;
    armor_msg.pose.orientation = tf2::toMsg(tf2_quat);

    // Fill in message
    armor_msg.distance_to_image_center = cv::norm<float>(cv::Point2f(pixel[0], pixel[1]) - cv::Point2f(
      float(image.image.cols) / 2, float(image.image.rows) / 2));

    if (armor_msg.distance_to_image_center < min_dist_to_image_center) {
      min_dist_to_image_center = armor_msg.distance_to_image_center;
      target_x = pixel[0];
      target_y = pixel[1];
    }

    armors_msg.armors.emplace_back(armor_msg);

    if (enable_rviz_) {
      armor_marker_.id++;
      armor_marker_.pose = armor_msg.pose;
      armor_marker_.scale.y = 0.23;
      text_marker_.id++;
      text_marker_.pose.position = armor_msg.pose.position;
      text_marker_.pose.position.y -= 0.1;
      text_marker_.text = "PERSON";
      marker_array_.markers.push_back(armor_marker_);
      marker_array_.markers.push_back(text_marker_);
    }
  }

  auto_aim_interfaces::msg::Target target_msg;
  target_msg.header = header;
  target_msg.tracking = false;
  float min_distance_to_image_center = 10000.0;
  geometry_msgs::msg::PoseStamped ps;
  ps.header = header;
  if (tf2_buffer_->canTransform("base", ps.header.frame_id, ps.header.stamp) && armors_msg.armors.size() > 0) {
    target_msg.tracking = true;
    for (auto & armor : armors_msg.armors) {
      if (armor.distance_to_image_center < min_distance_to_image_center) {
        min_distance_to_image_center = armor.distance_to_image_center;
        ps.pose = armor.pose;
        target_msg.position = tf2_buffer_->transform(ps, "base").pose.position;
        target_msg.velocity.x = 0.0;
        target_msg.velocity.y = 0.0;
        target_msg.velocity.z = 0.0;
      }
    }
  } else {
    // std::cout << "Temporary failure to transform" << std::endl;
  }

  if (armors_msg.armors.size() > 0) {
    vision_interface::msg::AutoAimVel auto_aim_msg;
    auto_aim_msg.v_yaw = - (target_x - float(image.image.cols) / 2) / (float(image.image.cols) / 2) * 0.785 * 0.02;
    auto_aim_msg.v_pitch = - (target_y - float(image.image.rows) / 2) / float(image.image.rows) * 2.0;
    auto_aim_pub_->publish(auto_aim_msg);
  }

  // int armor_i = 0;
  // std::cout << "===================" << std::endl;
  // for (auto & armor : armors_msg.armors) {
  //   std::cout << "Armor " << armor_i << ": " << armor.pose.position.x << ", "
  //             << armor.pose.position.y << ", " << armor.pose.position.z << std::endl;
  //   armor_i++;
  // }
  // std::cout << "===================" << std::endl;

  armors_pub_->publish(armors_msg);
  target_pub_->publish(target_msg);
  if(enable_depth_){
      cv::Mat depth_image(
          cv::Size(640, 480),
          CV_16UC1,
          (void*)image.depth.get_data(),
          cv::Mat::AUTO_STEP);
      depth_img_pub_.publish(
        cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1, depth_image)
          .toImageMsg());
  }
  if constexpr (ALLOW_DEBUG_AND_PROFILING) {
    if (enable_profiling_) {
      pnp_end_time = node_->now();
      // Publish profiling data
      const auto inference_time = yolo_engines_[image.id]->get_profiling_time();
      std_msgs::msg::Float64 total_latency_msg, inference_latency_msg, pnp_latency_msg;
      total_latency_msg.data = (pnp_end_time - time_stamp_ros).seconds() * 1000;
      total_latency_pub_->publish(total_latency_msg);
      inference_latency_msg.data = inference_time;
      inference_latency_pub_->publish(inference_latency_msg);
      pnp_latency_msg.data = (pnp_end_time - extraction_end_time).seconds() * 1000;
      pnp_latency_pub_->publish(pnp_latency_msg);
      // Publish debug images
      if (enable_debug_) {
        cv::Mat visualized_image = image.image.clone();
        // visualize_armors(visualized_image, armors);
        yolo_engines_[image.id]->visualize_bboxes(visualized_image, bboxes);
        cv::putText(
          visualized_image, fmt::format("Total latency: {} ms", total_latency_msg.data),
          cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        cv::putText(
          visualized_image, fmt::format("Inference latency: {} ms", inference_latency_msg.data),
          cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        visualized_img_pub_.publish(
          cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", visualized_image).toImageMsg());

        cv::Mat binary_image = image.image.clone();
        cv::cvtColor(binary_image, binary_image, cv::COLOR_BGR2GRAY);
        cv::threshold(binary_image, binary_image, binary_threshold_, 255, cv::THRESH_BINARY);
        cv::cvtColor(binary_image, binary_image, cv::COLOR_GRAY2BGR);
        yolo_engines_[image.id]->visualize_bboxes(binary_image, bboxes);
        binary_img_pub_.publish(
          cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", binary_image).toImageMsg());
      }
    }
    if (enable_rviz_) {
      armor_marker_.action = armors_msg.armors.empty() ? visualization_msgs::msg::Marker::DELETE
                                                       : visualization_msgs::msg::Marker::ADD;
      marker_array_.markers.push_back(armor_marker_);
      marker_array_pub_->publish(marker_array_);
      marker_array_.markers.clear();
    }
  }
}

void IrmDetector::visualize_armors(cv::Mat & image, const std::vector<Armor> & armors) const
{
  for (const auto & armor : armors) {
    cv::circle(image, armor.left_light.top, 5, cv::Scalar(0, 255, 0), 2);
    cv::circle(image, armor.left_light.bottom, 5, cv::Scalar(0, 255, 0), 2);
    cv::circle(image, armor.right_light.top, 5, cv::Scalar(0, 255, 0), 2);
    cv::circle(image, armor.right_light.bottom, 5, cv::Scalar(0, 255, 0), 2);

    cv::line(image, armor.left_light.top, armor.left_light.bottom, cv::Scalar(0, 255, 0), 2);
    cv::line(image, armor.right_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 2);
    cv::line(image, armor.left_light.top, armor.right_light.top, cv::Scalar(0, 255, 0), 2);
    cv::line(image, armor.left_light.bottom, armor.right_light.bottom, cv::Scalar(0, 255, 0), 2);
  }
}

rcl_interfaces::msg::SetParametersResult IrmDetector::param_event_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  for (const auto & parameter : parameters) {
    std::string name = parameter.get_name();
    if (name == "debug") {
      enable_debug_ = parameter.as_bool();
    } else if (name == "binary_threshold") {
      binary_threshold_ = static_cast<int>(parameter.as_int());
    } else if (name == "enemy_color") {
      enemy_color_ = static_cast<int>(parameter.as_int());
    } else if (name == "light.min_ratio") {
      light_min_ratio_ = parameter.as_double();
    } else if (name == "light.max_ratio") {
      light_max_ratio_ = parameter.as_double();
    } else if (name == "light.max_angle") {
      light_max_angle_ = parameter.as_double();
    } else if (name == "armor.min_small_center_distance") {
      armor_min_small_center_distance_ = parameter.as_double();
    } else if (name == "armor.max_small_center_distance") {
      armor_max_small_center_distance_ = parameter.as_double();
    } else if (name == "armor.min_large_center_distance") {
      armor_min_large_center_distance_ = parameter.as_double();
    } else if (name == "armor.max_large_center_distance") {
      armor_max_large_center_distance_ = parameter.as_double();
    }
  }
  return result;
}
}  // namespace irmv_detection

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(irmv_detection::IrmDetector)
