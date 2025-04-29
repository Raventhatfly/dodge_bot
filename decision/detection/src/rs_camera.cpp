#include "irmv_detection/camera.hpp"
#include <functional>
#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/rs.hpp>
#include <chrono>
#include <fmt/format.h>

namespace irmv_detection
{
RealsenseCamera::RealsenseCamera(const Config & config, const CameraCallback & callback)
: config_(config), camera_callback_(callback)
{
  // Configure and start RealSense pipeline
  rs_config_.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 60);
  rs_config_.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_Z16,60);

  try {
    pipe_.start(rs_config_);
  } catch (const rs2::error & e) {
    throw invalid_camera_error(fmt::format("RealSense error: {}", e.what()));
  }

  // Wait for the first frame to verify the configuration
  rs2::frameset frames = pipe_.wait_for_frames();
  rs2::frame color_frame = frames.get_color_frame();
  rs2::frame depth_frame = frames.get_depth_frame();
  if (!depth_frame) {
    throw invalid_camera_error("Failed to get depth frame");
  }else{
    depth_frame_width_ = depth_frame.as<rs2::video_frame>().get_width();
    depth_frame_height_ = depth_frame.as<rs2::video_frame>().get_height();
  }
  if (!color_frame) {
    throw invalid_camera_error("Failed to get color frame");
  }
  else{
    color_frame_width_ = color_frame.as<rs2::video_frame>().get_width();
    color_frame_height_ = color_frame.as<rs2::video_frame>().get_height();
  }

  // Initialize triple buffer
  for (int i = 0; i < 3; i++) {
    stamped_img_buf_[i].image = cv::Mat(
      config_.image_size, CV_8UC3, config_.image_buffers[i]);
    stamped_img_buf_[i].id = i;
  }
  
  triple_buffer_ = std::make_unique<TripleBuffer<StampedImage>>(stamped_img_buf_);
  // depth_buffer_ = std::make_shared<cv::Mat>(
  //   cv::Size(depth_frame_width_, depth_frame_height_), CV_16UC1, depth_frame.get_data());
  receive_thread_ = std::jthread(&RealsenseCamera::receive_thread, this);
}

void RealsenseCamera::receive_thread()
{
  namespace chrono = std::chrono;
  int frame_count = 0;
  auto starting_time = chrono::system_clock::now();

  while (!shutdown_) {
    auto start_time = chrono::system_clock::now();
    auto stamped_image = triple_buffer_->get_producer_buffer();
    
    // Wait for frames
    rs2::frameset frames = pipe_.wait_for_frames();
    rs2::frame color_frame = frames.get_color_frame();
    rs2::frame depth_frame = frames.get_depth_frame();
    
    if (color_frame) {
      // Convert to OpenCV format and copy to buffer
      cv::Mat color(
        cv::Size(config_.image_size.width, config_.image_size.height),
        CV_8UC3,
        (void*)color_frame.get_data(),
        cv::Mat::AUTO_STEP);
      
      color.copyTo(stamped_image->image);
      stamped_image->time_stamp = start_time;
      triple_buffer_->producer_commit();
      
      // Process callback
      camera_callback_(*triple_buffer_->get_consumer_buffer());
      
      frame_count++;
      if (frame_count == 100) {
        auto cur_time = chrono::system_clock::now();
        fmt::print(
          "RealSense FPS: {}\n", 
          100 / (chrono::duration<double>(cur_time - starting_time).count()));
        starting_time = cur_time;
        frame_count = 0;
      }
    }
    if (depth_frame) {
      // Convert to OpenCV format and copy to buffer
      cv::Mat depth(
        cv::Size(depth_frame_width_, depth_frame_height_),
        CV_16UC1,
        (void*)depth_frame.get_data(),
        cv::Mat::AUTO_STEP);
      depth_buffer_ = std::make_shared<cv::Mat>(depth.clone());
    }
  }
}

std::shared_ptr<cv::Mat> RealsenseCamera::get_depth_frame()
{
  return depth_buffer_;
}

RealsenseCamera::~RealsenseCamera()
{
  shutdown_ = true;
  pipe_.stop();
  triple_buffer_->producer_commit(); // Wake up consumer thread
}
}