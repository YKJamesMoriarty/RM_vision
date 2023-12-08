/**
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
  * @file       rm_rune_detector.cpp
  * @brief      能量机关检测模块
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023-12-13       Penguin         1. done
  *
  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
  */

// #include <tf2/LinearMath/Quaternion.h>

// #include <rclcpp/logging.hpp>
// #include <rclcpp/qos.hpp>
// #include <rclcpp/utilities.hpp>
// #include <serial_driver/serial_driver.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rm_rune_detector/rm_rune_detector.hpp"

namespace rm_rune_detector
{
    RMRuneDetectorNode::RMRuneDetectorNode(const rclcpp::NodeOptions &options)
        : Node("rm_rune_detector", options)
    {
        is_detect_rune_ = true;
        RCLCPP_INFO(this->get_logger(), "Start RMRuneDetectorNode!");

        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_info", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info)
            {
                cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
                cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
                pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
                cam_info_sub_.reset();
            });

        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&RMRuneDetectorNode::ImageCallback, this, std::placeholders::_1));
    }

    void RMRuneDetectorNode::ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
    {
        RCLCPP_INFO(this->get_logger(), "Processing rune image...");
    }

    RMRuneDetectorNode::~RMRuneDetectorNode()
    {
        RCLCPP_INFO(this->get_logger(), "RuneDetectorNode destroyed!");
    }
}

#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_rune_detector::RMRuneDetectorNode)
