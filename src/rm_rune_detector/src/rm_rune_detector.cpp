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
    RMRuneDetector::RMRuneDetector(const rclcpp::NodeOptions &options)
        : Node("rm_serial_driver", options)
    {
        RCLCPP_INFO(this->get_logger(), "Start RuneDetector!");
        detect_rune_thread_ = std::thread(std::bind(&RMRuneDetector::DetectRune, this));
    }

    void RMRuneDetector::DetectRune()
    {
        while (rclcpp::ok())
        {
            RCLCPP_INFO(this->get_logger(), "Detecting rune...");
        }
    }

    RMRuneDetector::~RMRuneDetector()
    {
        if (detect_rune_thread_.joinable())
        {
            detect_rune_thread_.join();
        }
        RCLCPP_INFO(this->get_logger(), "RuneDetectorNode destroyed!");
    }
}

#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_rune_detector::RMRuneDetector)
