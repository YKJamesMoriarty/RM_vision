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
        
    }
}