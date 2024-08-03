/**
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
  * @file       sim_serial_driver.cpp
  * @brief      仿真虚拟串口通信模块
  * @note       基于原来的串口模块修改，保留自瞄用对于odom2gimbal的tf发布
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2022            ChenJun         1. done
  *  V1.0.1     2023-12-11      Penguin         1. 添加与rm_rune_dector_node模块连接的Client
  *  V1.0.2     2024-3-1        LihanChen       1. 添加导航数据包，并重命名packet和相关函数
  *  V1.0.3     2024-7-19       ZikangXie       1. 修改为仿真用的虚拟串口节点模块
  *
  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
  */

#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/sim_serial_driver.hpp"

namespace sim_serial_driver
{
SimSerialDriver::SimSerialDriver(const rclcpp::NodeOptions & options)
: Node("sim_serial_driver", options),
tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this)),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(*tf_buffer_)
  {
    RCLCPP_INFO(get_logger(), "Start SimSerialDriver!");
    timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);

    auto timer_callback = [this]() {this->publishOdomToGimbalTransform();};
    timer_ = this->create_wall_timer(
      rclcpp::Rate(10).period(), timer_callback); // Set the rate as needed
    detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");

    // Tracker reset service client
    reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("tracker/reset");
    resetTracker();

    while(rclcpp::ok()){

      if (!initial_set_param_) {
                setParam(rclcpp::Parameter("detect_color", 0));
              }
      else{
          resetTracker();
        
      }
    }
    
    // Initialize the TF listener
    // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    // Set up the timer to periodically check for transform updates
    
  }
void SimSerialDriver::publishOdomToGimbalTransform()
  {
    try {
      // Lookup the latest transform between "chassis" and "gimbal_pitch"
      geometry_msgs::msg::TransformStamped transform;
      transform = tf_buffer_->lookupTransform("chassis", "gimbal_pitch", rclcpp::Time(0));

      // Create a new transformStamped message for "odom" to "gimbal_link"
      // odom_to_gimbal.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
      // RCLCPP_INFO(this->get_logger(), "odom_to_gimbal.header.stamp: %s" , std::to_string(odom_to_gimbal.header.stamp.sec).c_str());
      transform.header.frame_id = "odom";
      transform.child_frame_id = "gimbal_link";

      // Broadcast the transform
      tf_broadcaster_->sendTransform(transform);
      RCLCPP_INFO(this->get_logger(), "tf send success");
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
    }
  }

  void SimSerialDriver::getParams()
{

}

void SimSerialDriver::setParam(const rclcpp::Parameter & param)
{
  if (!detector_param_client_->service_is_ready()) {
    // RCLCPP_WARN(get_logger(), "Armor service not ready, skipping parameter set");
    return;
  }

  if (
    !set_param_future_.valid() ||
    set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    RCLCPP_INFO(get_logger(), "Setting armor detect_color to %ld...", param.as_int());

    set_param_future_ = detector_param_client_->set_parameters(
      {param}, [this, param](const ResultFuturePtr & results) {
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set armor detect_color to %ld!", param.as_int());
        initial_set_param_ = true;
      });
  }
}

void SimSerialDriver::resetTracker()
{
  if (!reset_tracker_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_tracker_client_->async_send_request(request);
  RCLCPP_INFO(get_logger(), "Reset tracker!");
}
SimSerialDriver::~SimSerialDriver()
{

}
}  // namespace sim_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(sim_serial_driver::SimSerialDriver)