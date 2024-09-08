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

#include <kdl/utilities/utility.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
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
#include <future>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/sim_serial_driver.hpp"
#include <vision_interfaces/msg/robot.hpp>


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

    // Create Publisher
    latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("latency", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("aiming_point", 10);
    robot_pub_ = this->create_publisher<vision_interfaces::msg::Robot>("serial_driver/robot", 10);

    // 使用异步回调来替代定时器
    timer_future_ = std::async(std::launch::async, &SimSerialDriver::timerTask, this);
    
    detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");

    // Tracker reset service client
    reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("tracker/reset");
    resetTracker();

    while(rclcpp::ok()){

      if (!initial_set_param_) {
                setParam(rclcpp::Parameter("detect_color", 0));
              }
      else{
          // resetTracker();
        
      }
    }
  
    
  }

void SimSerialDriver::timerTask()
{
    while (rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        publishOdomToGimbalTransform();


    }
}

void SimSerialDriver::publishOdomToGimbalTransform()
{
  // RCLCPP_INFO(this->get_logger(), "publishOdomToGimbalTransform called");

  try {
    // Lookup the latest transform between "chassis" and "gimbal_pitch"
    geometry_msgs::msg::TransformStamped transform;
    transform = tf_buffer_->lookupTransform("chassis", "gimbal_pitch", rclcpp::Time(0));
    // RCLCPP_INFO(this->get_logger(), "lookupTransform success");

    // Create a new transformStamped message for "odom" to "gimbal_link"
    transform.header.frame_id = "gimbal_link";
    transform.child_frame_id = "camera_link";
    // If the transform does not exist, proceed to publish
    tf_broadcaster_->sendTransform(transform);

    // Change the transform to euler angles
    tf2::Quaternion quat;
    tf2::fromMsg(transform.transform.rotation, quat);
    tf2::Matrix3x3 mat(quat);
    double robot_roll, robot_pitch, robot_yaw;
    mat.getRPY(robot_roll, robot_pitch, robot_yaw);

    // RCLCPP_INFO(this->get_logger(), "tf send success");
    // RCLCPP_INFO(this->get_logger(), "Pitch: %f, Yaw: %f", robot_pitch, robot_yaw);

    // Publish the robot message
    robot_msg.muzzle_speed = 20;
    robot_msg.self_pitch = robot_pitch*180/M_PI;
    robot_msg.self_yaw = robot_yaw*180/M_PI;
    robot_pub_->publish(robot_msg);


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
    if (timer_future_.valid()) {
        timer_future_.wait();
    }
}
}  // namespace sim_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(sim_serial_driver::SimSerialDriver)