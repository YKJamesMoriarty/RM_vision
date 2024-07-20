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


#ifndef RM_SERIAL_DRIVER__SIM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__SIM_SERIAL_DRIVER_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <auto_aim_interfaces/msg/target.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>

// C++ system

#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace sim_serial_driver
{
class SimSerialDriver : public rclcpp::Node
{
public:
  explicit SimSerialDriver(const rclcpp::NodeOptions & options);

  virtual ~SimSerialDriver() override;

private:
  void publishOdomToGimbalTransform();
  void getParams();
  void setParam(const rclcpp::Parameter & param);
  void resetTracker();


  // Param client to set detect_colr
  using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
  bool initial_set_param_ = false;
  uint8_t previous_receive_color_ = 0;
  rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
  ResultFuturePtr set_param_future_;

  double timestamp_offset_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace sim_serial_driver

#endif  // RM_SERIAL_DRIVER__SIM_SERIAL_DRIVER_HPP_
