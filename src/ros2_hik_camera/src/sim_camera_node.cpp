#include "MvCameraControl.h"
// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace sim_camera
{
class SimCameraNode : public rclcpp::Node
{
public:
  explicit SimCameraNode(const rclcpp::NodeOptions & options) : Node("sim_camera", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting SimCameraNode!");
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw");
    
    image_msg_.header.frame_id = "camera_optical_frame";
    // image_msg_.header.frame_id = "camera_link";
    image_msg_.encoding = "rgb8";
    // 创建QoS策略
    rclcpp::QoS custom_qos(1000); // 队列大小为1000
    custom_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    custom_qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    custom_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    // 使用自定义的QoS创建订阅
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "front_camera/image", 
      custom_qos, 
      std::bind(&SimCameraNode::imageCallback, this, std::placeholders::_1));
    
    // Load camera info
    camera_name_ = this->declare_parameter("camera_name", "sim_camera");
    camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    auto camera_info_url =
      this->declare_parameter("camera_info_url", "package://hik_camera/config/sim_camera_info.yaml");
    if (camera_info_manager_->validateURL(camera_info_url)) {
      camera_info_manager_->loadCameraInfo(camera_info_url);
      camera_info_msg_ = camera_info_manager_->getCameraInfo();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }

    params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&SimCameraNode::parametersCallback, this, std::placeholders::_1));
  }

  ~SimCameraNode() override
  {
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    RCLCPP_INFO(this->get_logger(), "SimCameraNode destroyed!");
  }

private:
  void declareParameters()
  {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    MVCC_FLOATVALUE f_value;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;
    // Exposure time
    param_desc.description = "Exposure time in microseconds";
    MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value);
    param_desc.integer_range[0].from_value = f_value.fMin;
    param_desc.integer_range[0].to_value = f_value.fMax;
    double exposure_time = this->declare_parameter("exposure_time", 5000, param_desc);
    MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
    RCLCPP_INFO(this->get_logger(), "Exposure time: %f", exposure_time);

    // Gain
    param_desc.description = "Gain";
    MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value);
    param_desc.integer_range[0].from_value = f_value.fMin;
    param_desc.integer_range[0].to_value = f_value.fMax;
    double gain = this->declare_parameter("gain", f_value.fCurValue, param_desc);
    MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
    RCLCPP_INFO(this->get_logger(), "Gain: %f", gain);
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : parameters) {
      if (param.get_name() == "exposure_time") {
        int status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_int());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set exposure time, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "gain") {
        int status = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set gain, status = " + std::to_string(status);
        }
      } else {
        result.successful = false;
        result.reason = "Unknown parameter: " + param.get_name();
      }
    }
    return result;
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
  {
    image_msg_.header.stamp = img_msg->header.stamp;
    image_msg_.height = img_msg->height;
    image_msg_.width = img_msg->width;
    image_msg_.step = img_msg->step;
    image_msg_.data = img_msg->data;
    camera_info_msg_.header = image_msg_.header;
    camera_pub_.publish(image_msg_, camera_info_msg_);
  }

  sensor_msgs::msg::Image image_msg_;
  image_transport::CameraPublisher camera_pub_;
  // Image subscrpition
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  void * camera_handle_;

  std::string camera_name_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  std::thread capture_thread_;

  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};
}  // namespace sim_camera

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(sim_camera::SimCameraNode)
