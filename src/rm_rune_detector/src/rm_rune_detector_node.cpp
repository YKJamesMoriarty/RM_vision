/**
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
  * @file       rm_rune_detector.cpp
  * @brief      能量机关检测模块
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023-12-13      Penguin         1. done
  *
  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
  */

// ROS
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rm_rune_detector/rune.hpp"
#include "rm_rune_detector/rm_rune_detector_node.hpp"

namespace rm_rune_detector
{
    /**
     * @brief RMRuneDetectorNode 的构造函数。
     * @param options 节点选项。
     */
    RMRuneDetectorNode::RMRuneDetectorNode(const rclcpp::NodeOptions &options)
        : Node("rm_rune_detector", options)
    {
        is_detect_rune_ = true;
        RCLCPP_INFO(this->get_logger(), "Start RMRuneDetectorNode!");

        // Debug Publishers
        debug_ = this->declare_parameter("debug", false);
        if (debug_)
        {
            CreateDebugPublishers();
        }

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

    /**
     * @brief 处理图像，检测能量机关并预测打击位置的回调函数
     * @param img_msg
     */
    void RMRuneDetectorNode::ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
    {
        RCLCPP_INFO(this->get_logger(), "Processing rune image...");
        auto runes = DetectRunes(img_msg);
    }

    /**
     * @brief 识别图中的能量机关靶标
     * @param img_msg
     * @return
     */
    std::vector<Target> RMRuneDetectorNode::DetectRunes(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
    {
        RCLCPP_INFO(this->get_logger(), "Detecting rune...");
        // Convert ROS img to cv::Mat
        auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;
        
        // auto armors = detector_->detect(img);
        std::vector<Target> armors;
        
        if (debug_)
        {
            // binary_img_pub_.publish(
            //     cv_bridge::CvImage(img_msg->header, "mono8", detector_->binary_img).toImageMsg());

            // // Sort lights and armors data by x coordinate
            // std::sort(
            //     detector_->debug_lights.data.begin(), detector_->debug_lights.data.end(),
            //     [](const auto &l1, const auto &l2)
            //     { return l1.center_x < l2.center_x; });
            // std::sort(
            //     detector_->debug_armors.data.begin(), detector_->debug_armors.data.end(),
            //     [](const auto &a1, const auto &a2)
            //     { return a1.center_x < a2.center_x; });

            // lights_data_pub_->publish(detector_->debug_lights);
            // armors_data_pub_->publish(detector_->debug_armors);

            // if (!armors.empty())
            // {
            //     auto all_num_img = detector_->getAllNumbersImage();
            //     number_img_pub_.publish(
            //         *cv_bridge::CvImage(img_msg->header, "mono8", all_num_img).toImageMsg());
            // }

            // detector_->drawResults(img);
            // // Draw camera center
            // cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);
            // // Draw latency
            // std::stringstream latency_ss;
            // latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
            // auto latency_s = latency_ss.str();
            // cv::putText(
            //     img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
            // result_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
        }
        return armors;
    }

    /**
     * @brief 发布调试信息
     */
    void RMRuneDetectorNode::CreateDebugPublishers()
    {
        // lights_data_pub_ =
        //     this->create_publisher<auto_aim_interfaces::msg::DebugLights>("/detector/debug_lights", 10);
        // armors_data_pub_ =
        //     this->create_publisher<auto_aim_interfaces::msg::DebugArmors>("/detector/debug_armors", 10);

        binary_img_pub_ = image_transport::create_publisher(this, "/detector/binary_img");
        number_img_pub_ = image_transport::create_publisher(this, "/detector/number_img");
        result_img_pub_ = image_transport::create_publisher(this, "/detector/result_img");
    }

    RMRuneDetectorNode::~RMRuneDetectorNode()
    {
        RCLCPP_INFO(this->get_logger(), "RuneDetectorNode destroyed!");
    }
}// namespace rm_rune_detector

#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_rune_detector::RMRuneDetectorNode)
