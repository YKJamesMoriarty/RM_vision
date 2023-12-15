/**
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
  * @file       rm_rune_detector_node.hpp
  * @brief      能量机关检测模块
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023-12-11      Penguin         1. done
  *
  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
  */

#ifndef RM_RUNE_DETECTOR__RM_RUNE_DETECTOR_HPP_
#define RM_RUNE_DETECTOR__RM_RUNE_DETECTOR_HPP_

// ROS
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/publisher.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rm_rune_detector/pnp_solver.hpp"
#include "rm_rune_detector/rune.hpp"
#include "rm_rune_detector/rune_detector.hpp"
namespace rm_rune_detector
{
    class RMRuneDetectorNode : public rclcpp::Node
    {
    public:
        explicit RMRuneDetectorNode(const rclcpp::NodeOptions &options);

        ~RMRuneDetectorNode() override;

    private:
        void ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

        std::vector<Target> DetectRunes(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);

        std::unique_ptr<RuneDetector> InitDetector();

        void CreateDebugPublishers();
        void PublishMarkers();
        void DestroyDebugPublishers();

        bool is_detect_rune_;

        // Rune Detector
        std::unique_ptr<RuneDetector> detector_;

        // Image subscrpition
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

        // Camera info part
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
        cv::Point2f cam_center_;
        std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;

        // PnPSolver part;
        std::unique_ptr<PnPSolver> pnp_solver_;

        // Debug information
        bool debug_;
        std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
        // rclcpp::Publisher<auto_aim_interfaces::msg::DebugLights>::SharedPtr lights_data_pub_;
        // rclcpp::Publisher<auto_aim_interfaces::msg::DebugArmors>::SharedPtr armors_data_pub_;
        image_transport::Publisher binary_img_for_R_pub_;
        image_transport::Publisher binary_img_for_targets_pub_;
        image_transport::Publisher result_img_pub_;

        // Visualization marker publisher
        visualization_msgs::msg::Marker R_sign_marker_;
        visualization_msgs::msg::MarkerArray R_sign_array_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr R_sign_pub_;
    
        //Rune imfomation
        cv::Point3d R_sign_pose;//R标的位置
        std::vector<cv::Point3d> target_pose;//靶标的位置

    };
} // namespace rm_rune_detector

#endif // RM_RUNE_DETECTOR__RM_RUNE_DETECTOR_HPP_