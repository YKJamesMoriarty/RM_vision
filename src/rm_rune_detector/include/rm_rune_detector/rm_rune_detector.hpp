

#ifndef RM_RUNE_DETECTOR__RM_RUNE_DETECTOR_HPP_
#define RM_RUNE_DETECTOR__RM_RUNE_DETECTOR_HPP_

// ROS
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "rm_rune_detector/pnp_solver.hpp"
namespace rm_rune_detector
{
    class RMRuneDetectorNode : public rclcpp::Node
    {
    public:
        explicit RMRuneDetectorNode(const rclcpp::NodeOptions &options);

        ~RMRuneDetectorNode() override;

    private:
        void ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
        bool is_detect_rune_;

        // Image subscrpition
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

        // Camera info part
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
        cv::Point2f cam_center_;
        std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
        std::unique_ptr<PnPSolver> pnp_solver_;
    };
} // namespace rm_rune_detector

#endif // RM_RUNE_DETECTOR__RM_RUNE_DETECTOR_HPP_