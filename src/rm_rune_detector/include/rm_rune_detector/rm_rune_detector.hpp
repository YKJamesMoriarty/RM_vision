

#ifndef RM_RUNE_DETECTOR__RM_RUNE_DETECTOR_HPP_
#define RM_RUNE_DETECTOR__RM_RUNE_DETECTOR_HPP_

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

namespace rm_rune_detector
{
    class RMRuneDetector : public rclcpp::Node
    {
    public:
        explicit RMRuneDetector(const rclcpp::NodeOptions &options);

        ~RMRuneDetector() override;

    private:
        void DetectRune();
        std::thread detect_rune_thread_;
    };
} // namespace rm_rune_detector

#endif // RM_RUNE_DETECTOR__RM_RUNE_DETECTOR_HPP_