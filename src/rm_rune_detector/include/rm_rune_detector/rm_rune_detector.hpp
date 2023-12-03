

#ifndef RM_RUNE_DETECTOR__RM_RUNE_DETECTOR_HPP_
#define RM_RUNE_DETECTOR__RM_RUNE_DETECTOR_HPP_

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace rm_rune_detector
{
    class RMRuneDetector : public rclcpp::Node
    {
    public:
        explicit RMRuneDetector(const rclcpp::NodeOptions &options);

        ~RMRuneDetector() override;

    private:
    };
} // namespace rm_rune_detector

#endif // RM_RUNE_DETECTOR__RM_RUNE_DETECTOR_HPP_