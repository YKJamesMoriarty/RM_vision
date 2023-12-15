
#ifndef RUNE_DETECTOR__PNP_SOLVER_HPP_
#define RUNE_DETECTOR__PNP_SOLVER_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <opencv2/core.hpp>

// STD
#include <array>
#include <vector>

#include "rm_rune_detector/rune.hpp"
namespace rm_rune_detector
{
    class PnPSolver
    {
    public:
        PnPSolver(
            const std::array<double, 9> &camera_matrix,
            const std::vector<double> &distortion_coefficients);

        // Get 3d position
        bool solvePnP(const R_Sign_Rectangle &R_sign_rect, cv::Mat &rvec, cv::Mat &tvec);

    private:
        cv::Mat camera_matrix_;
        cv::Mat dist_coeffs_;

        // Unit: mm
        static constexpr float R_SIGN_WIDTH = 74;
        static constexpr float R_SIGN_HEIGHT = 74;

        // Four vertices of armor in 3d
        std::vector<cv::Point3f> R_sign_points_;
    };

} // namespace rm_auto_aim

#endif // RUNE_DETECTOR__PNP_SOLVER_HPP_
