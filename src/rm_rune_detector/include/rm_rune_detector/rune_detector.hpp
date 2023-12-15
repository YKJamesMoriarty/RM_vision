/**
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
  * @file       rune_detector.hpp
  * @brief      能量机关检测模块检测图片中的能量机关靶标
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023-12-11      Penguin
  *
  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
  */
#ifndef RUNE_DETECTOR__DETECTOR_HPP_
#define RUNE_DETECTOR__DETECTOR_HPP_

// OpenCV
#include <opencv2/core.hpp>

#include "rm_rune_detector/rune.hpp"
#include "rm_rune_detector/pnp_solver.hpp"

namespace rm_rune_detector
{
    struct HSV
    {
        int H;
        int S;
        int V;
    };

    class RuneDetector
    {
    public:
        struct TargetParams
        {
            // minor_axis / major_axis
            float min_ratio;
            float max_ratio;
        };

        struct HSVParams
        {
            HSV red_min;
            HSV red_max;
            HSV blue_min;
            HSV blue_max;
        };

        RuneDetector(
            const int &bin_thres, const int &color,
            const TargetParams &t, const HSVParams &hsv);

        R_Sign_Rectangle DetectRSign(const cv::Mat &input);

        int binary_thres;
        int binary_thres_for_R;
        int detect_color;

        // Debug msgs
        cv::Mat binary_img_for_R;
        cv::Mat binary_img_for_targets;
        cv::Mat result_img;



    private:
        cv::Mat PreprocessImageForR(const cv::Mat &rgb_img);
        R_Sign_Rectangle FindRSign(const cv::Mat &binary_img_for_R);
        cv::Mat PreprocessImageForTargets(const cv::Mat &binary_img_for_R);
        std::vector<Ellipse> FindPossibleTargets(const cv::Mat &rbg_img, const cv::Mat &binary_img);
        std::vector<Target> FilterTargets(const std::vector<Ellipse> &possible_targets);

        // cv::Point rotation_center_;
        // R_Sign_Rectangle R_sign_rect_;
        int __rotation_radius__; // 在8米距离下的大致旋转半径
        TargetParams t;
        HSVParams hsv;
        std::vector<Ellipse> ellipse_;
        std::vector<Target> targets_;
    };

} // namespace rm_rune_detector

#endif // RUNE_DETECTOR__DETECTOR_HPP_
