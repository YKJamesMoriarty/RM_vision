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
        std::vector<Target> DetectTargets(
            const cv::Point3d &rotation_center, const cv::Mat &tvec,
            const std::array<double, 9> &camera_matrix,
            const std::vector<double> &dist_coeffs);

        int binary_thresh;
        int detect_color;

        // Debug msgs
        cv::Mat binary_img_for_R;
        cv::Mat binary_img_for_targets;
        cv::Mat result_img;

        //调试时输出的数据，用于查看效果

    private:
        cv::Mat PreprocessImageForR(const cv::Mat &rgb_img);
        R_Sign_Rectangle FindRSign(const cv::Mat &binary_img_for_R);
        cv::Mat PreprocessImageForTargets(const cv::Mat &binary_img,
                                          const cv::Point2i &rotation_center, const int rotation_radius);
        std::vector<Ellipse> FindPossibleTargets(const cv::Mat &rbg_img, const cv::Mat &binary_img);
        std::vector<Target> FilterTargets(const std::vector<Ellipse> &possible_targets);

        TargetParams t;
        HSVParams hsv;
        std::vector<Ellipse> ellipse_;
        std::vector<Target> targets_;
    };

} // namespace rm_rune_detector

#endif // RUNE_DETECTOR__DETECTOR_HPP_
