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

namespace rm_rune_detector
{
    struct HSV
    {
        int H;
        int S;
        int V;
    };

    struct Rectangle
    {
        Rectangle() = default;
        Rectangle(int rect_x, int rect_y, int rect_w, int rect_h)
        {
            x = rect_x;
            y = rect_y;
            w = rect_w;
            h = rect_h;
        }
        int x, y, w, h;
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

        RuneDetector(const int &bin_thres, const int &color, const TargetParams &t, const HSVParams &hsv);

        std::vector<Target> Detect(const cv::Mat &input);

        int binary_thres;
        int binary_thres_for_R;
        int detect_color;

        // Debug msgs
        cv::Mat binary_img_for_R;
        cv::Mat result_img;

    private:
        cv::Mat PreprocessImageForR(const cv::Mat &rgb_img);
        cv::Point FindRSign(const cv::Mat &binary_img_for_R);
        std::vector<Ellipse> FindPossibleTargets(const cv::Mat &rbg_img, const cv::Mat &binary_img);
        std::vector<Target> FilterTargets(const std::vector<Ellipse> &possible_targets);

        TargetParams t;
        HSVParams hsv;
        std::vector<Ellipse> ellipse_;
        std::vector<Target> targets_;
    };

} // namespace rm_rune_detector

#endif // RUNE_DETECTOR__DETECTOR_HPP_
