/**
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
  * @file       rune.hpp
  * @brief      用于能量机关的相关描述
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

#ifndef RUNE_DETECTOR__ARMOR_HPP_
#define RUNE_DETECTOR__ARMOR_HPP_

#include <opencv2/core.hpp>

// STL
#include <algorithm>
#include <string>

namespace rm_rune_detector
{
    const int RED = 0;
    const int BLUE = 1;

    enum class ArmorType
    {
        SMALL,
        LARGE,
        INVALID
    };
    const std::string ARMOR_TYPE_STR[3] = {"small", "large", "invalid"};

    struct Light : public cv::RotatedRect
    {
        Light() = default;
        explicit Light(cv::RotatedRect box) : cv::RotatedRect(box)
        {
            cv::Point2f p[4];
            box.points(p);
            std::sort(p, p + 4, [](const cv::Point2f &a, const cv::Point2f &b)
                      { return a.y < b.y; });
            top = (p[0] + p[1]) / 2;
            bottom = (p[2] + p[3]) / 2;

            length = cv::norm(top - bottom);
            width = cv::norm(p[0] - p[1]);

            tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
            tilt_angle = tilt_angle / CV_PI * 180;
        }

        int color;
        cv::Point2f top, bottom;
        double length;
        double width;
        float tilt_angle;
    };

    struct Target
    {
        Target() = default;
        Target(const Light &l1, const Light &l2)
        {
            if (l1.center.x < l2.center.x)
            {
                left_light = l1, right_light = l2;
            }
            else
            {
                left_light = l2, right_light = l1;
            }
            center = (left_light.center + right_light.center) / 2;
        }

        // Light pairs part
        Light left_light, right_light;
        cv::Point2f center;
        ArmorType type;

        // Number part
        cv::Mat number_img;
        std::string number;
        float confidence;
        std::string classfication_result;
    };

} // namespace rm_rune_detector

#endif // RUNE_DETECTOR__ARMOR_HPP_