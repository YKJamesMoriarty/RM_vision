/**
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
  * @file       rune.hpp
  * @brief      用于能量机关的相关描述
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

#ifndef RUNE_DETECTOR__ARMOR_HPP_
#define RUNE_DETECTOR__ARMOR_HPP_

#include <opencv2/core.hpp>

// STL
#include <string>

/**
 * @brief 包含了与符文检测相关的结构体和枚举类型的命名空间
 */
namespace rm_rune_detector
{

    /**
     * @brief 靶标状态的类型枚举类
     */
    enum class TargetType
    {
        DISACTIVED, /* 未激活的目标 */
        ACTIVED,    /* 激活的目标 */
        NEGATIVE,   /* 未亮起的靶标 */
        INVALID     /* 无效的目标 */
    };

    const int RED = 0;  /* 红色 */
    const int BLUE = 1; /* 蓝色 */

    const std::string ARMOR_TYPE_STR[3] = {"disactived", "actived", "invalid"}; /* 靶标类型的字符串表示 */

    /**
     * @brief 符文检测中的椭圆结构体，继承自cv::RotatedRect
     */
    struct Ellipse : public cv::RotatedRect
    {
        /**
         * @brief 默认构造函数
         */
        Ellipse() = default;

        /**
         * @brief 构造函数，根据给定的旋转矩形构造椭圆
         * @param box 给定的旋转矩形
         */
        explicit Ellipse(cv::RotatedRect box) : cv::RotatedRect(box)
        {
            // 获取椭圆的主轴和副轴的长度
            major_axis = std::max(box.size.width, box.size.height);
            minor_axis = std::min(box.size.width, box.size.height);
        }

        float major_axis; /* 椭圆的主轴长度 */
        float minor_axis; /* 椭圆的副轴长度 */
        int color;        /* 椭圆的颜色 */
    };

    /**
     * @brief 能量机关检测中的靶标结构体
     */
    struct Target_Image
    {
        Target_Image() = default;
        Target_Image(cv::RotatedRect &rect)
        {
            rectangle = rect;
            center = rect.center;
        }

        cv::RotatedRect rectangle;
        cv::Point2f center;
        TargetType type;
    };

    /**
     * @brief 能量机关检测中的靶标结构体
     * @note 使用旋转角度描述
     */
    struct Target
    {
        Target() = default;
        Target(double angle)
        {
            this->angle = angle;
            this->type = TargetType::NEGATIVE;
        }
        double angle; // Unit: degree
        TargetType type;
    };

    /**
     * @brief 用来描述识别到的R标的外接框的结构体
     */
    struct R_Sign_Rectangle
    {
        R_Sign_Rectangle() = default;
        R_Sign_Rectangle(int x, int y, int w, int h)
        {
            left_top = cv::Point(x, y);
            left_bottom = cv::Point(x, y + h);
            right_top = cv::Point(x + w, y);
            right_bottom = cv::Point(x + w, y + h);
            center = cv::Point(x + w / 2, y + h / 2);
        }
        cv::Point left_top;
        cv::Point left_bottom;
        cv::Point right_top;
        cv::Point right_bottom;
        cv::Point center;
    };

    double DegreesToRadians(double degrees);

    double RadiansToDegrees(double radians);

    double MesureDegreeRange(double degree);

} // namespace rm_rune_detector

#endif // RUNE_DETECTOR__ARMOR_HPP_