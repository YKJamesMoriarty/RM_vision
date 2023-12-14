/**
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
  * @file       rune_detector.cpp
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
// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

// STD
#include <algorithm>
#include <cmath>
#include <vector>

#include "rm_rune_detector/rune_detector.hpp"
#include "rm_rune_detector/rune.hpp"

namespace rm_rune_detector
{
    /**
     * @brief RuneDetector类的构造函数
     * @param bin_thres 二值化阈值
     * @param color 检测颜色
     * @param t 靶标识别相关参数
     * @param hsv 红色和蓝色HSV颜色空间阈值
     */
    RuneDetector::RuneDetector(const int &bin_thres, const int &color, const TargetParams &t, const HSVParams &hsv)
        : binary_thres(bin_thres), detect_color(color), t(t), hsv(hsv)
    {
        this->binary_thres_for_R = 100;
    }

    /**
     * @brief 检测函数，用于检测输入图像中的目标
     * @param input 输入图像
     * @return 检测到的靶标列表
     */
    std::vector<Target> RuneDetector::Detect(const cv::Mat &input)
    {
        result_img = input.clone();
        // TODO:完成能量机关靶标的检测与状态判别
        binary_img_for_R = PreprocessImageForR(input);
        std::vector<Target> res_tmp;
        targets_ = res_tmp;
        return targets_;
    }

    /**
     * @brief 对输入图像进行预处理用来寻找R标
     * @param rgb_img 输入图像
     * @return binary_img_for_R
     */
    cv::Mat RuneDetector::PreprocessImageForR(const cv::Mat &rgb_img)
    {
        cv::Mat gray_img;
        cv::cvtColor(rgb_img, gray_img, cv::COLOR_RGB2GRAY);

        cv::Mat binary_img_for_R;
        cv::threshold(gray_img, binary_img_for_R, binary_thres_for_R, 255, cv::THRESH_BINARY);

        return binary_img_for_R;
    }

    cv::Point RuneDetector::FindRSign(const cv::Mat &binary_img_for_R)
    {
        // 寻找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary_img_for_R, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 寻找R标
        std::vector<Rectangle> rectangles;
        for (size_t i = 0; i < contours.size(); i++)
        {
            if (cv::contourArea(contours[i]) < 100 || cv::contourArea(contours[i]) > 1000)
                continue;

            // 拟合矩形
            cv::Rect rect = cv::boundingRect(contours[i]);
            if (abs(static_cast<double>(rect.width) / rect.height - 1) > 0.1)
                continue;

            Rectangle rectangle(rect.x, rect.y, rect.width, rect.height);
            rectangles.push_back(rectangle);
        }
        // 对筛选出来的矩形按照面积进行升序排序
        std::sort(rectangles.begin(), rectangles.end(), [](const Rectangle &a, const Rectangle &b)
                  { return a.w * a.h < b.w * b.h; });

        cv::Point rotation_center;
        Rectangle R_sign;
        for (size_t i = 0; i < rectangles.size(); i++)
        {
            int x = rectangles[i].x;
            int y = rectangles[i].y;
            int w = rectangles[i].w;
            int h = rectangles[i].h;
            cv::Mat rectangle_frame = binary_img_for_R(cv::Rect(x, y, w, h));

            // 归一化数组
            cv::Mat normalized_rectangle_frame;
            rectangle_frame.convertTo(normalized_rectangle_frame, CV_32F, 1.0 / 255);

            double total = cv::sum(normalized_rectangle_frame)[0];
            int total_elements = normalized_rectangle_frame.total();
            if (total / total_elements < 0.5 || total_elements > 3000)
                continue;

            rotation_center = cv::Point(x + w / 2, y + h / 2);
            R_sign = rectangles[i];
            cv::rectangle(result_img, cv::Point(x, y), cv::Point(x + w, y + h), cv::Scalar(255, 255, 0), 2);
            cv::putText(result_img, "R", cv::Point(x, y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 2);
        }
        return rotation_center;
    }

    /**
     * @brief 在输入图像中寻找可能的靶标
     * @param rbg_img rgb图像
     * @param binary_img 二值化图像
     * @return 可能的靶标列表
     */
    std::vector<Ellipse> RuneDetector::FindPossibleTargets(const cv::Mat &rbg_img, const cv::Mat &binary_img)
    {
        using std::vector;
        vector<vector<cv::Point>> contours;
        vector<cv::Vec4i> hierarchy;
        cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        return vector<Ellipse>();
    }
} // namespace rm_rune_detector
