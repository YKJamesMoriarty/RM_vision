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
#include "rm_rune_detector/pnp_solver.hpp"

namespace rm_rune_detector
{
    /**
     * @brief RuneDetector类的构造函数
     * @param bin_thres 二值化阈值
     * @param color 检测颜色
     * @param t 靶标识别相关参数
     * @param hsv 红色和蓝色HSV颜色空间阈值
     */
    RuneDetector::RuneDetector(
        const int &bin_thres, const int &color,
        const TargetParams &t, const HSVParams &hsv)
        : binary_thres(bin_thres), detect_color(color),
          t(t), hsv(hsv)
    {
        this->binary_thres_for_R = 100;
        this->__rotation_radius__ = 216;
    }

    /**
     * @brief 检测函数，用于检测输入图像中的R标
     * @param input 输入图像
     * @return 检测到的靶标列表
     */
    R_Sign_Rectangle RuneDetector::DetectRSign(const cv::Mat &input)
    {
        result_img = input.clone();
        cv::Point center(result_img.cols / 2, result_img.rows / 2); // 获取图像的中心
        cv::Scalar color(255, 0, 0);                                // 定义圆的颜色（BGR格式）
        cv::circle(result_img, center, 4, color, 2);                // 绘制圆

        // TODO:完成能量机关靶标的检测与状态判别
        binary_img_for_R = PreprocessImageForR(input);

        R_Sign_Rectangle R_sign_rect = FindRSign(binary_img_for_R);

        return R_sign_rect;
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

    /**
     * @brief 寻找图中的R标
     * @param binary_img_for_R
     * @return R标的矩形参数
     */
    R_Sign_Rectangle RuneDetector::FindRSign(const cv::Mat &binary_img_for_R)
    {
        bool R_sign_found = false;
        // 寻找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary_img_for_R, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 拟合图像中的轮廓为矩形
        std::vector<cv::Rect> rectangles;
        for (size_t i = 0; i < contours.size(); i++)
        {
            if (cv::contourArea(contours[i]) < 100 || cv::contourArea(contours[i]) > 1000)
                continue;

            // 拟合矩形
            cv::Rect rect = cv::boundingRect(contours[i]);
            if (abs(static_cast<double>(rect.width) / rect.height - 1) > 0.1)
                continue;

            rectangles.push_back(rect);
        }
        // 对筛选出来的矩形按照面积进行升序排序
        std::sort(rectangles.begin(), rectangles.end(), [](const cv::Rect &a, const cv::Rect &b)
                  { return a.width * a.height < b.width * b.height; });

        // cv::Point R_sign_center;
        R_Sign_Rectangle R_sign_rect;
        for (size_t i = 0; i < rectangles.size(); i++)
        {
            if (R_sign_found)
                break;

            int x = rectangles[i].x;
            int y = rectangles[i].y;
            int w = rectangles[i].width;
            int h = rectangles[i].height;
            cv::Mat rectangle_frame = binary_img_for_R(cv::Rect(x, y, w, h));

            // 归一化数组
            cv::Mat normalized_rectangle_frame;
            rectangle_frame.convertTo(normalized_rectangle_frame, CV_32F, 1.0 / 255);

            double total = cv::sum(normalized_rectangle_frame)[0];
            int total_elements = normalized_rectangle_frame.total();
            if (total / total_elements < 0.5 || total_elements > 3000)
                continue;

            // R_sign_center = cv::Point(x + w / 2, y + h / 2);
            R_sign_rect = R_Sign_Rectangle(x, y, w, h);
            cv::rectangle(result_img, cv::Point(x, y), cv::Point(x + w, y + h), cv::Scalar(255, 255, 0), 2);
            cv::putText(result_img, "R", cv::Point(x, y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 2);

            R_sign_found = true;
        }
        return R_sign_rect;
    }

    /**
     * @brief 基于对R标的检测结果，对图像进行进一步的预处理用来寻找能量机关靶标
     * @param binary_img 经过部分处理的二值化图像
     * @return binary_img_for_targets 用于识别靶标的二值化图像
     */
    cv::Mat RuneDetector::PreprocessImageForTargets(const cv::Mat &binary_img)
    {
        // TODO:实现自适应旋转半径
        cv::Mat binary_img_for_targets = binary_img.clone();

        // 将旋转半径内部部分进行填充，便于后续的靶标
        // cv::circle(binary_img_for_targets,
        //            rotation_center_, __rotation_radius__ - 60,
        //            cv::Scalar(0, 0, 0), -1); // 填充圆

        cv::Mat mask = cv::Mat::zeros(binary_img.size(), binary_img.type()); // 创建一个全黑的掩码
        // cv::circle(mask,
        //            rotation_center_, __rotation_radius__ + 70,
        //            cv::Scalar(255, 255, 255), -1); // 在掩码上画一个白色的圆

        // 将掩码应用到图像上
        cv::bitwise_and(binary_img_for_targets, mask, binary_img_for_targets);

        // 闭运算
        cv::Mat kernel = cv::Mat::ones(7, 7, CV_8U);
        cv::morphologyEx(binary_img_for_targets, binary_img_for_targets, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 2);

        // 寻找轮廓
        // std::vector<std::vector<cv::Point>> contours;
        // std::vector<cv::Vec4i> hierarchy;
        // cv::findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 显示二值图像
        // cv::imshow("binary", binary);
        return binary_img_for_targets;
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
