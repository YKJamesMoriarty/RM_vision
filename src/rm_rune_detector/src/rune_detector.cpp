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
#include <opencv2/calib3d.hpp>

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
        : binary_thresh(bin_thres), detect_color(color),
          t(t), hsv(hsv)
    {
        this->binary_thresh = 80;
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
        cv::threshold(gray_img, binary_img_for_R, binary_thresh, 255, cv::THRESH_BINARY);

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

            R_sign_rect = R_Sign_Rectangle(x, y, w, h);
            cv::rectangle(result_img, cv::Point(x, y), cv::Point(x + w, y + h), cv::Scalar(255, 255, 0), 2);
            cv::putText(result_img, "R", cv::Point(x, y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 2);

            R_sign_found = true;
        }
        return R_sign_rect;
    }

    /**
     * @brief 识别图中的能量机关靶标
     * @param rotation_center 相机光心坐标系下的旋转中心的3维坐标
     * @param tvec 相机光心坐标系下的旋转中心的平移向量
     * @param camera_matrix 相机内参矩阵
     * @param dist_coeffs 相机畸变系数
     * @return 靶标列表
     */
    std::vector<Target> RuneDetector::DetectTargets(
        const cv::Point3d &rotation_center, const cv::Mat &tvec,
        const std::array<double, 9> &camera_matrix,
        const std::vector<double> &dist_coeffs)
    {
        // 先将相机参数转换为Mat类型，用于opencv投影函数的计算
        cv::Mat camera_matrix_mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data()));
        cv::Mat dist_coeffs_mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data()));

        // 首先，将旋转中心和对应的旋转半径参考点投影到图像上获取在图像中的旋转半径像素个数
        //  Unit: m
        std::vector<cv::Point3f> object_points = {
            cv::Point3f(rotation_center.x, rotation_center.y, rotation_center.z),
            cv::Point3f(rotation_center.x + 1.4, rotation_center.y, rotation_center.z),
            cv::Point3f(rotation_center.x, rotation_center.y + 1.4, rotation_center.z),
            cv::Point3f(rotation_center.x, rotation_center.y, rotation_center.z + 1.4)};
        cv::Vec3d rvec(0.0, 0.0, 0.0);         // 创建一个无旋转的旋转向量
        std::vector<cv::Point2f> image_points; // 创建一个空的2D点向量，用来储存投影后的2D点
        // 计算2D点的坐标
        cv::projectPoints(object_points, rvec, tvec, camera_matrix_mat, dist_coeffs_mat, image_points);

        // 画一下旋转圆的效果
        cv::circle(result_img,
                   image_points[0], 30,
                   cv::Scalar(0, 255, 255), -1);
        cv::circle(result_img,
                   image_points[1], 20,
                   cv::Scalar(255, 0, 255), -1);
        cv::circle(result_img,
                   image_points[2], 10,
                   cv::Scalar(255, 0, 255), -1);
        cv::circle(result_img,
                   image_points[3], 10,
                   cv::Scalar(255, 0, 255), -1);
        // 计算旋转半径，这里选用x轴和y轴上的参考点的投影与旋转中心连线的长度的均值作为旋转半径
        cv::Point p0 = image_points[0]; // 旋转中心在图像上的投影点
        cv::Point px = image_points[1]; // x轴上的参考点在图像上的投影点
        cv::Point py = image_points[2]; // y轴上的参考点在图像上的投影点
        double r1 = cv::norm(px - p0);
        double r2 = cv::norm(py - p0);
        int r = int((r1 + r2) / 2);

        // 进一步处理图像，用于寻找能量机关靶标
        binary_img_for_targets = PreprocessImageForTargets(
            binary_img_for_R,
            image_points[0], r);

        std::vector<Target> targets;
        return targets;
    }

    /**
     * @brief 基于对R标的检测结果，对图像进行进一步的预处理用来寻找能量机关靶标
     * @param binary_img 经过部分处理的二值化图像
     * @return binary_img_for_targets 用于识别靶标的二值化图像
     */
    cv::Mat RuneDetector::PreprocessImageForTargets(
        const cv::Mat &binary_img,
        const cv::Point2i &rotation_center,
        const int rotation_radius)
    {
        // TODO:实现自适应旋转半径
        cv::Mat binary_img_for_targets = binary_img.clone();
        // 计算内外旋转圆环半径
        int rotation_radius_min = int(rotation_radius * 0.75);
        int rotation_radius_max = int(rotation_radius * 1.25);
        // 将旋转半径内部部分进行填充，便于后续的靶标
        cv::circle(binary_img_for_targets,
                   rotation_center, rotation_radius_min,
                   cv::Scalar(0, 0, 0), -1); // 填充圆

        cv::Mat mask = cv::Mat::zeros(binary_img.size(), binary_img.type()); // 创建一个全黑的掩码
        cv::circle(mask,
                   rotation_center, rotation_radius_max,
                   cv::Scalar(255, 255, 255), -1); // 在掩码上画一个白色的圆

        // 将掩码应用到图像上
        cv::bitwise_and(binary_img_for_targets, mask, binary_img_for_targets);

        // 闭运算
        cv::Mat kernel = cv::Mat::ones(9, 9, CV_8U);
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
