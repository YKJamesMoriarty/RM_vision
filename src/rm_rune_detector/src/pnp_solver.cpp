/**
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
  * @file       pnp_solver.cpp
  * @brief      能量机关检测模块PNP解算
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
#include "rm_rune_detector/pnp_solver.hpp"

#include <opencv2/calib3d.hpp>
#include <vector>

namespace rm_rune_detector
{
    PnPSolver::PnPSolver(
        const std::array<double, 9> &camera_matrix, const std::vector<double> &dist_coeffs)
        : camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
          dist_coeffs_(cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone())
    {
        // Unit: m
        constexpr double R_sign_half_y = R_SIGN_WIDTH / 2.0 / 1000.0;
        constexpr double R_sign_half_z = R_SIGN_HEIGHT / 2.0 / 1000.0;

        // Start from bottom left in clockwise order
        // Model coordinate: x forward, y left, z up
        R_sign_points_.emplace_back(cv::Point3f(0, R_sign_half_y, -R_sign_half_z));
        R_sign_points_.emplace_back(cv::Point3f(0, R_sign_half_y, R_sign_half_z));
        R_sign_points_.emplace_back(cv::Point3f(0, -R_sign_half_y, R_sign_half_z));
        R_sign_points_.emplace_back(cv::Point3f(0, -R_sign_half_y, -R_sign_half_z));
    }

    /**
     * @brief 对R标进行PnP解算
     * @param[in] R_sign_rect [in]R标在图像中的外接矩形
     * @param[out] rvec [out]旋转向量
     * @param[out] tvec [out]平移向量
     */
    bool PnPSolver::SolvePnP_RSign(const R_Sign_Rectangle &R_sign_rect,
                                   cv::Mat &rvec,
                                   cv::Mat &tvec)
    {
        std::vector<cv::Point2f> image_armor_points;

        // Fill in image points
        image_armor_points.emplace_back(R_sign_rect.left_bottom);
        image_armor_points.emplace_back(R_sign_rect.left_top);
        image_armor_points.emplace_back(R_sign_rect.right_top);
        image_armor_points.emplace_back(R_sign_rect.right_bottom);
        // Solve pnp
        return cv::solvePnP(
            R_sign_points_, image_armor_points, camera_matrix_, dist_coeffs_, rvec, tvec, false,
            cv::SOLVEPNP_IPPE);
    }

    bool PnPSolver::SolvePnP_Target(const R_Sign_Rectangle &R_sign_rect,
                                    const Target_Image &target,
                                    cv::Mat &rvec,
                                    cv::Mat &tvec)
    {
        // std::vector<cv::Point2f> image_armor_points;

        // // Fill in image points
        // image_armor_points.emplace_back(R_sign_rect.left_bottom);
        // image_armor_points.emplace_back(R_sign_rect.left_top);
        // image_armor_points.emplace_back(R_sign_rect.right_top);
        // image_armor_points.emplace_back(R_sign_rect.right_bottom);
        // // Solve pnp
        // return cv::solvePnP(
        //     R_sign_points_, image_armor_points, camera_matrix_, dist_coeffs_, rvec, tvec, false,
        //     cv::SOLVEPNP_IPPE);
        return false;
    }

} // namespace rm_rune_detector
