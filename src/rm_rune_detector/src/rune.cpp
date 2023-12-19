/**
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
  * @file       rune.cpp
  * @brief      用于能量机关的相关描述
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023-12-20      Penguin         1. done
  *
  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
  */

#include "rm_rune_detector/rune.hpp"

namespace rm_rune_detector
{

    // 角度转弧度
    double DegreesToRadians(double degrees)
    {
        return degrees * M_PI / 180.0;
    }

    // 弧度转角度
    double RadiansToDegrees(double radians)
    {
        return radians * 180.0 / M_PI;
    }

    /**
     * @brief 确保角度范围在-180到180之间
     * @param degree
     * @return
     */
    double MesureDegreeRange(double degree)
    {
        if (degree > 180)
            degree -= 360;
        else if (degree < -180)
            degree += 360;
        return degree;
    }
} // namespace rm_rune_detector