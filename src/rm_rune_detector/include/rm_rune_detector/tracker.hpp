/**
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
  * @file       tracker.hpp
  * @brief      能量机关跟踪模块
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

#ifndef RM_RUNE_DETECTOR__TRACKER_HPP_
#define RM_RUNE_DETECTOR__TRACKER_HPP_

// C++ system
#include <vector>

#include "rm_rune_detector/rune.hpp"

namespace rm_rune_detector
{
    class Tracker
    {
    public:
        Tracker(double delta_angle_limit);
        ~Tracker() = default;

        int Update(const std::vector<double> &angles);

        void Reset();

        std::vector<Target> targets_;
    private:
        // std::vector<Target> last_targets_;
        bool is_tracking_ = false;
        int target_id_ = -1;//打击目标的id (0,1,2,3,4)
        double delta_angle_limit_;
    };
} // namespace rm_rune_detector

#endif // RM_RUNE_DETECTOR__TRACKER_HPP_