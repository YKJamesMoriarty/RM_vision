/**
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
  * @file       tracker.cpp
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

#include "rm_rune_detector/tracker.hpp"
#include "rm_rune_detector/rune.hpp"

namespace rm_rune_detector
{
    Tracker::Tracker(double delta_angle_limit)
    {
        this->delta_angle_limit_ = delta_angle_limit;
        Tracker::Reset();
    }

    void Tracker::Reset()
    {
        // Target target_0((-180));
        // Target target_1((-108));
        // Target target_2((-36));
        // Target target_3((36));
        // Target target_4((108));
        is_tracking_ = false;
        target_id_ = -1;
        // last_targets_.push_back(Target((-180)));
        // last_targets_.push_back(Target((-108)));
        // last_targets_.push_back(Target((-36)));
        // last_targets_.push_back(Target((36)));
        // last_targets_.push_back(Target((108)));
        targets_.push_back(Target((-180)));
        targets_.push_back(Target((-108)));
        targets_.push_back(Target((-36)));
        targets_.push_back(Target((36)));
        targets_.push_back(Target((108)));
    }

    /**
     * @brief 更新跟踪器
     * @param angles 检测到的所有目标的角度向量
     * @return 打击目标的id (0,1,2,3,4)
     * @note 如果没有检测到目标，返回-1
     */
    int Tracker::Update(const std::vector<double> &angles)
    {
        int res = -1;
        if (is_tracking_ && angles.empty())
        {
            res = target_id_;
            return res;
        }
        else if (angles.empty()) // 没有目标时，直接返回
        {
            return res;
        }

        int angle_num = angles.size();
        if (angle_num == 1) // 当只有一个目标时，直接更新
        {
            // TODO:加入位置滤波
            targets_[0].angle = angles[0];
            if (targets_[0].type == TargetType::NEGATIVE)
                targets_[0].type = TargetType::DISACTIVED;
            targets_[1].angle = MesureDegreeRange(angles[0] + 72);
            targets_[2].angle = MesureDegreeRange(angles[0] + 144);
            targets_[3].angle = MesureDegreeRange(angles[0] + 216);
            targets_[4].angle = MesureDegreeRange(angles[0] + 288);

            // TODO:计算位置差分
        }
        else // 当有多个目标时
        {
            std::vector<bool> is_updated(angle_num, false); // 用于记录angles向量中的目标是否已经被更新
            // 先匹配id=0的目标
            int index = 0; // 用于记录angles向量中与targets_[0]匹配的目标的下标
            for (int i = 0; i < angle_num; i++)
            {
                if (abs(angles[i] - targets_[0].angle) < delta_angle_limit_) // 若旋转角度差小于delta_angle_limit_度
                {
                    is_updated[0] = true;
                    targets_[0].angle = angles[i];
                    if (targets_[0].type == TargetType::DISACTIVED)
                        targets_[0].type = TargetType::ACTIVED;
                    targets_[1].angle = MesureDegreeRange(angles[i] + 72);
                    targets_[2].angle = MesureDegreeRange(angles[i] + 144);
                    targets_[3].angle = MesureDegreeRange(angles[i] + 216);
                    targets_[4].angle = MesureDegreeRange(angles[i] + 288);
                    index = i;
                    break;
                }
            }
            // 再依次匹配angles向量中的其他目标
            int p = 1;         // traget向量中的位置
            int q = (index + 1) % angle_num; // angles向量中的位置
            bool have_new_target = false;
            while (p < 5 || q != index)
            {
                if (abs(angles[q] - targets_[p].angle) < delta_angle_limit_) // 若旋转角度差小于delta_angle_limit_度
                {
                    targets_[p].angle = (angles[q] + targets_[p].angle) / 2; // 使用平均值更新角度
                    if (targets_[p].type == TargetType::NEGATIVE)            // 出现新目标
                    {
                        have_new_target = true;
                        targets_[p].type = TargetType::DISACTIVED;
                        res = p;
                    }
                    is_updated[p] = true;
                    q = (q + 1) % angle_num;
                }
                p++;
            }

            // TODO:对未更新的目标进行处理

            if (res == -1) // 如果没有出现新目标，返回上一次打击的目标的id
            {
                res = target_id_;
            }

            if (have_new_target) // 如果出现新目标，更新各个目标的激活状态
            {
                for (int i = 0; i < 5; i++)
                {
                    if (i != res && targets_[i].type == TargetType::DISACTIVED)
                        targets_[i].type = TargetType::ACTIVED;
                }
            }
            // else // 没有出现新目标不需要更新激活状态
            // {
            // }
        }

        is_tracking_ = true;
        target_id_ = res;
        return res;
    }

} // namespace rm_rune_detector
