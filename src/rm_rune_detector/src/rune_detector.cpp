// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

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

namespace rm_rune_detector
{
    RuneDetector::RuneDetector(
        const int &bin_thres, const int &color)
        : binary_thres(bin_thres), detect_color(color)
    {
    }

} // namespace rm_rune_detector
