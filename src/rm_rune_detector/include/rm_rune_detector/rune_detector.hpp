#ifndef RUNE_DETECTOR__DETECTOR_HPP_
#define RUNE_DETECTOR__DETECTOR_HPP_

// OpenCV
#include <opencv2/core.hpp>

namespace rm_rune_detector
{
    class RuneDetector
    {
    public:
        RuneDetector(const int &bin_thres, const int &color);
        int binary_thres;
        int detect_color;

        // Debug msgs
        cv::Mat binary_img;

    private:
    };

} // namespace rm_rune_detector

#endif // RUNE_DETECTOR__DETECTOR_HPP_
