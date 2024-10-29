#ifndef VIDEOPRO_COMMON_H
#define VIDEOPRO_COMMON_H
#include <opencv2/opencv.hpp>
#include <opencv2/freetype.hpp>
class VideoPro_Common
{
    private:
        /* data */
    public:
        VideoPro_Common(/* args */);
        unsigned long long VidePro_ComGetTime();
        double calculateContrastRatio(const cv::Mat& image);
        ~VideoPro_Common();
};

#endif	// VIDEOPRO_COMMON_H
