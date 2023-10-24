#ifndef _ITRACKER_H_
#define _ITRACKER_H_

#include <opencv2/opencv.hpp>

class itracker
{
public:
    itracker();
    ~itracker();

    void init(const cv::Rect &roi, cv::Mat image);
    cv::Rect update(cv::Mat image, bool &islost);
    void reset();

private:
    bool m_init;
    cv::Mat m_oriPatch;
};


#endif