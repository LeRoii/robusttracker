#ifndef _ITRACKER_H_
#define _ITRACKER_H_

#include <opencv2/opencv.hpp>

class itracker
{
public:
    itracker();
    ~itracker();

    void init(const cv::Rect &roi, cv::Mat image);
    cv::Rect update(cv::Mat image);
    void reset();
    bool isLost();
    cv::Point centerPt();
    void setGateSize(int s);

private:
    bool m_isLost;
    cv::Mat m_oriPatch;
    cv::Point m_centerPt;
    int m_GateSize;
};


#endif