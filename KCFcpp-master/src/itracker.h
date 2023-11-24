#ifndef _ITRACKER_H_
#define _ITRACKER_H_

#include <opencv2/opencv.hpp>

class itracker
{
public:
    itracker();
    ~itracker();

    void init(cv::Rect &roi, cv::Mat image);
    void init(const cv::Point &pt, cv::Mat image);
    cv::Rect update(cv::Mat image);
    void reset();
    bool isLost();
    cv::Point centerPt();
    void setGateSize(int s);
    cv::Rect updateTP(cv::Mat image);
    int m_GateSize;
    void resetTemplate(cv::Mat &img);

private:
    bool m_isLost;
    cv::Mat m_oriPatch;
    cv::Point m_centerPt;
    
    cv::Mat m_template;

    int m_templateSearchWindowSize;
    int m_templateSearchOffset;
};


#endif