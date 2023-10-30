#ifndef _REALTRACKER_H_
#define _REALTRACKER_H_

#include "idetector.h"
#include "itracker.h"
#include "multitracker.h"
#include <mtracking/Ctracker.h>
#include <opencv2/opencv.hpp>


class realtracker
{
public:
    realtracker(std::string enginepath);
    ~realtracker();

    void init(const cv::Rect &roi, cv::Mat image);
    void runDetector(cv::Mat &frame);
    void runDetector(cv::Mat &frame, std::vector<TrackingObject> &detRet);
    void runTracker(cv::Mat &frame);
    int update(cv::Mat &frame, std::vector<TrackingObject> &detRet, cv::Point &pt);
    void reset();

private:
    itracker *m_kcf;
    idetector *m_detector;
    FrameInfo m_frameInfo;
    std::unique_ptr<BaseTracker> m_mtracker;
    regions_t m_regions;
    float m_fps;
    cv::Point m_kcfRet;
    // int m_GateSize;
    bool m_kcfLost;

};

#endif