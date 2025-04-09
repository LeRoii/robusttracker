#ifndef _REALTRACKER_H_
#define _REALTRACKER_H_


#include <opencv2/opencv.hpp>
#include <memory>
#include "common.h"

class realtracker
{
public:
    explicit realtracker(std::string cfg);
    ~realtracker();

    void init(const cv::Rect &roi, cv::Mat image);
    void init(const cv::Point &pt, cv::Mat &trackImg, cv::Mat &detImage);
    void runDetectorOut(cv::Mat &frame, bbox_t *detRet, int &boxs_count);
    void update(cv::Mat &frame, cv::Mat &frameTracker, uint8_t *trackerStatus, int &x_, int &y_, cv::Rect &rect);
    void reset();
    bool trackerLost();
    void gateAdjust(int dir);//0:up, 1:down, 2:left, 3:right
    int getGateSize();

private:
    class RealtrackerImpl;
    std::unique_ptr<RealtrackerImpl> m_impl;

};


#endif