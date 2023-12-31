#ifndef _REALTRACKER_H_
#define _REALTRACKER_H_

#include "idetector.h"
#include "itracker.h"
#include "multitracker.h"
#include <mtracking/Ctracker.h>
#include <opencv2/opencv.hpp>


enum class EN_TRACKER_FSM
{
    LOST = 0,
    INIT = 1,
    STRACK = 2,
    DTRACK = 3,
    SEARCH = 4,

};

class realtracker
{
public:
    realtracker(std::string enginepath);
    ~realtracker();

    void init(const cv::Rect &roi, cv::Mat image);
    void init(const cv::Point &pt, cv::Mat image);
    void runDetector(cv::Mat &frame);
    void runDetector(cv::Mat &frame, std::vector<TrackingObject> &detRet);
    void runDetectorNoDraw(cv::Mat &frame, std::vector<TrackingObject> &detRet);
    void runTracker(cv::Mat &frame);
    void runTrackerNoDraw(cv::Mat &frame);
    // int update(cv::Mat &frame, std::vector<TrackingObject> &detRet, cv::Point &pt);
    EN_TRACKER_FSM update(cv::Mat &frame, std::vector<TrackingObject> &detRet, uint8_t *trackerStatus);
    void reset();
    void setFrameScale(double s);
    void setGateSize(int s);
    void setIrFrame(bool ir);

private:
    void fsmUpdate(cv::Mat &frame);
    void FSM_PROC_STRACK(cv::Mat &frame);
    void FSM_PROC_DTRACK(cv::Mat &frame);
    void FSM_PROC_SEARCH(cv::Mat &frame);

    itracker *m_stracker;
    idetector *m_detector;
    FrameInfo m_frameInfo;
    std::unique_ptr<BaseTracker> m_mtracker;
    regions_t m_regions;
    float m_fps;
    cv::Point m_kcfRet;
    // int m_GateSize;
    bool m_kcfLost;

    double m_frameScale;
    EN_TRACKER_FSM m_state;

    cv::Mat m_initTarget;
    int m_trackCls;
    int lastId;
    TrackingObject m_trackObj;
    int m_mtrackerLostCnt;

    cv::Rect m_strackerRet;
    int m_trackerOffsetLimit;

    bool m_irFrame;

};

#endif