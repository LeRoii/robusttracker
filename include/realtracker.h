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
    SSEARCH = 5,

};

struct stTrackerCfg
{
    int gateSize;                    // 32
    int visClsNum;                   // 12
    int irClsNum;                    // 7
    int initalOffsetLimit;           // 100
    int offsetLimitCeil;             // 150
    int initFusMinDistThres;         // 70
    double initialMinDistThres;      // 15.0
    double initialAreaDifThres;      // 30.0
    int searchFrameCntThres;         // 3
    int strackerFailCntThres;        // 6
    double trackUpdateSimThres;      // 0.7
    int ssearchCntThres;             // 35
    int featureCalCntThres;          // 3
    int featureAbanDistThres;        // 100
    double minDistCeil;              // 60
    int minDistThresScalCnt;         // 3
    double areaDifThresCeil;         // 50
    double maxSimThresFSwitch;       // 0.65
    double maxSimDifThresFSwitch;    // 0.3
    int strackerWaitThres;           // 5
    int containRectCntThres;         // 3
    int detectorBoxDifThres;         // 5
    int offsetLimitIncCntThres;      // 10
    int trackTraceSizeThres;         // 100
    int trackVeloUpdateSizeDifThres; // 8
    int trackVeloBufSize;            // 10
    int dtrackerLostCntThres;        // 2
    double detectorVisNmsConf;       // 0.45
    double detectorVisConf;          // 0.4
    double detectorIrNmsConf;        // 0.45
    double detectorIrConf;           // 0.4
    int trackFinalLostCntThres;      // 30
};

class trackObj
{
public:
    trackObj() = default;
    ~trackObj() = default;

    void init(const bbox_t &box, cv::Mat frame);

    cv::Point center();
    void update(cv::Mat img, const cv::Rect &box, double s);
    void updateWithoutDet();
    bool isLost();
    void predict();

    cv::Rect m_rect;
    float m_prob;
    int m_cls;
    int m_age;

    int m_lostCnt;

    std::deque<cv::Point> m_trace;
    cv::Mat m_hist;
    float m_velo[2];
    float m_acc[2];

    bool m_strackerLost;
    bool m_dtrackerLost;

    cv::Rect m_strackerRet;
    cv::Rect m_dtrackerRet;
    cv::Mat m_patch;

    cv::Rect m_initRect;
    float rx, ry;

private:
    inline void calcVelo();

    std::deque<std::pair<int, int>> m_veloBuf;

    cv::Rect m_lastPos;
};

class realtracker
{
public:
    realtracker(std::string rgbEnginepath, std::string irEnginepath, int rgbClassNum, int irClassNum);
    realtracker(std::string cfg);
    ~realtracker();

    void init(const cv::Rect &roi, cv::Mat image);
    void init(const cv::Point &pt, cv::Mat &trackImg, cv::Mat &detImage);
    // void runDetector(cv::Mat &frame);
    void runDetector(cv::Mat &frame, bbox_t *detRet, int &boxs_count);
    void runDetectorNoDraw(cv::Mat &frame, bbox_t *detRet, int &boxs_count);
    void runDetectorOut(cv::Mat &frame, bbox_t *detRet, int &boxs_count);
    void runTracker(cv::Mat &frame, bool alone = true);
    void runTrackerNoDraw(cv::Mat &frame, bool alone = true);
    // int update(cv::Mat &frame, std::vector<TrackingObject> &detRet, cv::Point &pt);
    // EN_TRACKER_FSM update(cv::Mat &frame, std::vector<TrackingObject> &detRet, uint8_t *trackerStatus);
    EN_TRACKER_FSM update(cv::Mat &frame, std::vector<TrackingObject> &detRet, uint8_t *trackerStatus);
    EN_TRACKER_FSM update(cv::Mat &frame, cv::Mat &frameTracker, uint8_t *trackerStatus, int &x_, int &y_, cv::Rect &);
    // EN_TRACKER_FSM update(cv::Mat& frame, cv::Mat& frameTracker, std::vector<bbox_t> &detRet, uint8_t *trackerStatus, bool isIRImg);
    void reset();
    void setFrameScale(double s);
    void setGateSize(int s);
    void setIrFrame(bool ir);

    bool trackerLost();

private:
    void fsmUpdate(cv::Mat &frame, cv::Mat &, cv::Rect &);
    void FSM_PROC_STRACK(cv::Mat &frame, cv::Mat &, cv::Rect &);
    void FSM_PROC_DTRACK(cv::Mat &frame, cv::Mat &, cv::Rect &);
    void FSM_PROC_SEARCH(cv::Mat &frame);
    void FSM_PROC_SSEARCH(cv::Mat &frame, cv::Rect &);

    itracker *m_stracker;
    CDetector *m_detector;
    CDetector *m_irDetector;
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
    trackObj m_trackObj;
    int m_mtrackerLostCnt;

    cv::Rect m_strackerRet;
    int m_trackerOffsetLimit;

    bool m_dtrackerLost;
    bool m_strackerLost;

    bool m_irFrame;

    double minDistThres;
    double areaDifThres;
    int m_dtrackerLostCnt;
    int osdw;
    bool sevorKeepFlag;
    int16_t x;
    int16_t y;
    int m_strackerfailedCnt;
    int m_ssearchCnt;
    int m_imgCenterX;
    int m_imgCenterY;
    bbox_t m_detRet[OBJ_NUMB_MAX_SIZE];
    int m_boxes_count;
};

#endif