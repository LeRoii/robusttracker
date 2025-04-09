#include "idetector.h"
#include "itracker.h"
#include "realtracker.h"
#include <arpa/inet.h>
#include "spdlog/spdlog.h"
#include "spdlog/stopwatch.h"
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#define TRACKER_DEBUG 1
#define TRACKER_DEBUG_DRAW 0



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

    void calcTjr(cv::Mat img, bbox_t *detRet, int m_boxes_count);

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
    cv::Point p1;

private:
    inline void calcVelo();

    std::deque<std::pair<int, int>> m_veloBuf;

    cv::Rect m_lastPos;
    bool m_findp;
};

class realtracker::RealtrackerImpl
{
public:
    RealtrackerImpl(std::string cfg);
    ~RealtrackerImpl();

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
   // EN_TRACKER_FSM update(cv::Mat &frame, std::vector<TrackingObject> &detRet, uint8_t *trackerStatus);
    void update(cv::Mat &frame, cv::Mat &frameTracker, uint8_t *trackerStatus, int &x_, int &y_, cv::Rect &);
    // EN_TRACKER_FSM update(cv::Mat& frame, cv::Mat& frameTracker, std::vector<bbox_t> &detRet, uint8_t *trackerStatus, bool isIRImg);
    void reset();
    void setFrameScale(double s);
    void setGateSize(int s);
    int getGateSize();
    void setIrFrame(bool ir);

    bool trackerLost();

    void gateAdjust(int dir);//0:up, 1:down, 2:left, 3:right
    
    // FrameInfo m_frameInfo;
    // void plot_tracks(cv::Mat &frame, std::vector<std::shared_ptr<Track>> &tracks);
    // std::vector<std::shared_ptr<Track>> tracks;
    int osdw;

private:
    void fsmUpdate(cv::Mat &frame, cv::Mat &, cv::Rect &);
    void FSM_PROC_STRACK(cv::Mat &frame, cv::Mat &, cv::Rect &);
    void FSM_PROC_DTRACK(cv::Mat &frame, cv::Mat &, cv::Rect &);
    void FSM_PROC_SEARCH(cv::Mat &frame);
    void FSM_PROC_SSEARCH(cv::Mat &frame, cv::Rect &);
    bool sseFind(float sim);

    itracker *m_stracker;
    CDetector *m_detector;
    CDetector *m_irDetector;


    float m_fps;
    cv::Point m_kcfRet;
    int m_GateSize;
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

    bool sevorKeepFlag;
    int16_t x;
    int16_t y;
    int m_strackerfailedCnt;
    int m_ssearchCnt;
    int m_imgCenterX;
    int m_imgCenterY;
    bbox_t m_detRet[OBJ_NUMB_MAX_SIZE];
    int m_boxes_count;

    bool m_adjustFlg;
    cv::Point m_adjstPt;

    bool m_initdet;
    int m_strackerslpcnt;
    float m_serDis;
    int m_gateS;
    bool m_smlv;

    cv::Rect m_finalRect;
};


static spdlog::stopwatch sw;
static stTrackerCfg trackerCfg;

#define CAL_VELO_MODE 0

static double calculateHistogramSimilarity(const cv::Mat &image1, const cv::Mat &image2)
{
    // cv::Mat hsvImage1, hsvImage2;
    // cv::cvtColor(image1, hsvImage1, cv::COLOR_BGR2HSV);
    // cv::cvtColor(image2, hsvImage2, cv::COLOR_BGR2HSV);

    // int hBins = 30;
    // int sBins = 32;
    // int histSize[] = {hBins, sBins};
    // float hRanges[] = {0, 180};
    // float sRanges[] = {0, 256};
    // const float* ranges[] = {hRanges, sRanges};
    // int channels[] = {0, 1};

    // cv::MatND hist1, hist2;
    // cv::calcHist(&hsvImage1, 1, channels, cv::Mat(), hist1, 2, histSize, ranges, true, false);
    // cv::calcHist(&hsvImage2, 1, channels, cv::Mat(), hist2, 2, histSize, ranges, true, false);

    // double similarity = cv::compareHist(hist1, hist2, cv::HISTCMP_BHATTACHARYYA);

    // return similarity;

    int bins = 64;
    std::vector<int> histSize;
    std::vector<float> ranges;
    std::vector<int> channels;

    for (int j = 0, stop = image1.channels(); j < stop; ++j)
    {
        histSize.push_back(bins);
        ranges.push_back(0);
        ranges.push_back(255);
        channels.push_back(j);
    }
    // cv::Rect roi = cv::Rect{box.x,box.y,box.w,box.h};
    // Clamp(roi.x, roi.width, frame.cols);
    // Clamp(roi.y, roi.height, frame.rows);
    std::vector<cv::Mat> img1 = {image1};
    std::vector<cv::Mat> img2 = {image2};
    cv::Mat hist1, hist2;
    cv::calcHist(img1, channels, cv::Mat(), hist1, histSize, ranges, false);
    cv::calcHist(img2, channels, cv::Mat(), hist2, histSize, ranges, false);
    cv::normalize(hist1, hist1, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
    cv::normalize(hist2, hist2, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

    return cv::compareHist(hist1, hist2, cv::HISTCMP_BHATTACHARYYA);
}

cv::Mat CalcHist(cv::Mat img, cv::Rect box)
{
    int bins = 64;
    std::vector<int> histSize;
    std::vector<float> ranges;
    std::vector<int> channels;

    for (int j = 0, stop = img.channels(); j < stop; ++j)
    {
        histSize.push_back(bins);
        ranges.push_back(0);
        ranges.push_back(255);
        channels.push_back(j);
    }
    cv::Rect roi = cv::Rect{box.x, box.y, box.width, box.height};
    // Clamp(roi.x, roi.width, frame.cols);
    // Clamp(roi.y, roi.height, frame.rows);
    std::vector<cv::Mat> regROI = {img(roi)};
    cv::Mat hist;
    cv::calcHist(regROI, channels, cv::Mat(), hist, histSize, ranges, false);
    cv::normalize(hist, hist, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

    return hist;
}

double calculateSSIM(const cv::Mat &imgg1, const cv::Mat &imgg2)
{
    // printf("realtracker::calculateSSIM start\n");
    // std::cout<<"img1 :"<<img1.size()<<std::endl;
    // std::cout<<"img2 :"<<img2.size()<<std::endl;
    cv::Mat img1, img2;

    cv::Size targetSize(std::min(imgg1.cols, imgg2.cols), std::min(imgg1.rows, imgg2.rows));
    cv::resize(imgg1, img1, targetSize);
    cv::resize(imgg2, img2, targetSize);

    // printf("realtracker::calculateSSIM\n");

    // 分离通道
    std::vector<cv::Mat> channels1, channels2;
    cv::split(img1, channels1);
    cv::split(img2, channels2);

    double ssim = 0.0;

    // 计算每个通道的SSIM
    for (int i = 0; i < 3; ++i)
    {
        // 转换为double类型
        channels1[i].convertTo(channels1[i], CV_64F);
        channels2[i].convertTo(channels2[i], CV_64F);

        // 计算均值
        double mean1 = cv::mean(channels1[i])[0];
        double mean2 = cv::mean(channels2[i])[0];

        // 计算方差
        cv::Mat var1, var2;
        cv::multiply(channels1[i] - mean1, channels1[i] - mean1, var1);
        cv::multiply(channels2[i] - mean2, channels2[i] - mean2, var2);
        double var1_scalar = cv::mean(var1)[0];
        double var2_scalar = cv::mean(var2)[0];

        // 计算协方差
        cv::Mat covar;
        cv::multiply(channels1[i] - mean1, channels2[i] - mean2, covar);
        double covar_scalar = cv::mean(covar)[0];

        // 计算SSIM
        double c1 = 0.01 * 255 * 0.01 * 255;
        double c2 = 0.03 * 255 * 0.03 * 255;
        double channel_ssim = (2 * mean1 * mean2 + c1) * (2 * covar_scalar + c2) / ((mean1 * mean1 + mean2 * mean2 + c1) * (var1_scalar + var2_scalar + c2));

        // 累加每个通道的SSIM
        ssim += channel_ssim;
    }

    // 求取均值
    ssim /= 3.0;

    return ssim;
}

int Clamp(int &v, int &size, int hi)
{
    int res = 0;
    if (v < 0)
    {
        res = v;
        v = 0;
        return res;
    }
    else if (v + size > hi - 1)
    {
        res = v;
        v = hi - 1 - size;
        if (v < 0)
        {
            size += v;
            v = 0;
        }
        res -= v;
        return res;
    }
    return res;
};

inline static int newCeil(float i)
{
    return i >= 0 ? (int)ceil(i) : -(int)ceil(-i);
}

static inline void offsetLimitInc(int &lmt)
{
    static int cnt = 0;
    cnt++;
    if (cnt == trackerCfg.offsetLimitIncCntThres)
    {
        lmt++;
        cnt = 0;
    }
}

void trackObj::init(const bbox_t &box, cv::Mat frame)
{
    m_rect = cv::Rect{box.x, box.y, box.w, box.h};
    m_prob = box.prop;
    m_cls = box.obj_id;
    m_age = 1;
    m_lostCnt = 0;
    m_trace.clear();
    m_trace.emplace_back(cv::Point(box.x + box.w / 2, box.y + box.h / 2));
    m_veloBuf.clear();

    int bins = 64;
    std::vector<int> histSize;
    std::vector<float> ranges;
    std::vector<int> channels;

    for (int j = 0, stop = frame.channels(); j < stop; ++j)
    {
        histSize.push_back(bins);
        ranges.push_back(0);
        ranges.push_back(255);
        channels.push_back(j);
    }
    // cv::Rect roi = cv::Rect{box.x, box.y, box.w, box.h};
    // // Clamp(roi.x, roi.width, frame.cols);
    // // Clamp(roi.y, roi.height, frame.rows);
    // std::vector<cv::Mat> regROI = {frame(roi)};
    // cv::calcHist(regROI, channels, cv::Mat(), m_hist, histSize, ranges, false);
    // cv::normalize(m_hist, m_hist, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

    m_lastPos = m_rect;

    // m_veloBUf.resize(10);
    m_strackerLost = false;
    m_dtrackerLost = false;

    // m_patch = frame(m_rect);

    m_initRect = m_rect;
    m_velo[0] = m_velo[1] = 0;

    rx = m_rect.x;
    ry = m_rect.y;

    std::cout << " trackObj::init:::" << m_rect << std::endl;
}

cv::Point trackObj::center()
{
    return cv::Point(m_rect.x + m_rect.width / 2, m_rect.y + m_rect.height / 2);
}

inline void trackObj::calcVelo()
{
    // if (m_veloBuf.size() < trackerCfg.trackVeloBufSize - 2)
    // {
    //     m_velo[0] = 0;
    //     m_velo[1] = 0;

    //     return;
    // }
    float sumx, sumy;
    sumx = sumy = 0.0;
    for (auto &velo : m_veloBuf)
    {
        sumx += velo.first;
        sumy += velo.second;
    }

    m_velo[0] = sumx / m_veloBuf.size();
    m_velo[1] = sumy / m_veloBuf.size();
#if TRACKER_DEBUG
    printf("m_veloBuf.size():%d\n", m_veloBuf.size());
#endif
}

void trackObj::update(cv::Mat img, const cv::Rect &box, double ssim)
{
#if TRACKER_DEBUG
    printf("trackObj::update:\n");
#endif
    m_rect = box;
    rx = m_rect.x;
    ry = m_rect.y;

    int sizeDif = abs(m_rect.width - m_lastPos.width) + abs(m_rect.height - m_lastPos.height);
#if TRACKER_DEBUG
// #if 1
    std::cout << "curPos:" << m_rect << std::endl;
    std::cout << "m_lastPos:" << m_lastPos << std::endl;
    // printf("w diff:%d, h diff:%d\n", abs(m_rect.width - m_lastPos.width), abs(m_rect.height - m_lastPos.height));
    printf("obj dist:%f\n",sqrtf(powf((m_rect.x - m_lastPos.x), 2) + powf((m_rect.y - m_lastPos.y), 2)));

#endif
    if (m_trace.size() > trackerCfg.trackTraceSizeThres)
        m_trace.pop_front();
    m_trace.emplace_back(cv::Point(box.x + box.width / 2, box.y + box.height / 2));
    m_age++;

    static int imgX = img.cols / 2;
    static int imgy = img.rows / 2;

    // if (m_lostCnt == 0 && sizeDif < trackerCfg.trackVeloUpdateSizeDifThres && ssim > trackerCfg.trackUpdateSimThres)
    if (m_lostCnt == 0 && ssim > trackerCfg.trackUpdateSimThres)
    {
        if (m_veloBuf.size() > trackerCfg.trackVeloBufSize)
            m_veloBuf.pop_front();

#if CAL_VELO_MODE == 0
        m_veloBuf.emplace_back(std::pair<int, int>{box.x - m_lastPos.x, box.y - m_lastPos.y});
#elif CAL_VELO_MODE == 1
        int16_t x = center().x - imgX;
        int16_t y = center().y - imgy;
        m_veloBuf.emplace_back(std::pair<int, int>{x, y});
#elif CAL_VELO_MODE == 2
        m_veloBuf.emplace_back(std::pair<int, int>{0, 0});
#endif

        calcVelo();
    }

    m_lostCnt = 0;
    m_lastPos = m_rect;

    Clamp(m_rect.x, m_rect.width, img.cols);
    Clamp(m_rect.y, m_rect.height, img.rows);
#if TRACKER_DEBUG
    if (!m_veloBuf.empty())
        printf("inst velo x:%d, inst velo y:%d\n", m_veloBuf.back().first, m_veloBuf.back().second);
    // std::cout<<"patch size:"<<m_patch.size()<<std::endl;
    // cv::imwrite("patch.png", m_patch);
#if TRACKER_DEBUG_DRAW
    cv::Point sp = cv::Point(m_rect.x + m_rect.width / 2, m_rect.y + m_rect.height / 2);
    cv::Point ep = cv::Point(sp.x + m_velo[0] * 30, sp.y + m_velo[1] * 30);
    cv::line(img, sp, ep, cv::Scalar(0, 255, 0), 2);
#endif

#endif

    // m_hist = CalcHist(img, box);
}

void trackObj::updateWithoutDet()
{
    // static float rx,ry;
    // rx = m_rect.x + m_velo[0] + m_acc[0];
    // ry = m_rect.y + m_velo[1] + m_acc[1];
    // m_rect.x += (int)newCeil(m_velo[0]);
    // m_rect.y += (int)newCeil(m_velo[1]);

    printf("vx:%f, vy:%f\n", m_velo[0], m_velo[1]);
    // printf("m_rect.x + m_velo[0]:%f, m_rect.x + m_velo[0]:%f\n", float(m_rect.x + m_velo[0]), float(m_rect.y + m_velo[1]));

    rx  += m_velo[0];
    ry  += m_velo[1];
    m_rect.x  = rx;
    m_rect.y  = ry;

    // m_rect.x = (int)round(rx);
    // m_rect.y = (int)round(ry);

    m_lostCnt++;
}

void trackObj::predict()
{
    m_rect.x += (int)newCeil(m_velo[0]);
    m_rect.y += (int)newCeil(m_velo[1]);
}

bool trackObj::isLost()
{
    if (m_trace.size() < 5)
    {
        return m_lostCnt > 1;
    }
    else if (m_trace.size() < 10)
    {
        return m_lostCnt > 3;
    }
    else if (m_trace.size() < 25)
    {
        return m_lostCnt > 5;
    }
    else
    {
        return m_lostCnt > trackerCfg.trackFinalLostCntThres;
    }
}

static cv::Mat safeCrop(cv::Mat img, cv::Rect roi)
{
    Clamp(roi.x, roi.width, img.cols);
    Clamp(roi.y, roi.height, img.rows);
    return img(roi);
}

realtracker::RealtrackerImpl::RealtrackerImpl(std::string cfg)
{
    YAML::Node config = YAML::LoadFile(cfg);
    std::string engine = config["engine"].as<std::string>();
    std::string irEngine = config["irengine"].as<std::string>();

    trackerCfg.gateSize = config["gateSize"].as<int>();
    trackerCfg.visClsNum = config["visClsNum"].as<int>();
    trackerCfg.irClsNum = config["irClsNum"].as<int>();
    trackerCfg.initalOffsetLimit = config["initalOffsetLimit"].as<int>();
    trackerCfg.offsetLimitCeil = config["offsetLimitCeil"].as<int>();
    trackerCfg.initFusMinDistThres = config["initFusMinDistThres"].as<int>();
    trackerCfg.initialMinDistThres = config["initialMinDistThres"].as<double>();
    trackerCfg.initialAreaDifThres = config["initialAreaDifThres"].as<double>();
    trackerCfg.searchFrameCntThres = config["searchFrameCntThres"].as<int>();
    trackerCfg.strackerFailCntThres = config["strackerFailCntThres"].as<int>();
    trackerCfg.trackUpdateSimThres = config["trackUpdateSimThres"].as<double>();
    trackerCfg.ssearchCntThres = config["ssearchCntThres"].as<int>();
    trackerCfg.featureCalCntThres = config["featureCalCntThres"].as<int>();
    trackerCfg.featureAbanDistThres = config["featureAbanDistThres"].as<int>();
    trackerCfg.minDistCeil = config["minDistCeil"].as<double>();
    trackerCfg.minDistThresScalCnt = config["minDistThresScalCnt"].as<int>();
    trackerCfg.areaDifThresCeil = config["areaDifThresCeil"].as<double>();
    trackerCfg.maxSimThresFSwitch = config["maxSimThresFSwitch"].as<double>();
    trackerCfg.maxSimDifThresFSwitch = config["maxSimDifThresFSwitch"].as<double>();
    trackerCfg.strackerWaitThres = config["strackerWaitThres"].as<int>();
    trackerCfg.containRectCntThres = config["containRectCntThres"].as<int>();
    trackerCfg.detectorBoxDifThres = config["detectorBoxDifThres"].as<int>();
    trackerCfg.offsetLimitIncCntThres = config["offsetLimitIncCntThres"].as<int>();
    trackerCfg.trackTraceSizeThres = config["trackTraceSizeThres"].as<int>();
    trackerCfg.trackVeloUpdateSizeDifThres = config["trackVeloUpdateSizeDifThres"].as<int>();
    trackerCfg.trackVeloBufSize = config["trackVeloBufSize"].as<int>();
    trackerCfg.dtrackerLostCntThres = config["dtrackerLostCntThres"].as<int>();
    trackerCfg.detectorVisNmsConf = config["detectorVisNmsConf"].as<double>();
    trackerCfg.detectorVisConf = config["detectorVisConf"].as<double>();
    trackerCfg.detectorIrNmsConf = config["detectorIrNmsConf"].as<double>();
    trackerCfg.detectorIrConf = config["detectorIrConf"].as<double>();
    trackerCfg.trackFinalLostCntThres = config["trackFinalLostCntThres"].as<int>();

    m_stracker = new itracker();
    m_stracker->setGateSize(trackerCfg.gateSize);

    m_detector = new CDetector(const_cast<char *>(engine.c_str()), 3, trackerCfg.visClsNum, trackerCfg.detectorVisNmsConf, trackerCfg.detectorVisConf);
    m_detector->Init();

    m_irDetector = new CDetector(const_cast<char *>(irEngine.c_str()), 3, trackerCfg.irClsNum, trackerCfg.detectorIrNmsConf, trackerCfg.detectorIrConf);
    m_irDetector->Init();

    m_fps = 25;

    m_state = EN_TRACKER_FSM::INIT;
    m_frameScale = 1.f;

    m_trackerOffsetLimit = trackerCfg.initalOffsetLimit;

    m_dtrackerLost = false;
    m_strackerLost = false;

    m_irFrame = false;

    m_dtrackerLostCnt = 0;
    osdw = trackerCfg.gateSize;

    m_imgCenterX = (m_frameScale == 1 ? 960 : 640);
    m_imgCenterY = (m_frameScale == 1 ? 540 : 360);
}

inline double getDistance(cv::Point point1, cv::Point point2)
{
    return sqrtf(powf((point1.x - point2.x), 2) + powf((point1.y - point2.y), 2));
}

void realtracker::RealtrackerImpl::init(const cv::Rect &roi, cv::Mat image)
{
    // m_stracker->init(roi, image);
}

void realtracker::RealtrackerImpl::init(const cv::Point &pt, cv::Mat &trackImage, cv::Mat &detImage)
{
    m_adjustFlg = false;
    m_trackerOffsetLimit = trackerCfg.initalOffsetLimit;
    // double x = (double)pt.x * m_frameScale;
    // double y = (double)pt.y * m_frameScale;
    // cv::Point iniPt = cv::Point{(int)x, (int)y};
    cv::Point iniPt = pt;

    m_stracker->init(pt, trackImage);
    m_strackerfailedCnt = 0;
    m_ssearchCnt = 0;

    // if (m_irFrame)
    // {
    //     m_trackerOffsetLimit = 20;
    //     bbox_t initBox;
    //     initBox.x = m_stracker->roix;
    //     initBox.y = m_stracker->roiy;
    //     initBox.w = initBox.h = m_stracker->m_GateSize;
    //     m_trackObj.init(initBox, trackImage);
    //     m_state = EN_TRACKER_FSM::STRACK;
    //     return;
    // }

    spdlog::warn("realtracker::init x:%d,y:%d, ptx:%d, pty:%d\n", iniPt.x, iniPt.y, pt.x, pt.y);
    memset(m_detRet, 0x00, sizeof(*m_detRet));

// #if TRACKER_DEBUG
//     runDetector(detImage, m_detRet, m_boxes_count);
// #else
//     runDetectorNoDraw(detImage, m_detRet, m_boxes_count);
// #endif
    float minDist = 1000.f;
    int minIdx = -1;
    // for (int i = 0; i < m_boxes_count; ++i)
    // {
    //     cv::Point center{m_detRet[i].x + m_detRet[i].w / 2, m_detRet[i].y + m_detRet[i].h / 2};
    //     double dist = getDistance(iniPt, center);
    //     // printf("obj pos:(%d, %d), dist:%f\n", brect.tl().x, brect.tl().y, dist);
    //     if (dist < minDist)
    //     {
    //         minDist = dist;
    //         minIdx = i;
    //     }
    // }
    minIdx = -1;
    printf("realtracker::init minDist = %f\n", minDist);

    m_state = EN_TRACKER_FSM::STRACK;
    if (minIdx != -1)
    {
//         cv::Rect closestRect{m_detRet[minIdx].x, m_detRet[minIdx].y, m_detRet[minIdx].w, m_detRet[minIdx].h};
//         if (closestRect.contains(iniPt) || minDist < trackerCfg.initFusMinDistThres)
//         {
//             m_state = EN_TRACKER_FSM::DTRACK;
//             m_initTarget = trackImage(closestRect);
// #if TRACKER_DEBUG
//             printf("\n\ninit pt with det\n");
//             cv::imwrite("initdet.png", m_initTarget);
// #endif
//             m_trackCls = m_detRet[minIdx].obj_id;
//             // lastId = m_frameInfo.m_tracks[0][minIdx].m_ID.m_val;
//             m_trackObj.init(m_detRet[minIdx], trackImage);

//             cv::Point center{m_detRet[minIdx].x + m_detRet[minIdx].w / 2, m_detRet[minIdx].y + m_detRet[minIdx].h / 2};
//             m_stracker->init(center, trackImage);
//         }
    }
    else
    {
        bbox_t initBox;
        initBox.x = iniPt.x - m_stracker->m_GateSize / 2;
        initBox.y = iniPt.y - m_stracker->m_GateSize / 2;
        initBox.w = initBox.h = m_stracker->m_GateSize;
        m_trackObj.init(initBox, trackImage);
    }

    minDistThres = trackerCfg.initialMinDistThres;
    areaDifThres = trackerCfg.initialAreaDifThres;
}

void realtracker::RealtrackerImpl::FSM_PROC_SEARCH(cv::Mat &frameDetect)
{
    printf("\nSM_PROC_SEARCH\n");
    static int searchFrameCnt = 0;
    searchFrameCnt++;
    if (searchFrameCnt > trackerCfg.searchFrameCntThres)
    {
        m_state = EN_TRACKER_FSM::LOST;
        searchFrameCnt = 0;
        m_mtrackerLostCnt = 0;
        m_stracker->reset();
    }
}

int intersectionArea(cv::Rect r1, cv::Rect r2)
{
    cv::Rect intersection = r1 & r2;
    return intersection.area();
}

void realtracker::RealtrackerImpl::FSM_PROC_STRACK(cv::Mat &frameDetect, cv::Mat &frameTracker, cv::Rect &trackRect)
{
   // printf("\nFSM_PROC_STRACK\n");
    cv::Mat &frame = frameDetect;

    runTracker(frame);

    if (m_stracker->isLost())
    {
        // rectangle(frame, m_trackObj.m_rect, cv::Scalar(0, 255, 255), 3, 8);
        trackRect = m_trackObj.m_rect;
        m_strackerfailedCnt++;
        printf("STRACK lost m_strackerfailedCnt:%d\n", m_strackerfailedCnt);
        if (m_strackerfailedCnt < trackerCfg.strackerFailCntThres)
            m_state = EN_TRACKER_FSM::SSEARCH;
        else
            m_state = EN_TRACKER_FSM::SEARCH;
    }
    else
    {
        // m_strackerfailedCnt = 0;
        // rectangle(frame, m_strackerRet, cv::Scalar(255, 255, 255), 3, 8);
        trackRect = m_strackerRet;
        m_state = EN_TRACKER_FSM::STRACK;
        m_trackObj.update(frame, m_strackerRet, m_stracker->getConf());
    }

    printf("m_trackObj age:%d, lostcnt:%d, trace size:%d, velo x:%f, velo y:%f\n",
           m_trackObj.m_age, m_trackObj.m_lostCnt, m_trackObj.m_trace.size(), m_trackObj.m_velo[0], m_trackObj.m_velo[1]);

    // printf("m_trackObj age:%d, lostcnt:%d, trace size:%d, velo x:%f, velo y:%f, acc x:%f, acc y:%f\n",
    //        m_trackObj.m_age, m_trackObj.m_lostCnt, m_trackObj.m_trace.size(), m_trackObj.m_velo[0], m_trackObj.m_velo[1],
    //        m_trackObj.m_acc[0], m_trackObj.m_acc[1]);

    return;
}

void realtracker::RealtrackerImpl::FSM_PROC_SSEARCH(cv::Mat &frame, cv::Rect &trackRect)
{
    printf("\nFSM_PROC_SSEARCH, m_ssearchCnt:%d\n", m_ssearchCnt);
//     if (m_ssearchCnt++ < trackerCfg.ssearchCntThres)
//     {
// #if TRACKER_DEBUG
//         spdlog::debug("velo x:{}, velo y:{}", m_trackObj.m_velo[0], m_trackObj.m_velo[1]);
//         cv::Point sp = cv::Point(m_trackObj.m_rect.x + m_trackObj.m_rect.width / 2, m_trackObj.m_rect.y + m_trackObj.m_rect.height / 2);
//         cv::Point ep = cv::Point(sp.x + m_trackObj.m_velo[0] * 80, sp.y + m_trackObj.m_velo[1] * 80);
//         cv::line(frame, sp, ep, cv::Scalar(0, 255, 0), 2);
//         std::cout << m_trackObj.m_rect << std::endl;
// #endif
//         m_trackObj.updateWithoutDet();
//         usleep(8000);
//         m_stracker->setRoi(m_trackObj.m_rect);
//         rectangle(frame, m_trackObj.m_rect, cv::Scalar(0, 255, 255), 3, 8);
//         std::cout << m_trackObj.m_rect << std::endl;
//         m_state = EN_TRACKER_FSM::SSEARCH;
//         trackRect = m_trackObj.m_rect;
//     }
//     else
//     {
//         m_stracker->isLost() = false;
//         runTracker(frame);
//         // rectangle(frame, m_strackerRet, cv::Scalar(255, 255, 255), 3, 8);
//         m_ssearchCnt = 0;
//         m_state = EN_TRACKER_FSM::STRACK;
//         trackRect = m_strackerRet;
//     }

    if (m_ssearchCnt++ < trackerCfg.ssearchCntThres)
    {
        m_trackObj.updateWithoutDet();
        std::cout << m_trackObj.m_rect << std::endl;
        usleep(8000);
        m_stracker->setRoi(m_trackObj.m_rect);
        trackRect = m_trackObj.m_rect;
    }
    else
    {
        spdlog::warn("ssearchCntThres met exit");
        m_state = EN_TRACKER_FSM::SEARCH;
        trackRect = m_trackObj.m_rect;
        return;
    }

    if(m_ssearchCnt % 12 == 0)
    {
        double sim = 0.f;
        auto rect = m_stracker->find(frame, sim);

        // if(sim > 0.6)
        if(sseFind(sim))
        {
            //find, to strack
            m_state = EN_TRACKER_FSM::STRACK;
            trackRect = rect;
            m_stracker->isLost() = false;
            m_stracker->reset();
            m_stracker->init(rect, frame);
            m_ssearchCnt = 1;
            m_strackerfailedCnt--;
            return;
        }

#if 1
        cv::rectangle(frame, rect, cv::Scalar(123,30,56), 2);
#endif

        trackRect = m_trackObj.m_rect;
        m_state = EN_TRACKER_FSM::SSEARCH;
        m_ssearchCnt++;

    }
}

void realtracker::RealtrackerImpl::FSM_PROC_DTRACK(cv::Mat &frameDetect, cv::Mat &frameTracker, cv::Rect &trackRect)
{
    printf("\nFSM_PROC_DTRACK\n");
    int boxes_count = 0;
#if TRACKER_DEBUG
    auto detimg = frameDetect.clone();
    auto debugimg = frameDetect.clone();
    runDetector(detimg, m_detRet, m_boxes_count);
    runDetector(detimg, m_detRet, m_boxes_count);
    runTracker(frameDetect, false);
#else

    runDetectorNoDraw(frameDetect, m_detRet, m_boxes_count);
    runTrackerNoDraw(frameTracker, false);
#endif

    m_trackObj.predict();

#if TRACKER_DEBUG
    std::cout << "FSM_PROC_DTRACK start:::" << m_trackObj.m_rect << std::endl;
#endif

    cv::Rect finalRect;
    cv::Rect m_dtrackerRet;
    cv::Scalar color = cv::Scalar(255, 255, 255);

    m_strackerLost = m_stracker->isLost();

    printf("detRet size :%d\n", m_boxes_count);
    if (m_boxes_count == 0)
    {
        if (m_strackerLost)
        {
            m_trackObj.updateWithoutDet();
#if TRACKER_DEBUG
            color = cv::Scalar(0, 0, 0);
#endif
        }
        else
        {
            finalRect.x = m_strackerRet.x + m_strackerRet.width / 2 - m_trackObj.m_rect.width / 2;
            finalRect.y = m_strackerRet.y + m_strackerRet.height / 2 - m_trackObj.m_rect.height / 2;
            finalRect.width = m_trackObj.m_rect.width;
            finalRect.height = m_trackObj.m_rect.height;
#if TRACKER_DEBUG
            color = cv::Scalar(0, 0, 255);
#endif

            m_trackObj.update(frameTracker, finalRect, 0);
        }

        if (m_trackObj.isLost())
        {
            spdlog::debug("tracker lost");
            m_state = EN_TRACKER_FSM::SEARCH;
            return;
        }

        // cv::rectangle(frameTracker, m_trackObj.m_rect, color, 2);
        printf("m_trackObj age:%d, lostcnt:%d, trace size:%d, velo x:%f, velo y:%f\n",
               m_trackObj.m_age, m_trackObj.m_lostCnt, m_trackObj.m_trace.size(), m_trackObj.m_velo[0], m_trackObj.m_velo[1]);
        std::cout << m_trackObj.m_rect << std::endl;

        m_state = EN_TRACKER_FSM::DTRACK;
        trackRect = m_trackObj.m_rect;

        return;
    }

    int minIdx = -1;
    double minDist = 10000.f;
    bool findLast = false;

    // auto trackObj = m_trackObj;

    auto cmpDist = [this](bbox_t box1, bbox_t box2)
    {
        cv::Point center1{box1.x + box1.w / 2, box1.y + box1.h / 2};
        double dist1 = ((box1.obj_id == m_trackObj.m_cls) ? getDistance(m_trackObj.center(), center1) : 1000);
        cv::Point center2{box2.x + box2.w / 2, box2.y + box2.h / 2};
        double dist2 = ((box2.obj_id == m_trackObj.m_cls) ? getDistance(m_trackObj.center(), center2) : 1000);
        // cv::Point center1{box1.x + box1.w / 2, box1.y + box1.h / 2};
        // double dist1 = getDistance(m_trackObj.center(), center1);
        // cv::Point center2{box2.x + box2.w / 2, box2.y + box2.h / 2};
        // double dist2 = getDistance(m_trackObj.center(), center2);
        // if(box1.obj_id != 1)
        //     dist1 = 1000;
        // if(box2.obj_id != 1)
        //     dist2 = 1000;

        return dist1 < dist2;
    };

    // std::sort(detRet.begin(), detRet.end(), cmpDist);
    std::sort(m_detRet, m_detRet + m_boxes_count, cmpDist);

    cv::Point center{m_detRet[0].x + m_detRet[0].w / 2, m_detRet[0].y + m_detRet[0].h / 2};
    minDist = getDistance(m_trackObj.center(), center);
    cv::Rect closestSSIMRect;
    // int areaDif = m_trackObj.m_initRect.width - m_detRet[0].w + m_trackObj.m_initRect.height - m_detRet[0].h;
    int areaDif = m_trackObj.m_rect.width - m_detRet[0].w + m_trackObj.m_rect.height - m_detRet[0].h;
    areaDif = abs(areaDif);
    double maxSSIM = 0;
    double maxSSIMDif = 0;

    // calculate hist start
    {
        int histCalCnt = m_boxes_count > trackerCfg.featureCalCntThres ? trackerCfg.featureCalCntThres : m_boxes_count;
        std::vector<cv::Mat> detHists;
        detHists.resize(histCalCnt);
        cv::Scalar colormap[3] = {{0, 255, 0}, {255, 0, 0}, {0, 0, 255}};
        int closestSSIMIdx = -1;

        for (int i = 0; i < histCalCnt; ++i)
        {
            // int bins = 64;
            // std::vector<int> histSize;
            // std::vector<float> ranges;
            // std::vector<int> channels;

            // for (int j = 0, stop = frame.channels(); j < stop; ++j)
            // {
            //     histSize.push_back(bins);
            //     ranges.push_back(0);
            //     ranges.push_back(255);
            //     channels.push_back(j);
            // }

            cv::Rect roi = cv::Rect{m_detRet[i].x, m_detRet[i].y, m_detRet[i].w, m_detRet[i].h};
            Clamp(roi.x, roi.width, frameTracker.cols);
            Clamp(roi.y, roi.height, frameTracker.rows);
            // std::vector<cv::Mat> regROI = { frame(roi) };
            // cv::calcHist(regROI, channels, cv::Mat(), detHists[i], histSize, ranges, false);
            // cv::normalize(detHists[i], detHists[i], 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

            // double res = cv::compareHist(detHists[i], m_trackObj.m_hist, cv::HISTCMP_BHATTACHARYYA);
            double res = calculateSSIM(frameTracker(roi), m_trackObj.m_patch);

            cv::Point center{m_detRet[i].x + m_detRet[i].w / 2, m_detRet[i].y + m_detRet[i].h / 2};
#if TRACKER_DEBUG_DRAW
            cv::line(debugimg, center, m_trackObj.center(), colormap[i], 2);
            cv::putText(debugimg, std::to_string(res), center, cv::FONT_HERSHEY_COMPLEX, 0.8, colormap[i], 1, 8, 0);
#endif
            if (getDistance(m_trackObj.center(), center) > trackerCfg.featureAbanDistThres)
            {
                spdlog::debug("ssim obj too far, continue");
                continue;
            }

            if (res > maxSSIM)
            {
                maxSSIMDif = res - maxSSIM;
                maxSSIM = res;
                closestSSIMIdx = i;
                closestSSIMRect = roi;
            }
        }
        // calculate hist end
    }

    printf("realtracker::FSM_PROC_DTRACK dist:%f, areaDif:%d\n", minDist, areaDif);
    // if((minIdx != -1 && minDist < 70) || findLast)
    // double minDistThres = 15.0;//m_trackObj.m_lostCnt > 10 ? 50.0 : 15.0;
    // double areaDifThres = 25.0f;

    // if(m_dtrackerLost && m_strackerLost)
    // {
    //     minDistThres = std::max(70.0, minDistThres*2);
    //     areaDifThres = 60;
    // }
    // else if(m_dtrackerLost)
    // {
    //     minDistThres = std::max(50.0, minDistThres);
    //     areaDifThres = 50;
    // }

    if (m_dtrackerLost)
    {
        m_dtrackerLostCnt++;
        if (m_dtrackerLostCnt > trackerCfg.dtrackerLostCntThres)
        {
            minDistThres = std::min(trackerCfg.minDistCeil, minDistThres * trackerCfg.minDistThresScalCnt);
            areaDifThres = trackerCfg.areaDifThresCeil;
        }
    }
    else
    {
        m_dtrackerLostCnt = 0;
    }

    spdlog::debug("minDistThres:{}, areaDifThres:{}", minDistThres, areaDifThres);
    cv::Rect derRect = cv::Rect{m_detRet[0].x, m_detRet[0].y, m_detRet[0].w, m_detRet[0].h};
    Clamp(derRect.x, derRect.width, frameTracker.cols);
    Clamp(derRect.y, derRect.height, frameTracker.rows);

    // std::cout<<"closest detRet:"<<derRect<<std::endl;

#if TRACKER_DEBUG_DRAW
    cv::rectangle(debugimg, m_trackObj.m_rect, cv::Scalar(0, 0, 0), 2);
    cv::rectangle(debugimg, derRect, cv::Scalar(255, 0, 127), 2);
    // cv::imwrite("detrect.png", frame(derRect));
    cv::imwrite("patch.png", m_trackObj.m_patch);
    // cv::imwrite("m_trackObj.m_rect.png", frame(m_trackObj.m_rect));

    // double sim = calculateHistogramSimilarity(frame(m_trackObj.m_rect), frame(derRect));
    // printf("FSM_PROC_DTRACKdddddddddddddddddddddd sim:%f\n", sim);

    // double sim = calculateHistogramSimilarity(m_trackObj.m_patch, frame(derRect));
    // printf("FSM_PROC_DTRACK 11111111111sim:%f\n", sim);

    // sim = calculateHistogramSimilarity(m_trackObj.m_patch, frame(m_trackObj.m_rect));
    // printf("FSM_PROC_DTRACK 22222222222222222222ssssssssssss:%f\n", sim);

    // std::cout<<"derRect:"<<derRect<<std::endl;
    // std::cout<<"m_trackObj.m_rect:"<<m_trackObj.m_rect<<std::endl;
    // std::cout<<"patch size:"<<m_trackObj.m_patch.size()<<std::endl;
    // std::cout<<"m_trackObj.m_patch:"<<derRect<<std::endl;

#endif

    if ((minDist < minDistThres) && areaDif < areaDifThres)
    {
        m_dtrackerRet = derRect;
        int area = intersectionArea(closestSSIMRect, derRect);
        if (area == 0 && maxSSIM > trackerCfg.maxSimThresFSwitch && maxSSIMDif > trackerCfg.maxSimDifThresFSwitch)
        {
            spdlog::debug("DTRACKER switch due to SSIM");
            m_dtrackerRet = closestSSIMRect;
        }

        m_dtrackerLost = false;
        minDistThres = trackerCfg.initialMinDistThres;
        areaDifThres = trackerCfg.initialAreaDifThres;
    }
    else
    {
        spdlog::debug("mtracker find nearest failed, find by feature");
        if (maxSSIM > trackerCfg.maxSimThresFSwitch)
        {
            m_dtrackerRet = closestSSIMRect;
            m_dtrackerLost = false;
            minDistThres = trackerCfg.initialMinDistThres;
            areaDifThres = trackerCfg.initialAreaDifThres;
        }
        else
        {
            m_dtrackerLost = true;
        }
    }

    if (m_strackerLost && m_dtrackerLost)
    {
        spdlog::debug("all tracker lost");
        // m_trackObj.updateWithoutDet();
        m_trackObj.m_lostCnt++;
        color = cv::Scalar(0, 255, 255); // b,g,r, yellow
    }
    else
    {
        if (m_strackerLost && !m_dtrackerLost)
        {
            static int strackerWait = 0;
            spdlog::debug("s tracker lost");
            finalRect = m_dtrackerRet;
#if TRACKER_DEBUG
            color = cv::Scalar(0, 255, 255);
#endif
            m_stracker->reset();
            if (strackerWait++ > trackerCfg.strackerWaitThres)
            {
                spdlog::debug("s tracker restart");
                m_stracker->init(cv::Point(m_dtrackerRet.x + m_dtrackerRet.width / 2, m_dtrackerRet.y + m_dtrackerRet.height / 2), frameTracker);
                strackerWait = 0;
            }

            m_trackObj.update(frameTracker, finalRect, 0.6);
        }
        else if (!m_strackerLost && m_dtrackerLost)
        {
            spdlog::debug("dtracker lost");
            finalRect.x = m_strackerRet.x + m_strackerRet.width / 2 - m_trackObj.m_rect.width / 2;
            finalRect.y = m_strackerRet.y + m_strackerRet.height / 2 - m_trackObj.m_rect.height / 2;
            finalRect.width = m_trackObj.m_rect.width;
            finalRect.height = m_trackObj.m_rect.height;

            m_trackObj.update(frameTracker, finalRect, 0.6);

#if TRACKER_DEBUG
            color = cv::Scalar(0, 0, 255);
#endif
        }
        else
        {
            spdlog::debug("all tracker good");
            cv::Point center = cv::Point{m_strackerRet.x + m_strackerRet.width / 2, m_strackerRet.y + m_strackerRet.height / 2};
            int ret = m_dtrackerRet.contains(center);
            static int containRetCnt = 0;
            if (!ret)
            {
                spdlog::critical("tracker result diff");
                finalRect.x = m_strackerRet.x + m_strackerRet.width / 2 - m_trackObj.m_rect.width / 2;
                finalRect.y = m_strackerRet.y + m_strackerRet.height / 2 - m_trackObj.m_rect.height / 2;
                finalRect.width = m_trackObj.m_rect.width;
                finalRect.height = m_trackObj.m_rect.height;
                // containRetCnt++;
                // if (containRetCnt > trackerCfg.containRectCntThres)
                // {
                //     spdlog::debug("reset stracker");
                //     m_stracker->reset();
                //     m_stracker->init(cv::Point(m_dtrackerRet.x + m_dtrackerRet.width / 2, m_dtrackerRet.y + m_dtrackerRet.height / 2), frameTracker);
                //     containRetCnt = 0;
                // }
            }
            else
            {
                containRetCnt = 0;
            }

            finalRect = m_dtrackerRet;
            printf("dtracker contains stracker:%d\n", ret);
            m_trackObj.update(frameTracker, finalRect, 0.8);
            
        }
        // printf("FSM_PROC_DTRACK calculateHistogramSimilarity\n");
        // double sim = calculateHistogramSimilarity(frame(m_trackObj.m_rect), frame(finalRect));
        // printf("FSM_PROC_DTRACK sim:%f\n", sim);
        // m_trackObj.update(frameTracker, finalRect, maxSSIM);
    }

    if (m_trackObj.isLost())
    {
        spdlog::debug("tracker lost");
        m_state = EN_TRACKER_FSM::SEARCH;
        return;
    }

#if TRACKER_DEBUG_DRAW
    cv::imshow("debugImg", debugimg);
    // cv::rectangle(frame, derRect, cv::Scalar(255, 0, 127), 2);
#endif

    cv::rectangle(frameTracker, m_trackObj.m_rect, color, 2);
    printf("m_trackObj age:%d, lostcnt:%d, trace size:%d, velo x:%f, velo y:%f\n",
           m_trackObj.m_age, m_trackObj.m_lostCnt, m_trackObj.m_trace.size(), m_trackObj.m_velo[0], m_trackObj.m_velo[1]);
    std::cout << m_trackObj.m_rect << std::endl;
    trackRect = m_trackObj.m_rect;

    m_state = EN_TRACKER_FSM::DTRACK;
}

void realtracker::RealtrackerImpl::fsmUpdate(cv::Mat &frameDetect, cv::Mat &frameTracker, cv::Rect &trackRect)
{

    switch (m_state)
    {
    case EN_TRACKER_FSM::STRACK:
        FSM_PROC_STRACK(frameDetect, frameTracker, trackRect);
        break;
    case EN_TRACKER_FSM::DTRACK:
        FSM_PROC_DTRACK(frameDetect, frameTracker, trackRect);
        break;
    case EN_TRACKER_FSM::SEARCH:
        FSM_PROC_SEARCH(frameDetect);
        break;
    case EN_TRACKER_FSM::SSEARCH:
        FSM_PROC_SSEARCH(frameDetect, trackRect);
        break;
    default:
        break;
    }
}

// EN_TRACKER_FSM realtracker::update(cv::Mat &frame, std::vector<TrackingObject> &detRet, uint8_t *trackerStatus)
// {
//     return m_state;
// }

void realtracker::RealtrackerImpl::update(cv::Mat &frameDetect, cv::Mat &frameTracker, uint8_t *trackerStatus, int &x_, int &y_, cv::Rect &trackRect)
{
#if TRACKER_DEBUG
    printf("realtracker::update start\n");
    sw.reset();
#endif

    static int offsetAbnormalCnt;

    fsmUpdate(frameDetect, frameTracker, trackRect);

    // uint8_t trackerStatus[9];
    memset(trackerStatus, 0, 9);

    // if(m_state == EN_TRACKER_FSM::STRACK || m_state == EN_TRACKER_FSM::DTRACK)
    if (m_state == EN_TRACKER_FSM::DTRACK || m_state == EN_TRACKER_FSM::STRACK || m_state == EN_TRACKER_FSM::SSEARCH)
    {
        trackerStatus[4] |= 0x02; // 0000 0010
        if (m_state == EN_TRACKER_FSM::DTRACK)
        {
            trackerStatus[4] |= 0x04; // 0000 0100
            cv::Rect bbox = m_trackObj.m_rect;
            uint16_t w = bbox.width;
            uint16_t h = bbox.height;
            w = ntohs(w);
            h = ntohs(h);
            memcpy(trackerStatus + 5, &w, 2);
            memcpy(trackerStatus + 7, &h, 2);
        }

        // int16_t x = m_trackObj.center().x - 960;
        // int16_t y = m_trackObj.center().y - 540;

        int16_t x = m_trackObj.center().x - m_imgCenterX;
        int16_t y = m_trackObj.center().y - m_imgCenterY;
        x_ = m_trackObj.center().x;
        y_ = m_trackObj.center().y;
        // int16_t x = m_strackerRet.x+m_stracker->m_GateSize/2- 960;
        // int16_t y = m_strackerRet.y+m_stracker->m_GateSize/2 - 540;

        // if (x > m_trackerOffsetLimit)
        // {
        //     x = m_trackerOffsetLimit;
        //     offsetLimitInc(m_trackerOffsetLimit);
        // }
        // else if (x < -m_trackerOffsetLimit)
        // {
        //     x = -m_trackerOffsetLimit;
        //     offsetLimitInc(m_trackerOffsetLimit);
        // }
        // if (y > m_trackerOffsetLimit)
        // {
        //     y = m_trackerOffsetLimit;
        //     offsetLimitInc(m_trackerOffsetLimit);
        // }
        // else if (y < -m_trackerOffsetLimit)
        // {
        //     y = -m_trackerOffsetLimit;
        //     offsetLimitInc(m_trackerOffsetLimit);
        // }

        //m_trackerOffsetLimit = m_trackerOffsetLimit > trackerCfg.offsetLimitCeil ? trackerCfg.offsetLimitCeil : m_trackerOffsetLimit;

        // printf("tracker center pt x:%d, y:%d\n", m_stracker->centerPt().x, m_stracker->centerPt().y);
#if TRACKER_DEBUG
        printf("tracker offset x:%d, y:%d\n", x, y);
#endif
        // if(abs(x) > trackerCfg.offsetLimitCeil || abs(y) > trackerCfg.offsetLimitCeil)
        // {
        //     offsetAbnormalCnt++;
        // }
        // else
        //     offsetAbnormalCnt = 0;

        // if(offsetAbnormalCnt > 40)
        // {
        //     x = 0;
        //     y = 0;
        //     m_state = EN_TRACKER_FSM::LOST;
        //     offsetAbnormalCnt = 0;
        // }

        x = ntohs(x);
        y = ntohs(y);

        // printf("tracker offset after ntohs x:%d, y:%d\n", x, y);
        memcpy(trackerStatus, &x, 2);
        memcpy(trackerStatus + 2, &y, 2);
    }
    else if (m_state == EN_TRACKER_FSM::SEARCH)
    {
        // printf("realtracker::update in search\n");
        trackerStatus[4] |= 0x02; // 0000 0010
        int16_t x = 0;
        int16_t y = 0;
        memcpy(trackerStatus, &x, 2);
        memcpy(trackerStatus + 2, &y, 2);
    }
#if TRACKER_DEBUG

    spdlog::debug("realtracker::update Elapsed {}", sw);
#endif
    if(m_state == EN_TRACKER_FSM::STRACK)
    {
        trackerStatus[4] = 0x03;
    }
    else if(m_state == EN_TRACKER_FSM::SSEARCH && m_ssearchCnt > 40)
    {
        trackerStatus[4] = 0x02;
    }
    else if(m_state == EN_TRACKER_FSM::SEARCH || m_state ==EN_TRACKER_FSM::LOST) 
    {
        trackerStatus[4] = 0x01;
    }


    return;
}

void realtracker::RealtrackerImpl::runTracker(cv::Mat &frame, bool alone)
{
    // sw.reset();
    // printf("realtracker::runTracker\n");
    cv::Rect kcfResult, templateRet;
    // cv::Point ScreenCenter = cv::Point(960,540);
    kcfResult = m_stracker->update(frame, alone);
#if TRACKER_DEBUG
    // spdlog::debug("m_stracker Elapsed {}", sw);
    // cv::Mat strakerRet = frame.clone();
    // rectangle(strakerRet, cv::Point(kcfResult.x, kcfResult.y), cv::Point(kcfResult.x + kcfResult.width, kcfResult.y + kcfResult.height), cv::Scalar(145, 79, 59), 3, 8);
    // cv::imshow("strakerRet", strakerRet);
#endif

    m_strackerRet = kcfResult;
    // m_strackerRet.x = kcfResult.x + 16 - osdw / 2;
    // m_strackerRet.y = kcfResult.y + 16 - osdw / 2;
    // m_strackerRet.width = m_strackerRet.height = osdw;

    // m_strackerRet = (kcfDist > templateDist ? templateRet : kcfResult);
}
void realtracker::RealtrackerImpl::runTrackerNoDraw(cv::Mat &frame, bool alone)
{
    // printf("realtracker::runTracker\n");
    // cv::Rect result;
    m_strackerRet = m_stracker->update(frame, alone);
}

static inline int boxDif(bbox_t &box1, bbox_t &box2)
{
    return (abs(int(box1.x - box2.x)) + abs(int(box1.y - box2.y)) + abs(int(box1.w - box2.w)) + abs(int(box1.h - box2.h)));
}

void realtracker::RealtrackerImpl::runDetector(cv::Mat &frame, bbox_t *detRet, int &boxs_count)
{
    printf("realtracker::runDetector\n");
    memset(detRet, 0x00, sizeof(*detRet));
    cv::Mat finalDet, rawDet;
    rawDet = frame.clone();

    // m_frameInfo.m_frames[0].GetMatBGRWrite() = rawDet;

    if (!m_irFrame)
    {
        m_detector->ImgInference(frame, detRet, boxs_count);
    }
    else
    {
        m_irDetector->ImgInference(frame, detRet, boxs_count);
    }

  //  printf("box size:%d\n", boxs_count);

    // detRet = boxs;

    return;
}



/**
 * @brief Plot tracks on the frame
 * 
 * @param frame Input frame
 * @param detections Detections
 * @param tracks Tracks
 */


void realtracker::RealtrackerImpl::runDetectorOut(cv::Mat &frame, bbox_t *detRet, int &boxs_count)
{
    // printf("realtracker::runDetectorOut\n");
    memset(detRet, 0x00, sizeof(*detRet));
    if (!m_irFrame)
    {
        m_detector->ImgInference(frame, detRet, boxs_count);
    }
    else
    {
        m_irDetector->ImgInference(frame, detRet, boxs_count);
    }
   
  //  printf("boxs_count:%d\n", boxs_count);
    for (int i = 0; i < boxs_count; i++)
    {
        // printf("box-->x:%d, y:%d, w:%d, h:%d, conf:%f, cls:%d\n", boxs[i].x, boxs[i].y, boxs[i].w, boxs[i].h, boxs[i].prob, boxs[i].obj_id);
        for (int j = i + 1; j < boxs_count; ++j)
        {
            // printf("\tbox-->x:%d, y:%d, w:%d, h:%d, conf:%f, cls:%d, diff:%d\n", boxs[j].x, boxs[j].y, boxs[j].w, boxs[j].h, boxs[j].prob, boxs[j].obj_id, boxDif(boxs[i], boxs[j]));

            if (boxDif(detRet[i], detRet[j]) < trackerCfg.detectorBoxDifThres)
            {
                detRet[j].w = 0;
                detRet[j].h = 0;
                // detRet.erase(detRet.begin() + j);
                --j;
            }
        }
    }

    // m_frameInfo.CleanRegions();
    // m_regions.clear();
    // for(auto &box:detRet)
    // for (int i = 0; i < boxs_count; ++i)
    // {
    //     m_regions.emplace_back(cv::Rect(cvRound(1.0*detRet[i].x), cvRound(1.0*detRet[i].y), cvRound(1.0*detRet[i].w), cvRound(1.0*detRet[i].h)), (detRet[i].obj_id), detRet[i].prop);
    // }

    // // printf("realtracker::CleanRegions\n");
    
    // // m_frameInfo.m_regions[0] = m_regions;


    // // m_mtracker->Update(m_frameInfo.m_regions[0], m_frameInfo.m_frames[0].GetUMatGray(), m_fps);

    // // // printf("realtracker::UpdateUpdate\n");
    // // m_mtracker->GetTracks(m_frameInfo.m_tracks[0]);
    // std::vector<Detection> gt_per_frame; 

    // //  unsigned int x, y, w, h;     // (x,y) - top-left corner, (w, h) - width & height of bounded box
    // // float prop;                  // confidence - probability that the object was found correctly
    // // unsigned int obj_id;         // class of object - from range [0, classes-1]
    // // unsigned int track_id;       // tracking id for video (0 - untracked, 1 - inf - tracked object)
    // // unsigned int frames_counter; // counter of frames on which the object was detected
    // // float x_3d, y_3d, z_3d;      // center of object (in Meters) if ZED 3D Camera is used
    //  cv::Rect_<float> bboxtlwh;
    // for (int i = 0; i < boxs_count; i++)
    // {
    //     Detection  Det;
    //     Det.bbox_tlwh=cv::Rect_<float>(detRet[i].x,detRet[i].y,detRet[i].w,detRet[i].h);
    //     Det.class_id=detRet[i].obj_id;
    //     Det.confidence=detRet[i].prop;
    //     // printf("realtracker::track   %f\n",  Det.confidence);
    //     gt_per_frame.push_back(Det);
    // }

    // tracker->track(gt_per_frame[frame_counter], frame);
    // printf("realtracker::track\n");
    // auto start = std::chrono::high_resolution_clock::now();
    // tracks =tracker->track(gt_per_frame, frame);
    // auto end = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    // std::cout<<"tracker->track  elapse:"<< duration.count()<<"ms"<< std::endl;

    
    // printf("  m_mtracker->GetTracks( end \n   ");

}
void realtracker::RealtrackerImpl::runDetectorNoDraw(cv::Mat &frame, bbox_t *detRet, int &boxs_count)
{
    printf("realtracker::runDetectorNoDraw\n");
    memset(detRet, 0x00, sizeof(*detRet));
    // m_frameInfo.m_frames[0].GetMatBGRWrite() = frame;
    // m_frameInfo.m_frames[0].GetMatBGRWrite() = frame.clone();
    if (!m_irFrame)
    {
        m_detector->ImgInference(frame, detRet, boxs_count);
    }
    else
    {
        m_irDetector->ImgInference(frame, detRet, boxs_count);
    }
}

void realtracker::RealtrackerImpl::reset()
{
    m_mtrackerLostCnt = 0;
    m_stracker->reset();
}

void realtracker::RealtrackerImpl::setFrameScale(double s)
{
    m_frameScale = s;

    m_imgCenterX = (m_frameScale == 1 ? 960 : 640);
    m_imgCenterY = (m_frameScale == 1 ? 540 : 360);
}

void realtracker::RealtrackerImpl::setGateSize(int s)
{
    osdw = s;
    m_stracker->setGateSize(s);
}
int realtracker::RealtrackerImpl::getGateSize()
{
   return osdw;
}

void realtracker::RealtrackerImpl::setIrFrame(bool ir)
{
    m_irFrame = ir;
}

bool realtracker::RealtrackerImpl::trackerLost()
{
    return (m_state == EN_TRACKER_FSM::SSEARCH && m_ssearchCnt > 30);
}

bool realtracker::RealtrackerImpl::sseFind(float sim)
{
    if(m_trackObj.m_velo[0] == 0 && m_trackObj.m_velo[1] == 0 || m_ssearchCnt > trackerCfg.ssearchCntThres * 0.5)
        return sim > 0.1;
    else if( m_ssearchCnt > trackerCfg.ssearchCntThres * 0.4)
        return sim > 0.2;
    else if( m_ssearchCnt > trackerCfg.ssearchCntThres * 0.3)
        return sim > 0.3;
    else
        return sim > 0.45;
}

void realtracker::RealtrackerImpl::gateAdjust(int dir)
{
    m_adjustFlg = true;
    m_adjstPt = m_trackObj.center();
    int pixelstep = 3;
    switch(dir)
    {
        case 0:m_adjstPt.y-=pixelstep;break;
        case 1:m_adjstPt.y+=pixelstep;break;
        case 2:m_adjstPt.x-=pixelstep;break;
        case 3:m_adjstPt.x+=pixelstep;break;
        default:break;
    }
}

realtracker::RealtrackerImpl::~RealtrackerImpl()
{
    if(m_stracker)
    {
        delete m_stracker;
        m_stracker = nullptr;
    }
    if(m_detector)
    {
        delete m_detector;
        m_detector = nullptr;
    }
    if(m_irDetector)
    {
        delete m_irDetector;
        m_irDetector = nullptr;
    }
    
}

realtracker::realtracker(std::string cfg)
{
    m_impl = std::make_unique<RealtrackerImpl>(cfg);
}

realtracker::~realtracker()
{   
    m_impl = nullptr;
}

void realtracker::init(const cv::Rect &roi, cv::Mat image)
{
    m_impl->init(roi, image);
}

void realtracker::init(const cv::Point &pt, cv::Mat &trackImg, cv::Mat &detImage)
{
    m_impl->init(pt, trackImg, detImage);
}

void realtracker::runDetectorOut(cv::Mat &frame, bbox_t *detRet, int &boxs_count)
{
    m_impl->runDetectorOut(frame, detRet, boxs_count);
}   

void realtracker::update(cv::Mat &frame, cv::Mat &frameTracker, uint8_t *trackerStatus, int &x_, int &y_, cv::Rect &rect)
{
    m_impl->update(frame, frameTracker, trackerStatus, x_, y_, rect);
}


void realtracker::reset()
{
    m_impl->reset();
}

bool realtracker::trackerLost()
{
    return m_impl->trackerLost();
}

void realtracker::gateAdjust(int dir)
{
    m_impl->gateAdjust(dir);
}

int realtracker::getGateSize()
{
    return m_impl->getGateSize();
}

