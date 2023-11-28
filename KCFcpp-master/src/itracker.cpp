#include "itracker.h"
#include "kcftracker.hpp"

static KCFTracker* trackerPtr = nullptr;

#define TRACKER_DEBUG 1


static double calculateHistogramSimilarity(const cv::Mat& image1, const cv::Mat& image2) {
    cv::Mat hsvImage1, hsvImage2;
    cv::cvtColor(image1, hsvImage1, cv::COLOR_BGR2HSV);
    cv::cvtColor(image2, hsvImage2, cv::COLOR_BGR2HSV);

    int hBins = 30;
    int sBins = 32;
    int histSize[] = {hBins, sBins};
    float hRanges[] = {0, 180};
    float sRanges[] = {0, 256};
    const float* ranges[] = {hRanges, sRanges};
    int channels[] = {0, 1};

    cv::MatND hist1, hist2;
    cv::calcHist(&hsvImage1, 1, channels, cv::Mat(), hist1, 2, histSize, ranges, true, false);
    cv::calcHist(&hsvImage2, 1, channels, cv::Mat(), hist2, 2, histSize, ranges, true, false);

    double similarity = cv::compareHist(hist1, hist2, cv::HISTCMP_BHATTACHARYYA);

    return similarity;
}

itracker::itracker():m_isLost(false)
{
    bool HOG = false;
    bool FIXEDWINDOW = false;
    bool MULTISCALE = false;
    bool SILENT = true;
    bool LAB = false;

    trackerPtr = new KCFTracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

    m_templateSearchWindowSize = 150;
    m_templateSearchOffset = m_templateSearchWindowSize/2;
}

itracker::~itracker()
{

}

// CalcConf(double sim)
// {

// }

void itracker::init(cv::Rect &roi, cv::Mat image)
{
    if(roi.x < 0)
        roi.x = 0;
    if(roi.x + roi.width > image.cols)
        roi.x = image.cols - roi.width - 1;
    if(roi.y < 0)
        roi.y = 0;
    if(roi.y + roi.height > image.rows)
        roi.y = image.rows - roi.height - 1;
    m_oriPatch = image(roi).clone();
    // cv::cvtColor(m_oriPatch, m_oriPatch, cv::COLOR_BGR2GRAY);
    // cv::imwrite("oripatch.png", m_oriPatch);
    trackerPtr->init(roi, image);
    m_GateSize = roi.width;
}

void itracker::init(const cv::Point &pt, cv::Mat image)
{
    // cv::cvtColor(m_oriPatch, m_oriPatch, cv::COLOR_BGR2GRAY);
    // cv::imwrite("oripatch.png", m_oriPatch);
    printf("\n\nstracker init with pt x:%d, y:%d\n", pt.x, pt.y);
    cv::Rect roi= cv::Rect{pt.x - m_GateSize/2, pt.y - m_GateSize/2, m_GateSize, m_GateSize};
    if(roi.x < 0)
        roi.x = 0;
    if(roi.x + roi.width > image.cols)
        roi.x = image.cols - roi.width - 1;
    if(roi.y < 0)
        roi.y = 0;
    if(roi.y + roi.height > image.rows)
        roi.y = image.rows - roi.height - 1;
    m_template = m_oriPatch = image(roi).clone();
    trackerPtr->init(roi, image);
    m_centerPt = pt;

}

cv::Rect itracker::updateTP(cv::Mat image)
{
    cv::Mat templret;
    cv::Rect rr = cv::Rect(m_centerPt.x - m_templateSearchOffset,m_centerPt.y-m_templateSearchOffset, m_templateSearchWindowSize,m_templateSearchWindowSize);
    if(rr.x < 0)
        rr.x = 0;
    if(rr.x + rr.width > image.cols)
        rr.x = image.cols - rr.width - 1;
    if(rr.y < 0)
        rr.y = 0;
    if(rr.y + rr.height > image.rows)
        rr.y = image.rows - rr.height - 1;
    rectangle(image,rr,cv::Scalar(135,32,156),2);

    matchTemplate(image(rr),m_template,templret,cv::TM_CCOEFF_NORMED);
    double maxVal,minVal;
    cv::Point minLoc,maxLoc;
    minMaxLoc(templret,&minVal,&maxVal,&minLoc,&maxLoc);
    
    // rectangle(image,cv::Rect(maxLoc.x,maxLoc.y,m_GateSize, m_GateSize),cv::Scalar(135,32,156),2);

    m_centerPt.x = m_centerPt.x - m_templateSearchOffset + maxLoc.x + m_GateSize/2;
	m_centerPt.y = m_centerPt.y-m_templateSearchOffset + maxLoc.y + m_GateSize/2;

    return cv::Rect(m_centerPt.x - m_GateSize/2,m_centerPt.y - m_GateSize/2,m_GateSize, m_GateSize);

}

cv::Rect itracker::update(cv::Mat image)
{
    static int st = 0;
    static float fallEdgePv = 0;
    static int bottomCnt = 0;
    static int lastSt = -1;
    static int simFailCnt = 0;
    static float lastPeakVal = 0;
    float peakVal;
    auto result = trackerPtr->update(image, peakVal);

    if(result.x < 0)
        result.x = 0;
    if(result.x + result.width > image.cols)
        result.x = image.cols - result.width - 1;
    if(result.y < 0)
        result.y = 0;
    if(result.y + result.height > image.rows)
        result.y = image.rows - result.height - 1;

    // std::cout<<result<<std::endl;
    auto retPatch = image(result);
    // islost = false;

    // cv::cvtColor(retPatch, retPatch, cv::COLOR_BGR2GRAY);

    // double ssim = cv::compareSSIM(m_oriPatch, retPatch);

    double sim = calculateHistogramSimilarity(m_oriPatch, retPatch);
    if(sim > 0.8f)
        simFailCnt++;
    else
        simFailCnt = 0;

#if TRACKER_DEBUG 
    printf("SSSSSSSSSsimilarity:%f, peakVal:%f, diff:%f, simFailCnt:%d\n", sim, peakVal, peakVal - lastPeakVal, simFailCnt);
#endif
    float peakDif = peakVal - lastPeakVal;
    
    
    
    do{
        lastSt = st;
        switch(st)
        {
            case 0:
                if(peakDif < -0.2)
                {
                    st = 1;
                    fallEdgePv = lastPeakVal + 0.005;
#if TRACKER_DEBUG
                    printf("\n\n-----------------ffffffallEdgePv = %f\n", fallEdgePv);
#endif
                }
                if(simFailCnt > 2)
                {
                    st = 2;
#if TRACKER_DEBUG
                    printf("stracker state------->2\n");
#endif
                }
                
                break;
            case 1:
                if(peakDif > 0.1 || peakVal >= fallEdgePv || bottomCnt > 10 || simFailCnt > 2)
                {
                    st = 2;
                }
                else
                {
                    bottomCnt++;
                    st = 1;
                }
                break;
            case 2:
                st = 0;
#if TRACKER_DEBUG
                printf("BBBBBBBBBBBBBBbottomCnt = %d\n", bottomCnt);
#endif
                
                if(bottomCnt > 10 || simFailCnt > 2)
                {
                    m_isLost = true;
                    fallEdgePv = 0;
                    bottomCnt = 0;
                    printf("------------------Lost---------------\n");
                }
                break;
            default:
                break;
        }
        
    }
    while(lastSt != st && !m_isLost);

    // m_isLost = false;
    

    lastPeakVal = peakVal;

    m_centerPt.x = result.x + m_GateSize/2;
	m_centerPt.y = result.y + m_GateSize/2;

    return result;
}

void itracker::reset()
{
    if(trackerPtr != nullptr)
    {
        delete trackerPtr;
    }

    bool HOG = false;
    bool FIXEDWINDOW = false;
    bool MULTISCALE = false;
    bool SILENT = true;
    bool LAB = false;

    m_isLost = false;

    trackerPtr = new KCFTracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
}

bool itracker::isLost()
{
    return m_isLost;
}

cv::Point itracker::centerPt()
{
    return m_centerPt;
}

void itracker::setGateSize(int s)
{
    m_GateSize = s;
}

void itracker::resetTemplate(cv::Mat &img)
{
    m_template = img.clone();
    cv::imwrite("m_template.png", m_template);
}