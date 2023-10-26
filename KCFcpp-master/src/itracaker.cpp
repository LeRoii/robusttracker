#include "itracker.h"
#include "kcftracker.hpp"

static KCFTracker* trackerPtr = nullptr;


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
}

itracker::~itracker()
{

}

// CalcConf(double sim)
// {

// }

void itracker::init(const cv::Rect &roi, cv::Mat image)
{
    m_oriPatch = image(roi).clone();
    // cv::cvtColor(m_oriPatch, m_oriPatch, cv::COLOR_BGR2GRAY);
    cv::imwrite("oripatch.png", m_oriPatch);
    trackerPtr->init(roi, image);
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
    // if()

    std::cout<<result<<std::endl;
    auto retPatch = image(result);
    // islost = false;

    // cv::cvtColor(retPatch, retPatch, cv::COLOR_BGR2GRAY);

    // double ssim = cv::compareSSIM(m_oriPatch, retPatch);

    double sim = calculateHistogramSimilarity(m_oriPatch, retPatch);
    if(sim > 0.99f)
        simFailCnt++;
    else
        simFailCnt = 0;

    
    printf("SSSSSSSSSsimilarity:%f, peakVal:%f, diff:%f\n", sim, peakVal, peakVal - lastPeakVal);
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
                    printf("\n\n-----------------ffffffallEdgePv = %f\n", fallEdgePv);
                }
                
                break;
            case 1:
                if(peakDif > 0.1 || peakVal >= fallEdgePv || bottomCnt > 10 || simFailCnt > 3)
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
                printf("BBBBBBBBBBBBBBbottomCnt = %d\n", bottomCnt);
                fallEdgePv = 0;
                bottomCnt = 0;
                if(bottomCnt > 10 || simFailCnt > 3)
                {
                    m_isLost = true;
                    printf("------------------Lost---------------\n");
                }
                break;
            default:
                break;
        }
        
    }
    while(lastSt != st);
    

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