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

itracker::itracker()
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

cv::Rect itracker::update(cv::Mat image, float &peakVal)
{
    static float lastPeakVal = 0;
    auto result = trackerPtr->update(image, peakVal);
    auto retPatch = image(result);
    // cv::cvtColor(retPatch, retPatch, cv::COLOR_BGR2GRAY);

    // double ssim = cv::compareSSIM(m_oriPatch, retPatch);

    double sim = calculateHistogramSimilarity(m_oriPatch, retPatch);
    
    printf("similarity:%f, peakVal:%f, diff:%f\n", sim, peakVal, peakVal - lastPeakVal);
    float peakDif = peakVal - lastPeakVal;
    
    static int st = 0;
    static float fallEdgePv = 0;
    static int bottomCnt = 0;
    static int lastSt = -1;
    
    do{
        lastSt = st;
        switch(st)
        {
            case 0:
                if(peakDif < -0.2)
                {
                    st = 1;
                    fallEdgePv = lastPeakVal + 0.005;
                    printf("ffffffallEdgePv = %f\n", fallEdgePv);
                }
                
                break;
            case 1:
                if(peakDif > 0.1 || peakVal >= fallEdgePv || bottomCnt > 10)
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
                printf("bottomCnt = %d\n", bottomCnt);
                fallEdgePv = 0;
                bottomCnt = 0;
                break;
            default:
                break;
        }
        
    }
    while(lastSt != st);
    

    lastPeakVal = peakVal;

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

    trackerPtr = new KCFTracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
}