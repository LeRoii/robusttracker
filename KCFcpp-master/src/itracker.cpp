#include "itracker.h"
#include "kcftracker.hpp"

static KCFTracker* trackerPtr = nullptr;

#define TRACKER_DEBUG 0

static double calculateSSIM(const cv::Mat& imgg1, const cv::Mat& imgg2)
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
    for (int i = 0; i < 3; ++i) {
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

itracker::itracker():m_isLost(true),m_init(false)
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
    // if(roi.x < 0)
    //     roi.x = 0;
    // if(roi.x + roi.width > image.cols)
    //     roi.x = image.cols - roi.width - 1;
    // if(roi.y < 0)
    //     roi.y = 0;
    // if(roi.y + roi.height > image.rows)
    //     roi.y = image.rows - roi.height - 1;
    // m_oriPatch = image(roi).clone();
    // // cv::cvtColor(m_oriPatch, m_oriPatch, cv::COLOR_BGR2GRAY);
    // // cv::imwrite("oripatch.png", m_oriPatch);
    // trackerPtr->init(roi, image);
    // m_GateSize = roi.width;

    // m_tmplSz = trackerPtr->padding*m_GateSize;
}

void itracker::init(const cv::Point &pt, cv::Mat image)
{
    // cv::cvtColor(m_oriPatch, m_oriPatch, cv::COLOR_BGR2GRAY);
    // cv::imwrite("oripatch.png", m_oriPatch);
    printf("\n\nstracker init with pt x:%d, y:%d\n", pt.x, pt.y);
    printf("\n\nstracker init with pt m_GateSize:%d\n", m_GateSize);
    cv::Rect roi= cv::Rect{pt.x - m_GateSize/2, pt.y - m_GateSize/2, m_GateSize, m_GateSize};
    if(roi.x < 0)
        roi.x = 0;
    if(roi.x + roi.width > image.cols)
        roi.x = image.cols - roi.width - 2;
    if(roi.y < 0)
        roi.y = 0;
    if(roi.y + roi.height > image.rows)
        roi.y = image.rows - roi.height - 2;
    m_template = m_oriPatch = image(roi).clone();
    roix = roi.x;
    roiy = roi.y;

    printf("\n\n%d,%d\n",image.cols,image.rows);
    trackerPtr->init(roi, image);
    printf("\nooooooooooooooooooooooooooooooooooooooooooooooooooooooo\n");
    m_centerPt = pt;

    m_init = true;
    m_isLost = false;

    m_tmplSz = trackerPtr->padding*m_GateSize;

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

cv::Rect itracker::update(cv::Mat image, bool alone)
{
    if(!m_init)
        return cv::Rect();
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

    // return result;

    // cv::cvtColor(retPatch, retPatch, cv::COLOR_BGR2GRAY);

    // double ssim = cv::compareSSIM(m_oriPatch, retPatch);

#if TRACKER_DEBUG
    cv::imshow("ori", m_oriPatch);
    cv::imshow("retppr", retPatch);
#endif

    static double lastSim = 0.0f;
    double sim = calculateSSIM(m_oriPatch, retPatch);
    int simFailedCntThres = alone ? 5 : 3;

    double simDif = sim - lastSim;

    lastSim = sim;
    if(sim > 0.75 || peakVal > 1.0f)
        m_oriPatch = retPatch.clone();
    // if(sim > 0.8f)
    // if(sim < 0.3f && peakVal < 1.0f)
    if(sim < 0.5f && peakVal < 1.0f)
        simFailCnt++;
    else
        simFailCnt = 0;
    
    if(!alone && sim < 0.5f)
        m_isLost = true;

#if TRACKER_DEBUG 
    printf("simDif:%f\n", simDif);
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
                if(simFailCnt > simFailedCntThres)
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
                    simFailCnt = 0;
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

    m_init = false;
    m_isLost = true;
}

bool& itracker::isLost()
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

cv::Rect itracker::getTmplRect()
{
    return cv::Rect(m_centerPt.x - m_tmplSz/2, m_centerPt.y - m_tmplSz/2, m_tmplSz, m_tmplSz);
}

void itracker::setRoi(cv::Rect roi)
{
    m_centerPt.x = roi.x + m_GateSize/2;
	m_centerPt.y = roi.y + m_GateSize/2;
    trackerPtr->setRoi(roi);
}