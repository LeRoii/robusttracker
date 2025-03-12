#include "itracker.h"
#include "kcftracker.hpp"

static KCFTracker* trackerPtr = nullptr;

#define TRACKER_DEBUG 1

static int randomcnt = 0;
static int randomNum = 30;
static int finalwidth = 32;
static int finalheight = 32;
static float scalef = 0.5;

static std::vector<double> preApce;
static std::vector<double> preResMax;


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
    cv::cvtColor(image1, hsvImage1, cv::COLOR_BGR2GRAY);
    cv::cvtColor(image2, hsvImage2, cv::COLOR_BGR2GRAY);

    cv::MatND hist1, hist2;
    int histSize = 256; // 直方图的大小
    float range[] = {0, 256}; // 像素值的范围
    const float* histRange = {range};
    cv::calcHist(&hsvImage1, 1, 0, cv::Mat(), hist1, 1, &histSize, &histRange);
    cv::calcHist(&hsvImage2, 1, 0, cv::Mat(), hist2, 1, &histSize, &histRange);

    // 比较直方图，使用巴氏距离作为相似度度量
    double similarity = cv::compareHist(hist1, hist2, cv::HISTCMP_BHATTACHARYYA);

    return similarity;
}

// static double calculateHistogramSimilarity(const cv::Mat& image1, const cv::Mat& image2) {
//     cv::Mat hsvImage1, hsvImage2;
//     cv::cvtColor(image1, hsvImage1, cv::COLOR_BGR2HSV);
//     cv::cvtColor(image2, hsvImage2, cv::COLOR_BGR2HSV);

//     int hBins = 30;
//     int sBins = 32;
//     int histSize[] = {hBins, sBins};
//     float hRanges[] = {0, 180};
//     float sRanges[] = {0, 256};
//     const float* ranges[] = {hRanges, sRanges};
//     int channels[] = {0, 1};

//     cv::MatND hist1, hist2;
//     cv::calcHist(&hsvImage1, 1, channels, cv::Mat(), hist1, 2, histSize, ranges, true, false);
//     cv::calcHist(&hsvImage2, 1, channels, cv::Mat(), hist2, 2, histSize, ranges, true, false);

//     double similarity = cv::compareHist(hist1, hist2, cv::HISTCMP_BHATTACHARYYA);

//     return similarity;
// }

itracker::itracker():m_isLost(true),m_init(false)
{
	bool HOG = true;
	bool FIXEDWINDOW = false;
	bool MULTISCALE = true;
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
        roi.x = image.cols - roi.width - 2;
    if(roi.y < 0)
        roi.y = 0;
    if(roi.y + roi.height > image.rows)
        roi.y = image.rows - roi.height - 2;
    m_template = m_oriPatch = image(roi).clone();

    printf("\n\n%d,%d\n",image.cols,image.rows);
    trackerPtr->init(roi, image);
    printf("\ntracker init by rect ");
    std::cout<<roi<<std::endl;
    m_centerPt = cv::Point(roi.x+roi.width/2, roi.y+roi.height/2);

    m_init = true;
    m_isLost = false;

    m_tmplSz = trackerPtr->padding*m_GateSize;

    preApce.clear();
    preResMax.clear();
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
    printf("\ntracker init by pt\n");
    m_centerPt = pt;

    m_init = true;
    m_isLost = false;

    m_tmplSz = trackerPtr->padding*m_GateSize;

    m_stpUpdt = 0;
    m_setupf = 5;

    preApce.clear();
    preResMax.clear();

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
    
    rectangle(image,cv::Rect(maxLoc.x,maxLoc.y,m_GateSize, m_GateSize),cv::Scalar(135,32,156),2);

    m_centerPt.x = m_centerPt.x - m_templateSearchOffset + maxLoc.x + m_GateSize/2;
	m_centerPt.y = m_centerPt.y-m_templateSearchOffset + maxLoc.y + m_GateSize/2;

    return cv::Rect(m_centerPt.x - m_GateSize/2,m_centerPt.y - m_GateSize/2,m_GateSize, m_GateSize);

}

cv::Rect itracker::find(cv::Mat image, double &sim)
{
    float peakVal;
    auto result = trackerPtr->seulDetect(image, peakVal);
    if(result.x < 0)
        result.x = 0;
    if(result.x + result.width > image.cols)
    {
        if(result.width > image.cols)
        {
            result.width = image.cols;
            result.x = 0;
        }
        else
            result.x = image.cols - result.width;
    }
    if(result.y < 0)
    {
        result.y = 0;
    }
    if(result.y + result.height > image.rows)
    {
        if(result.height > image.rows)
        {
            result.height = image.rows;
            result.y = 0;
        }
        else
            result.y = image.rows - result.height;
    }
    auto retPatch = image(result);
    // sim = calculateHistogramSimilarity(m_oriPatch, retPatch);

    double ssim = calculateSSIM(m_oriPatch, retPatch);
    // sim = ssim;
    if(peakVal > 40.f)
        sim = 0.35 + ssim;
    else if(peakVal > 30.f)
        sim = 0.3 + ssim;
    else if(peakVal > 20.f)
        sim = 0.25 + ssim;
    else
        sim = ssim - 0.15;

    // if(peakVal > 0.3)
    // {
    //     sim = ssim + peakVal - 0.3;
    // }
    // else
    // {
    //     sim = ssim - (0.3 - peakVal);
    // }
    printf("itracker::find peakVal:%f, sim:%f\n", peakVal, sim);

    return result;
}

cv::Rect itracker::update(cv::Mat image, bool alone)
{
    
    if(!m_init)
        return cv::Rect();

    
    static int simFailCnt = 0;
    double peakVal;
    double apc;
    auto result = trackerPtr->update(image, apc, peakVal);

    preApce.push_back(apc);
    preResMax.push_back(peakVal);

    double addApce = 0;
    double addResMax = 0;
    double comApce = 0;
    double comResMax = 0;


    int sz =  preApce.size()-1;
    for(int i=0;i<sz;i++){

        addApce +=preApce[i];
        addResMax +=preResMax[i];
    }
    //cout<<"addResMax = "<<addResMax<<endl;
    if(m_isLost)
        return result;

    if(sz>0) {

        addApce = addApce / sz;
        addResMax = addResMax / sz;

        //cout<<"sz ="<<sz<<endl;
        comApce = 0.5 * addApce;
        comResMax = 0.5 * addResMax;
       printf("comApce = %f, comResMax = %f\n", comApce, comResMax);
       printf("apc:%f, peakVal:%f\n", apc, peakVal);

        // if (apc > comApce && peakVal > comResMax) {
        //     m_isLost = false;

        // } else {
        //     m_isLost = true;
        //     printf("tracking lost\n");
        // }

        if (apc < comApce && peakVal < comResMax) 
            simFailCnt++;
        else
            simFailCnt = 0;

        if(simFailCnt > 2)
            m_isLost = true;

        if (apc < comApce || peakVal < comResMax) 
            m_conf = 0.5;
        else
        {
            trackerPtr->updateRoi(image);
            m_conf = 0.9;
        }

        printf("simFailCnt:%d\n", simFailCnt);

    }

    return result;



    // std::cout<<"bf itracker:"<<result<<std::endl;

    if(result.x < 0)
        result.x = 0;
    if(result.x + result.width > image.cols)
    {
        if(result.width > image.cols)
        {
            result.width = image.cols;
            result.x = 0;
        }
        else
            result.x = image.cols - result.width;
    }
    if(result.y < 0)
    {
        result.y = 0;
    }
    if(result.y + result.height > image.rows)
    {
        if(result.height > image.rows)
        {
            result.height = image.rows;
            result.y = 0;
        }
        else
            result.y = image.rows - result.height;
    }

    // std::cout<<"itracker:"<<result<<std::endl;
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
    double hsim;// = calculateHistogramSimilarity(m_oriPatch, retPatch);
    // printf("hsim:%f\n", hsim);
    // printf("sim:%f\n", sim);
    int simFailedCntThres = 3;

    double simDif = hsim - lastSim;

    lastSim = hsim;
    // if(sim > 0.95 || peakVal > 1.0f)
    // if(sim > 0.95)
    //     m_oriPatch = retPatch.clone();
    // // if(sim > 0.8f)
    // // if(sim < 0.3f && peakVal < 1.0f)
    // if(sim < 0.9f && peakVal < 1.0f)
    //     simFailCnt++;
    // else
    //     simFailCnt = 0;
    
    // if(!alone && sim < 0.5f)
    //     m_isLost = true;


    // if(sim > 0.99 || peakVal > 1.0f)
    if((sim > 0.7 || peakVal > 30.f) || m_setupf++ < 5)
    // if(hsim < 0.2)
    {
        m_oriPatch = retPatch.clone();
        // trackerPtr->updateRoi(image);
        m_stpUpdt = 0;
    }
    else
    {
        if(m_stpUpdt++ > 30 && sim > 0.5 && peakVal > 30.f)
        {
#if TRACKER_DEBUG
            printf("m_stpUpdt met, updt patch\n");
#endif
            m_setupf = 0;
            m_stpUpdt = 0;
        }
    }
    if(peakVal > 60.f)
        trackerPtr->updateRoi(image);
    // if(sim > 0.8f)
    // if(sim < 0.8f && peakVal < 1.0f)
    // if(peakVal < 0.5f || sim < 0.5f)
    // if(peakVal < 0.4f || sim < 0.6f)
    // // if(hsim > 0.4)
    //     simFailCnt++;
    // else
    //     simFailCnt = 0;

    if(sim < 0.2 || peakVal < 20.f)
    {
        // m_isLost = true;
        simFailCnt++;
        if(sim < 0.1)
        {
            simFailCnt ++;
        }
    }
    else
    {
        simFailCnt = 0;
    }
    
#if 1
    // printf("simDif:%f\n", simDif);
    // printf("sim:%f\n", sim);
    printf("SSSSSSSSSsimilarity:%f, peakVal:%f, simFailCnt:%d\n", sim, peakVal, simFailCnt);
#endif
    if(simFailCnt > simFailedCntThres)
    {
        m_isLost = true;
        simFailCnt = 0;
        printf("------------------Lost---------------\n");
    }

    m_conf = simFailCnt > 0 ? 0.5 : 0.9;

    // m_isLost = false;
    
    m_centerPt.x = result.x + result.width/2;
	m_centerPt.y = result.y + result.height/2;

    // cv::Rect roi(result.x > 20 ? result.x-20 : 0, result.y > 20 ? result.y-20 : 0, 20, 20);

    // if(roi.x + roi.width > image.cols)
    //     roi.x = image.cols - result.width - 1;
    // if(roi.y + roi.height > image.rows)
    //     roi.y = image.rows - result.height - 1;

    // std::cout<<roi<<std::endl;
    // cv::Mat roiImage = image(roi);

    // cv::Mat edges;
    // cv::Canny(roiImage, edges, 50, 150);

    // // 提取轮廓
    // std::vector<std::vector<cv::Point>> contours;
    // cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // cv::Rect maxBoundingRect;
    // double maxArea = 0.0;
    // for (const auto& contour : contours) {
    //     double area = cv::contourArea(contour);
    //     if (area > maxArea) {
    //         maxArea = area;
    //         maxBoundingRect = cv::boundingRect(contour);
    //     }
    // }

    // if(randomcnt == randomNum)
    // {
    //     finalwidth = abs(maxBoundingRect.width - 32) > scalef*32 ? ((maxBoundingRect.width > 32) ? 32*(1+scalef) : 32*(1-scalef)) : maxBoundingRect.width;
    //     finalheight = abs(maxBoundingRect.height - 32) > scalef*32 ? ((maxBoundingRect.height > 32) ? 32*(1+scalef) : 32*(1-scalef)) : maxBoundingRect.height;
    //     randomNum = rand() % 10 +10;
    //     randomcnt = 0;
    // }
    // else
    // {
    //     randomcnt++;
    //     result.width = finalwidth = result.width;
    //     result.height = finalheight = result.height;
    // }

    return result;
}

void itracker::reset()
{
    if(trackerPtr != nullptr)
    {
        delete trackerPtr;
    }

	bool HOG = true;
	bool FIXEDWINDOW = false;
	bool MULTISCALE = true;
	bool SILENT = true;
	bool LAB = false;

    m_isLost = false;

    trackerPtr = new KCFTracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

    m_init = false;
    m_isLost = true;

    m_setupf = 5;
    m_stpUpdt = 0;

    preApce.clear();
    preResMax.clear();
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

    // roi.width = roi.height = m_GateSize;
    trackerPtr->setRoi(roi);
}

float itracker::getConf()
{
    return m_conf;
}