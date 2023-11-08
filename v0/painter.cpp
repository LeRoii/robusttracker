#include "painter.h"
#include <iostream>
#include <sstream>
#include <cmath>
#include<arpa/inet.h>
#include "serial.h"
#include "common.h"
#include <chrono>
#include <sys/vfs.h>

extern ST_SYS_STATUS stSysStatus;
extern ST_A1_CONFIG stA1Cfg;
extern ST_A2_CONFIG stA2Cfg;
extern ST_C1_CONFIG stC1Cfg;
extern ST_C2_CONFIG stC2Cfg;
extern ST_E1_CONFIG stE1Cfg;
extern ST_E2_CONFIG stE2Cfg;
extern ST_S1_CONFIG stS1Cfg;
extern ST_S2_CONFIG stS2Cfg;
extern ST_U_CONFIG stUCfg;
extern ST_A1C1E1_CONFIG stA1C1E1Cfg;
extern ST_A2C2E2_CONFIG stA2C2E2Cfg;
extern ST_A1C1E1S1_CONFIG stA1C1E1S1Cfg;
extern ST_A2C2E2S2_CONFIG stA2C2E2S2Cfg;
extern ST_T1F1B1D1_CONFIG stT1F1B1D1Cfg;
extern ST_T2F2B2D2_CONFIG stT2F2B2D2Cfg;

// OSD 字体宽度
int fontThickness = 1;

// OSD 颜色
cv::Scalar osdColor = cv::Scalar(0, 255, 0);

static std::string Convert(double num)
{
    std::ostringstream oss;
    if (abs(num / 10) >= 10) {
        oss << std::setprecision(5) << num;
    } else if ((abs(num / 10) < 10) && (abs(num / 10) >= 1)) {
        oss << std::setprecision(4) << num;
    } else if (abs(num) < 1 && abs(num) > 0) {
        oss << std::setprecision(2) << num;
    } else {
        oss << std::setprecision(3) << num;
    }

    std::string str(oss.str());
    return str;
}

// 绘制脱靶量
void PaintTrackerMissDistance(cv::Mat &frame0)
{
    int fHeight = frame0.rows;
    int fWidth = frame0.cols;
    int xStart = (fWidth / 3) * 2;
    int yStart = fHeight / 12;
    cv::putText(frame0, "Miss X " + Convert(stSysStatus.trackMissDistance[0]), cv::Point(xStart, yStart), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, osdColor, fontThickness, cv::LINE_AA);
    cv::putText(frame0, "Miss Y " + Convert(stSysStatus.trackMissDistance[1]), cv::Point(xStart, yStart + 30), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, osdColor, fontThickness, cv::LINE_AA);
}

// 绘制画面横滚角度数轴，画面从左到右对应x由小到大
void PaintRollAngleAxis(cv::Mat &frame0, double currRollAngle)
{
    int fHeight = frame0.rows;
    int fWidth = frame0.cols;
    int xStart = fWidth / 2;
    int yStart = fHeight / 12;
    const int interval = 10;
    int curCeil = ceil(currRollAngle);
    int curFloor = floor(currRollAngle);
    uint32_t distance = abs(curCeil % 5);
    int x = xStart - 15 * interval + ((double)curCeil - currRollAngle) * interval;
    
    // 绘制当前横滚角度
    std::string currStr = Convert(currRollAngle);
    cv::line(frame0, cv::Point(xStart, yStart + 10), cv::Point(xStart, yStart + 30), osdColor, fontThickness, cv::LINE_AA);

    int xCurrRollAngle = xStart - 10;
    int leftBigScalePaintSize = 0;
    if (currRollAngle < -15) {
        xCurrRollAngle = xStart - 20;
        leftBigScalePaintSize = 10;
    }
    cv::putText(frame0, currStr, cv::Point(xCurrRollAngle, yStart + 60), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, osdColor, fontThickness, cv::LINE_AA);
    
    // 计算左侧和右侧小刻度值，以及开始大刻度具体数值
    int leftSmallScale = 0;
    int rightSmallScale = 0;
    int tempPos = currRollAngle - 15;
    if (curCeil == curFloor && curCeil % 5 == 0) { // 5的倍数，类似0、5、10等
        tempPos = curCeil - 15;
        leftSmallScale = 0;
        rightSmallScale = 0;
    } else if (curCeil - curFloor == 1 && (curCeil % 5) - (curFloor % 5) == 1) { // 类似-139.5、0.5、1.5等等
        int temp = currRollAngle / 5;
        int tempAngle = temp * 5;
        tempPos = ((curCeil < 0) && (curCeil % 5 != 0)) ? (tempAngle - 15) : (tempAngle - 10);
        leftSmallScale = (curCeil < 0) ? distance : 5 - distance;
        rightSmallScale = 5 - leftSmallScale;
        if (distance == 0) { // 当前位置向上取整为0的
            tempPos -= 5;
            leftSmallScale = 0;
            rightSmallScale = 0;
        }
    } else if (curCeil - curFloor == 1 && (curCeil / 5) - (curFloor / 5) == 1) { // 类似-0.5、4.5等等向上取整为5的倍数的
        if (curCeil < 0 && curFloor < 0) { // 当前角度小于0，向上向下取整均小于0的
            int temp = curCeil / 5;
            int tempAngle = temp * 5;
            tempPos = tempAngle - 15;
            leftSmallScale = (curCeil < 0) ? distance : 5 - distance;
            rightSmallScale = 5 - leftSmallScale;
        } else {
            tempPos = curCeil - 15;
            leftSmallScale = 0;
            rightSmallScale = 0;
        }
    } else {
        int temp = currRollAngle / 5;
        int tempAngle = temp * 5;
        tempPos = tempAngle - 15;
        leftSmallScale = (curCeil < 0) ? distance : 5 - distance;
        rightSmallScale = 5 - leftSmallScale;
    }

    // 左侧小刻度绘制
    for (int i = 0; i < leftSmallScale; i++) {
        cv::line(frame0, cv::Point(x, yStart - 5), cv::Point(x, yStart - 12), cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
        x += interval;
    }
    
    // 绘制从第一个大刻度起的连续5段刻度
    for (int i = 0; i < 4; i++) {
        cv::line(frame0, cv::Point(x, yStart), cv::Point(x, yStart - 20), cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
        cv::putText(frame0, Convert(tempPos), cv::Point(x - 10 - leftBigScalePaintSize, yStart - 35), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
        tempPos += 5;
        x += interval;
        for (int j = 0; j < 4; j++) {
            cv::line(frame0, cv::Point(x, yStart - 5), cv::Point(x, yStart - 12), cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
            x += interval;
        }
    }

    // 接着上面绘制一个大刻度
    cv::line(frame0, cv::Point(x, yStart), cv::Point(x, yStart - 20), cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
    cv::putText(frame0, Convert(tempPos), cv::Point(x - 10 - leftBigScalePaintSize, yStart - 35), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);

    // 当前刻度若为5的倍数，则再绘制两段刻度退出，最后一个刻度是大刻度 
    if (curCeil == curFloor && curCeil % 5 == 0) {
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 4; j++) {
                x += interval;
                cv::line(frame0, cv::Point(x, yStart - 5), cv::Point(x, yStart - 12), cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
            }
            x += interval;
            tempPos += 5;
            cv::line(frame0, cv::Point(x, yStart), cv::Point(x, yStart - 20), cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
            cv::putText(frame0, Convert(tempPos), cv::Point(x - 10 - leftBigScalePaintSize, yStart - 35), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
        }

        return;
    }

    // 当前刻度若向上取整为5的倍数，则接着需要绘制两段刻度
    int r = (rightSmallScale == 0) ? 2 : 1;
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < 4; j++) {
            x += interval;
            cv::line(frame0, cv::Point(x, yStart - 5), cv::Point(x, yStart - 12), cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
        }
        x += interval;
        tempPos += 5;
        cv::line(frame0, cv::Point(x, yStart), cv::Point(x, yStart - 20), cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
        cv::putText(frame0, Convert(tempPos), cv::Point(x - 10 - leftBigScalePaintSize, yStart - 35), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
    }
    
    // 绘制最右侧小刻度
    for (int j = 0; j < rightSmallScale; j++) {
        x += interval;
        cv::line(frame0, cv::Point(x, yStart - 5), cv::Point(x, yStart - 12), cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
    }
}

std::string AddTabBeforeNum(int num)
{
    std::string str = Convert(num);
    if (num >= 0 && num < 10) {
        str = "    " + Convert(num);
    } else if (num >= 10 && num < 100) {
        str = "   " + Convert(num);
    }  else if (num >= 100) {
        str = "  " + Convert(num);
    } else if (num < 0 && num > -10) {
        str = "  " + Convert(num);
    } else if (num <= -10 && num > -100) {
        str = " " + Convert(num);
    }
    return str;
}

// 绘制俯仰角度刻度数轴，画面从上到下对应y由小到大
void PaintPitchAngleAxis(cv::Mat &frame0, double currPitchAngle)
{
    int fHeight = frame0.rows;
    int fWidth = frame0.cols;
    int xStart = fWidth / 18;
    int yStart = fHeight / 2;
    const int interval = 10;
    
    int curCeil = ceil(currPitchAngle);
    int curFloor = floor(currPitchAngle);
    int y = yStart - 10 * interval - interval - ((double)curCeil - currPitchAngle) * interval;
    uint32_t distance = abs(curCeil % 5);
    
    // 绘制当前俯仰角度
    std::string currStr = Convert(currPitchAngle);
    cv::line(frame0, cv::Point(xStart + 5, yStart), cv::Point(xStart + 20, yStart), cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
    cv::putText(frame0, currStr, cv::Point(xStart + 25, yStart + 10), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);

    int upSmallScale = (curCeil < 0) ? (5 - distance) : distance;
    // 绘制最上侧小刻度
    for (int i = 0; i < upSmallScale; i++) {
        y += interval;
        cv::line(frame0, cv::Point(xStart - 7, y), cv::Point(xStart - 15, y), cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
    }

    int temp = currPitchAngle / 5;
    int tempAngle = temp * 5;
    int tempPos = tempAngle + 10;
    if (currPitchAngle < -1) {
        tempPos = tempAngle + 5;
    }

    if ((currPitchAngle > 0) && (curCeil % 5 == 0)) {
        tempPos += 5;
    }
    
    // 从上到下连续绘制3段刻度
    for (int i = -2; i <= 0; i++) {
        y += interval;
        cv::line(frame0, cv::Point(xStart, y), cv::Point(xStart - 20, y), cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
        //std::string tempStr = (tempPos == 5) ? ("   " + Convert(tempPos)) : Convert(tempPos);
        cv::putText(frame0, AddTabBeforeNum(tempPos), cv::Point(xStart - 90, y), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
        tempPos -= 5;
        for (int j = 0; j < 4; j++) {
            y += interval;
            cv::line(frame0, cv::Point(xStart - 7, y), cv::Point(xStart - 15, y), cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
        }
    }

    // 接着绘制一个大刻度
    y += interval;
    cv::line(frame0, cv::Point(xStart, y), cv::Point(xStart - 20, y), cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
    cv::putText(frame0, AddTabBeforeNum(tempPos), cv::Point(xStart - 90, y), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
    
    // 如果是5的倍数，则再绘制一段刻度退出，最下面一个刻度是大刻度
    if (distance == 0) {
        for (int j = 0; j < 4; j++) {
            y += interval;
            cv::line(frame0, cv::Point(xStart - 7, y), cv::Point(xStart - 15, y), cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
        }
        y += interval;
        tempPos -= 5;
        cv::line(frame0, cv::Point(xStart, y), cv::Point(xStart - 20, y), cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
        cv::putText(frame0, AddTabBeforeNum(tempPos), cv::Point(xStart - 90, y), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
        
        return;
    }

    // 绘制最下侧小刻度
    for (int j = 0; j < (5 - upSmallScale); j++) {
        y += interval;
        cv::line(frame0, cv::Point(xStart - 7, y), cv::Point(xStart - 15, y), cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
    }
}

// 绘制中心十字
void PaintCrossPattern(cv::Mat &frame0, float currRollAngle, float currPitchAngle)
{
    int fHeight = frame0.rows;
    int fWidth = frame0.cols;
    int x = fWidth / 2;
    int y = fHeight / 2;
    int lineLen = fHeight / 12;

    // 绘制x线
    cv::line(frame0, cv::Point(x - lineLen, y), cv::Point(x - lineLen / 4, y), cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    cv::line(frame0, cv::Point(x + lineLen, y), cv::Point(x + lineLen / 4, y), cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

    // 绘制中心点
    cv::circle(frame0, cv::Point(x, y), 2, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

    // 绘制y线
    cv::line(frame0, cv::Point(x, y - lineLen), cv::Point(x, y - lineLen / 4), cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    cv::line(frame0, cv::Point(x, y + lineLen), cv::Point(x, y + lineLen / 4), cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
}

static void ConvertLatitudeAndLongitudeUnits(const double input, int *degrees, int *minutes, double *seconds)
{
    *degrees = floor(input);
    *minutes = floor((input - *degrees) * 60);
    *seconds = (input - *degrees) * 3600 - *minutes * 60;
}

static std::string ConvertDegreesNum2Str(double num, const char type)
{
    std::ostringstream oss;
    if (type == 'd') {
        oss << num;
    } else if (type == 'm') {
        if (abs(num) < 10) {
            oss <<std::setw(2)<<std::setfill('0')<<num;
        } else {
            oss << num;
        }
    } else {
        if (num == 0) {
            oss <<std::setw(1)<<std::setfill('0')<<num;
            oss<<setiosflags(ios::fixed)<<std::setprecision(2)<<num;
        } else if (ceil(num) == floor(num) && num != 0 && num < 10) {
            char strNum[64];
            sprintf(strNum, "%d%.2f\n", (int)num / 10, num);
            oss<<strNum;
        } else {
            oss<<setiosflags(ios::fixed)<<std::setprecision(2)<<num;
        }
        
    }

    std::string str(oss.str());
    return str;
}

std::string ConvertMetersNum2Str(double num)
{
    std::ostringstream oss;
    oss<<setiosflags(ios::fixed)<<std::setprecision(3)<<num;
    std::string str(oss.str());
    return str;
}

// 绘制度分秒 1°00′00.00″
static void PaintDegMinSec(cv::Mat &frame0, int x, int y, const double inputAngle, bool isLongitude)
{
    if ((isLongitude && (abs(inputAngle) >= 180)) || (!isLongitude && ((inputAngle) >= 90))) {
        return;
    }
    int degrees = 0;
    int minutes = 0;
    double seconds = 0;

    if (isLongitude) {
        if (inputAngle >= 0) {
            cv::putText(frame0, "E ", cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
        } else {
            cv::putText(frame0, "W ", cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
        }
    } else {
        if (inputAngle >= 0) {
            cv::putText(frame0, "N ", cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
        } else {
            cv::putText(frame0, "S ", cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
        }
    }
    ConvertLatitudeAndLongitudeUnits(abs(inputAngle), &degrees, &minutes, &seconds);
    std::string currDeg = ConvertDegreesNum2Str(degrees, 'd') + "deg " + ConvertDegreesNum2Str(minutes, 'm') + "'" + ConvertDegreesNum2Str(seconds, 's') + "''";
    cv::putText(frame0, currDeg, cv::Point(x + 20, y), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
}

// 绘制地理坐标
void PaintCoordinate(cv::Mat &frame0)
{
    int fHeight = frame0.rows;
    int fWidth = frame0.cols;
    int x = (fWidth / 8) * 7;
    int y = (fHeight / 4) * 3;
    stSysStatus.isTSeriesDevice = false;
    stSysStatus.osdSet2Ctrl.enTAGGPSShow = true;
    if (!stSysStatus.isTSeriesDevice && stSysStatus.osdSet2Ctrl.enTAGGPSShow) {
        // 目标位置坐标
        cv::putText(frame0, "TAG", cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
        PaintDegMinSec(frame0, x, y + 30, stSysStatus.TAGCoordinate.longitude, true);
        PaintDegMinSec(frame0, x, y + 60, stSysStatus.TAGCoordinate.latitude, false);
        cv::putText(frame0, "ALT", cv::Point(x, y + 90), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
        std::string tagAlt = ConvertMetersNum2Str(stSysStatus.TAGCoordinate.altitude) + "m";
        cv::putText(frame0, tagAlt, cv::Point(x + 45, y + 90), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
    }
    stSysStatus.osdSet1Ctrl.enACFTGPS1Show = true;
    if (stSysStatus.osdSet1Ctrl.enACFTGPS1Show) {
        // 载机位置坐标
        cv::putText(frame0, "ACFT", cv::Point(x, y + 140), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
        PaintDegMinSec(frame0, x, y + 170, stSysStatus.ACFTCoordinate.longitude, true);
        PaintDegMinSec(frame0, x, y + 200, stSysStatus.ACFTCoordinate.latitude, false);
        cv::putText(frame0, "ALT", cv::Point(x, y + 230), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
        std::string acftAlt = ConvertMetersNum2Str(stSysStatus.ACFTCoordinate.altitude) + "m";
        cv::putText(frame0, acftAlt, cv::Point(x + 45 , y + 230), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
    }
}

std::string ConvertTimesNum2Str(double num)
{
    std::ostringstream oss;
    oss<<setiosflags(ios::fixed)<<std::setprecision(1)<<num;
    std::string str(oss.str());
    return str;
}

std::string currTime;

// 绘制界面上的其他参数
void PaintViewPara(cv::Mat &frame0)
{
    int fHeight = frame0.rows;
    int fWidth = frame0.cols;
    int x = fWidth / 23;
    int y = fHeight / 8;

    stSysStatus.osdSet1Ctrl.enTimeShow = true;
    if (stSysStatus.osdSet1Ctrl.enTimeShow) {
        std::time_t curr = std::chrono::system_clock::to_time_t (std::chrono::system_clock::now());
        std::stringstream ss;
        ss << std::put_time(std::localtime(&curr), "%Y/%m/%d");
        std::string currDate(ss.str());
        cv::putText(frame0, currDate, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
        std::stringstream ss1;
        ss1 << std::put_time(std::localtime(&curr), "%H:%M:%S");
        std::string currTimeTemp(ss1.str());
        stSysStatus.isTSeriesDevice = false;
        if ((stSysStatus.isTSeriesDevice && stSysStatus.osdSet2Ctrl.enIRShow) || !stSysStatus.isTSeriesDevice) { // enIRShow T系列该值作为时间输入使能位
            currTime = currTimeTemp;
        }

        cv::putText(frame0, currTime, cv::Point(x, y + 30), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
    }

    stSysStatus.osdSet1Ctrl.enEOFieldOfViewOrMultiplyShow = true;
    if (stSysStatus.osdSet1Ctrl.enEOFieldOfViewOrMultiplyShow) {
        uint16_t visibleLightOpticalZoomFactor = 0;
        memcpy(&visibleLightOpticalZoomFactor, &stT1F1B1D1Cfg.d1Config.currSensorOpticsAmplificationFactor, 2);
        if (stSysStatus.isTSeriesDevice && stSysStatus.osdSet2Ctrl.enTAGGPSShow) { // T系列此项值相当于EO和FOV的输入，如果未使能，则对EO和FOV的值不予改变
            stSysStatus.eoValue = (visibleLightOpticalZoomFactor * 0.1) * (stT1F1B1D1Cfg.d1Config.visibleLightElectronicMagnification + 1);
            stSysStatus.fovValue = (double)stT1F1B1D1Cfg.d1Config.currSensorHoriFieldOfViewAngle * 0.01;
        }
        
        if (!stSysStatus.isTSeriesDevice) { // A系列此值为TAGGPS展示，不对EO和FOV产生影响
            stSysStatus.eoValue = (visibleLightOpticalZoomFactor * 0.1) * (stT1F1B1D1Cfg.d1Config.visibleLightElectronicMagnification + 1);
            cv::putText(frame0, "EO", cv::Point(x, y + 60), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
            cv::putText(frame0, ConvertTimesNum2Str(stSysStatus.eoValue) + "x", cv::Point(x + 40, y + 60), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
            
            stSysStatus.fovValue = (double)stT1F1B1D1Cfg.d1Config.currSensorHoriFieldOfViewAngle * 0.01;
            cv::putText(frame0, "    FOV " + ConvertTimesNum2Str(stSysStatus.fovValue) + "deg", cv::Point(x + 75, y + 60), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
        }
        
        // if (stSysStatus.isTSeriesDevice && stSysStatus.osdSet2Ctrl.enTFShow) {
        //     cv::putText(frame0, "FOV  " + ConvertTimesNum2Str(stSysStatus.fovValue) + "deg", cv::Point(x, y + 60), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
        // } else {
        //     cv::putText(frame0, "EO  " + ConvertTimesNum2Str(stSysStatus.eoValue) + "x", cv::Point(x, y + 60), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
        // }
    }

    stSysStatus.osdSet2Ctrl.enIRShow = true;
    if (!stSysStatus.isTSeriesDevice && stSysStatus.osdSet2Ctrl.enIRShow) {
        double irValue = (double)stT1F1B1D1Cfg.d1Config.thermalImagingElectronicMagnification + 1;
        cv::putText(frame0, "IR  " + ConvertTimesNum2Str(irValue) + "x", cv::Point(x, y + 90), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
    }
    stSysStatus.osdSet2Ctrl.enLRFShow = true;
    if ((!stSysStatus.isTSeriesDevice && stSysStatus.osdSet2Ctrl.enLRFShow) || (stSysStatus.isTSeriesDevice && stSysStatus.osdSet1Ctrl.enACFTGPS1Show)) {
        cv::putText(frame0, "LRF  " + ConvertTimesNum2Str(stSysStatus.lrfValue) + "m", cv::Point(x, y + 120), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
    }

    stSysStatus.osdSet2Ctrl.enGPSIsMGRS = false;
    // T系列的SD卡状态跟随OSD开关使能而展示，A系列的有TF状态开关，需要其使能才展示，enGPSIsMGRS在A系列则相当于TF状态开关
    if (!stSysStatus.isTSeriesDevice && !stSysStatus.osdSet2Ctrl.enGPSIsMGRS) {
        return;
    }
    struct statfs diskInfo;
    statfs("/", &diskInfo);
    unsigned long long totalBlocks = diskInfo.f_bsize;  
    unsigned long long totalSize = totalBlocks * diskInfo.f_blocks;  
    uint32_t mbTotalsize = totalSize>>20;  
    unsigned long long freeDisk = diskInfo.f_bavail*totalBlocks;  
    uint32_t mbFreedisk = freeDisk>>20;
    uint32_t recordVideoTime = mbFreedisk / 1.5;
    std::string sdCardState = "No SD Card";
    if (recordVideoTime > 0) {
        sdCardState = "SD Card remain " + Convert(recordVideoTime) + " MB";
    }
    cv::putText(frame0, sdCardState, cv::Point(x, y + 160), cv::FONT_HERSHEY_SIMPLEX, stSysStatus.osdFontSize, cv::Scalar(0, 255, 0), fontThickness, cv::LINE_AA);
}