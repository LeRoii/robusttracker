#include "painter.h"
#include <iostream>
#include <sstream>
#include <cmath>
#include<arpa/inet.h>
#include "serial.h"
#include "common.h"
#include <chrono>

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
    cv::line(frame0, cv::Point(xStart, yStart + 10), cv::Point(xStart, yStart + 30), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    cv::putText(frame0, currStr, cv::Point(xStart - 10, yStart + 45), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    
    // 计算左侧和右侧小刻度值，以及开始大刻度具体数值
    int leftSmallScale = 0;
    int rightSmallScale = 0;
    int tempPos = currRollAngle - 15;
    if (curCeil == curFloor && curCeil % 5 == 0) { // 5的倍数，类似0、5、10等
        tempPos = curCeil - 15;
        leftSmallScale = 0;
        rightSmallScale = 0;
        printf("111 tempPos=%d\n", tempPos);
    } else if (curCeil - curFloor == 1 && (curCeil % 5) - (curFloor % 5) == 1) { // 类似-139.5、0.5、1.5等等
        int temp = currRollAngle / 5;
        int tempAngle = temp * 5;
        tempPos = tempAngle - 10;
        leftSmallScale = (curCeil < 0) ? distance : 5 - distance;
        rightSmallScale = 5 - leftSmallScale;
        printf("222 tempPos=%d\n", tempPos);
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
            printf("333 tempPos=%d\n", tempPos);
        } else {
            tempPos = curCeil - 15;
            leftSmallScale = 0;
            rightSmallScale = 0;
        }
        printf("444 tempPos=%d\n", tempPos);
    } else {
        int temp = currRollAngle / 5;
        int tempAngle = temp * 5;
        tempPos = tempAngle - 15;
        leftSmallScale = (curCeil < 0) ? distance : 5 - distance;
        rightSmallScale = 5 - leftSmallScale;
    }

    // 左侧小刻度绘制
    for (int i = 0; i < leftSmallScale; i++) {
        cv::line(frame0, cv::Point(x, yStart - 5), cv::Point(x, yStart - 12), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        x += interval;
    }
    
    printf("tempPos=%d\n", tempPos);
    // 绘制从第一个大刻度起的连续5段刻度
    for (int i = 0; i < 4; i++) {
        cv::line(frame0, cv::Point(x, yStart), cv::Point(x, yStart - 20), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        cv::putText(frame0, Convert(tempPos), cv::Point(x - 10, yStart - 35), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        tempPos += 5;
        x += interval;
        for (int j = 0; j < 4; j++) {
            cv::line(frame0, cv::Point(x, yStart - 5), cv::Point(x, yStart - 12), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
            x += interval;
        }
    }

    // 接着上面绘制一个大刻度
    cv::line(frame0, cv::Point(x, yStart), cv::Point(x, yStart - 20), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    cv::putText(frame0, Convert(tempPos), cv::Point(x - 10, yStart - 35), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

    // 当前刻度若为5的倍数，则再绘制两段刻度退出，最后一个刻度是大刻度 
    if (curCeil == curFloor && curCeil % 5 == 0) {
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 4; j++) {
                x += interval;
                cv::line(frame0, cv::Point(x, yStart - 5), cv::Point(x, yStart - 12), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
            }
            x += interval;
            tempPos += 5;
            cv::line(frame0, cv::Point(x, yStart), cv::Point(x, yStart - 20), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
            cv::putText(frame0, Convert(tempPos), cv::Point(x - 10, yStart - 35), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        }

        return;
    }

    // 当前刻度若向上取整为5的倍数，则接着需要绘制两段刻度
    int r = (rightSmallScale == 0) ? 2 : 1;
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < 4; j++) {
            x += interval;
            cv::line(frame0, cv::Point(x, yStart - 5), cv::Point(x, yStart - 12), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        }
        x += interval;
        tempPos += 5;
        cv::line(frame0, cv::Point(x, yStart), cv::Point(x, yStart - 20), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        cv::putText(frame0, Convert(tempPos), cv::Point(x - 10, yStart - 35), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    }
    
    // 绘制最右侧小刻度
    for (int j = 0; j < rightSmallScale; j++) {
        x += interval;
        cv::line(frame0, cv::Point(x, yStart - 5), cv::Point(x, yStart - 12), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
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
    cv::line(frame0, cv::Point(xStart + 5, yStart), cv::Point(xStart + 20, yStart), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    cv::putText(frame0, currStr, cv::Point(xStart + 30, yStart), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

    int upSmallScale = (curCeil < 0) ? (5 - distance) : distance;
    // 绘制最上侧小刻度
    for (int i = 0; i < upSmallScale; i++) {
        y += interval;
        cv::line(frame0, cv::Point(xStart - 7, y), cv::Point(xStart - 15, y), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
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
        cv::line(frame0, cv::Point(xStart, y), cv::Point(xStart - 20, y), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        //std::string tempStr = (tempPos == 5) ? ("   " + Convert(tempPos)) : Convert(tempPos);
        cv::putText(frame0, AddTabBeforeNum(tempPos), cv::Point(xStart - 65, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        tempPos -= 5;
        for (int j = 0; j < 4; j++) {
            y += interval;
            cv::line(frame0, cv::Point(xStart - 7, y), cv::Point(xStart - 15, y), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        }
    }

    // 接着绘制一个大刻度
    y += interval;
    cv::line(frame0, cv::Point(xStart, y), cv::Point(xStart - 20, y), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    cv::putText(frame0, AddTabBeforeNum(tempPos), cv::Point(xStart - 65, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    
    // 如果是5的倍数，则再绘制一段刻度退出，最下面一个刻度是大刻度
    if (distance == 0) {
        for (int j = 0; j < 4; j++) {
            y += interval;
            cv::line(frame0, cv::Point(xStart - 7, y), cv::Point(xStart - 15, y), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        }
        y += interval;
        tempPos -= 5;
        cv::line(frame0, cv::Point(xStart, y), cv::Point(xStart - 20, y), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        cv::putText(frame0, AddTabBeforeNum(tempPos), cv::Point(xStart - 65, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        
        return;
    }

    // 绘制最下侧小刻度
    for (int j = 0; j < (5 - upSmallScale); j++) {
        y += interval;
        cv::line(frame0, cv::Point(xStart - 7, y), cv::Point(xStart - 15, y), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    }
}

void PaintCrossPattern(cv::Mat &frame0, float currRollAngle, float currPitchAngle)
{
    int fHeight = frame0.rows;
    int fWidth = frame0.cols;
    int x = fWidth / 2;
    int y = fHeight / 2;
    int lineLen = fHeight / 12;

    // 绘制x线
    cv::line(frame0, cv::Point(x - lineLen, y), cv::Point(x - lineLen / 4, y), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    cv::line(frame0, cv::Point(x + lineLen, y), cv::Point(x + lineLen / 4, y), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

    // 绘制中心点
    cv::circle(frame0, cv::Point(x, y), 2, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);

    // 绘制y线
    cv::line(frame0, cv::Point(x, y - lineLen), cv::Point(x, y - lineLen / 4), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    cv::line(frame0, cv::Point(x, y + lineLen), cv::Point(x, y + lineLen / 4), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
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
            cv::putText(frame0, "E", cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        } else {
            cv::putText(frame0, "W", cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        }
    } else {
        if (inputAngle >= 0) {
            cv::putText(frame0, "N", cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        } else {
            cv::putText(frame0, "S", cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        }
    }
    ConvertLatitudeAndLongitudeUnits(abs(inputAngle), &degrees, &minutes, &seconds);
    cv::putText(frame0,
        ConvertDegreesNum2Str(degrees, 'd') + "deg" + ConvertDegreesNum2Str(minutes, 'm') + "'" + ConvertDegreesNum2Str(seconds, 's') + "''",
        cv::Point(x + 15, y), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
}

void PaintCoordinate(cv::Mat &frame0)
{
    int fHeight = frame0.rows;
    int fWidth = frame0.cols;
    int x = (fWidth / 8) * 7;
    int y = (fHeight / 4) * 3;

    // 目标位置坐标
    cv::putText(frame0, "TAG", cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    PaintDegMinSec(frame0, x, y + 20, stSysStatus.TAGCoordinate.longitude, true);
    PaintDegMinSec(frame0, x, y + 40, abs(stSysStatus.TAGCoordinate.latitude), false);
    cv::putText(frame0, "ALT", cv::Point(x, y + 60), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    cv::putText(frame0, ConvertMetersNum2Str(stSysStatus.TAGCoordinate.altitude) + "m", cv::Point(x + 40, y + 60),
        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

    // 载机位置坐标
    cv::putText(frame0, "ACFT", cv::Point(x, y + 100), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    PaintDegMinSec(frame0, x, y + 120, abs(stSysStatus.ACFTCoordinate.longitude), true);
    PaintDegMinSec(frame0, x, y + 140, abs(stSysStatus.ACFTCoordinate.latitude), false);
    cv::putText(frame0, "ALT", cv::Point(x, y + 160), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    cv::putText(frame0, ConvertMetersNum2Str(stSysStatus.ACFTCoordinate.altitude) + "m", cv::Point(x + 40 , y + 160),
        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
}

std::string ConvertTimesNum2Str(double num)
{
    std::ostringstream oss;
    oss<<setiosflags(ios::fixed)<<std::setprecision(1)<<num;
    std::string str(oss.str());
    return str;
}

void PaintViewPara(cv::Mat &frame0)
{
    int fHeight = frame0.rows;
    int fWidth = frame0.cols;
    int x = fWidth / 23;
    int y = fHeight / 8;

    std::time_t curr = std::chrono::system_clock::to_time_t (std::chrono::system_clock::now());
    std::stringstream ss;
    ss << std::put_time(std::localtime(&curr), "%Y/%m/%d");
    std::string currDate(ss.str());
    cv::putText(frame0, currDate, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    std::stringstream ss1;
    ss1 << std::put_time(std::localtime(&curr), "%H:%M:%S");
    std::string currTime(ss1.str());
    cv::putText(frame0, currTime, cv::Point(x, y + 20), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

    printf("---------------PaintViewPara C2 opCmd1=%#x\n", stA2C2E2Cfg.c2Config.opCmd1);
    if (stA2C2E2Cfg.c2Config.opCmd1 == 0x53) {
        uint16_t visibleLightOpticalZoomFactor = 0;
        printf("---------------PaintViewPara opCmdPara1[%02X]-[%02X]\n", stA2C2E2Cfg.c2Config.opCmdPara1[0], stA2C2E2Cfg.c2Config.opCmdPara1[1]);
        memcpy(&visibleLightOpticalZoomFactor, &stA2C2E2Cfg.c2Config.opCmdPara1, 2);
        visibleLightOpticalZoomFactor = ntohs(visibleLightOpticalZoomFactor);
        printf("---------------visibleLightOpticalZoomFactor=%#x\n", visibleLightOpticalZoomFactor);
        stSysStatus.eoValue = (visibleLightOpticalZoomFactor * 0.1) * (stT1F1B1D1Cfg.d1Config.visibleLightElectronicMagnification + 1);
        printf("---------------eoValue111=%04f\n", stSysStatus.eoValue);
    }

    double fovValue = (double)stT1F1B1D1Cfg.d1Config.currSensorHoriFieldOfViewAngle * 0.01;
    double irValue = (double)stT1F1B1D1Cfg.d1Config.thermalImagingElectronicMagnification + 1;

    cv::putText(frame0, "EO ", cv::Point(x, y + 40), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    cv::putText(frame0, ConvertTimesNum2Str(stSysStatus.eoValue) + "x", cv::Point(x + 25, y + 40), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    cv::putText(frame0, "FOV " + ConvertTimesNum2Str(fovValue) + "deg", cv::Point(x + 75, y + 40), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    cv::putText(frame0, "IR ", cv::Point(x, y + 60), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    cv::putText(frame0, ConvertTimesNum2Str(irValue) + "x", cv::Point(x + 25, y + 60), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    cv::putText(frame0, "LRF ", cv::Point(x, y + 80), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    cv::putText(frame0,  ConvertTimesNum2Str(stSysStatus.lrfValue) + "m", cv::Point(x + 25, y + 80), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
}