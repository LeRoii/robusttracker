
/****************************************************************************************
 * 文 件 名 : 	VideoPro_Common.c															*
 * 项目名称 :   videoPro                                                            *
 * 模 块 名 :   通用处理模块                                          					*
 * 功    能 :   通用处理函数文件                                   							*
 * 操作系统 : 	LINUX																	*
 * 修改记录 :	初始版本																*
 * 版    本 :	Rev 0.0.1																*
 *--------------------------------------------------------------------------------------*
 * 设    计 ：	walker      '2024-06-26                                                 	*
 * 编    码 ： 	walker      '2024-06-26                                          		*
 * 修 	 改 ： 																			*
 ****************************************************************************************/
#include <stddef.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <sys/time.h>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/freetype.hpp>
#include <iostream>


#include "VideoPro_Common.h"


VideoPro_Common::VideoPro_Common(/* args */)
{

}
VideoPro_Common::~VideoPro_Common()
{

}
/****************************************************************************************
 * 函 数 名 ：    gettime()                                                     
 * 功    能 ：    获取当前系统时间                                                      
 * 输入参数 ：                                                                                  
 * 输出参数 ：                                                                        
 * 返 回 值 ：   当前时间（单位为微秒）
 * 修改记录 ：                                                               
 * 版    本 ：   v 0.0.1                                                            
 *--------------------------------------------------------------------------------------*
 * 设    计 ：    walker      '2024-4-24                                                *
 * 编    码 ：    walker      '2024-4-24                                               *
 * 修    改 ：    walker      '2024-4-24                                                 *
 ***************************************************************************************/
unsigned long long  VideoPro_Common::VidePro_ComGetTime()
{
	struct timespec current_time;
	struct timeval t;
	memset(&current_time,0,sizeof(struct timespec));
	memset(&t,0,sizeof(struct timeval));
	if(0 == clock_gettime(CLOCK_MONOTONIC,&current_time))
	{
		return current_time.tv_sec*1000000ULL+current_time.tv_nsec/1000;
	}
	else
	{
		gettimeofday(&t, 0);
		return t.tv_sec * 1000000ULL + t.tv_usec;
	}
}

double VideoPro_Common::calculateContrastRatio(const cv::Mat& image) 
{
    // cv::Mat grayImage;
    // cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
 
    // cv::Mat grad;
    // cv::Sobel(grayImage, grad, CV_64F, 1, 1);
 
    // double totalEntropy = 0.0;
    // int totalPixels = 0;
 
    // for (int y = 0; y < grad.rows; ++y) {
    //     for (int x = 0; x < grad.cols; ++x) {
    //         double gradMag = std::abs(grad.at<double>(y, x));
    //         if (gradMag > 0.0) {
    //             double entropy = -std::log(gradMag) / std::log(255.0);
    //             totalEntropy += entropy;
    //             totalPixels++;
    //         }
    //     }
    // }
    // return totalPixels > 0 ? totalEntropy / totalPixels : 0.0;



	 // 确保图像是灰度图
    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image;
    }
 
    // 计算最小值和最大值
    double minVal, maxVal;
    cv::minMaxLoc(gray, &minVal, &maxVal);
 
    // 计算熵
    double eme = 0.0;
    int height = gray.rows;
    int width = gray.cols;
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            double p = (double)gray.at<uchar>(i, j) / maxVal;
            eme -= p * std::log2(p);
        }
    }
 
    return eme;
}