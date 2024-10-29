#ifndef VIDEOPRO_ENHANCEMENT_H
#define VIDEOPRO_ENHANCEMENT_H
#include <opencv2/opencv.hpp>
#include <opencv2/freetype.hpp>

/****************************************************************************************
 * 函 数 名 ： VideoPro_EnhancementInit
 * 功    能 ： 图像增强初始化
 ***************************************************************************************/
int VideoPro_Enhancement_Init();

void VideoPro_Enhancement_Pro(cv::Mat &frame);

/****************************************************************************************
 * 函 数 名 ： VideoPro_Enhancement_Destroy
 * 功    能 ： 图像增强句柄销毁
 ***************************************************************************************/
int VideoPro_Enhancement_Destroy();
#endif	// #define VIDEOPRO_ENHANCEMENT_H

