#ifndef _SDIREADER_H_
#define _SDIREADER_H_

#include <opencv2/opencv.hpp>

void Sdireader_Init(const std::string cfgpath);
void Sdireader_GetFrame(cv::Mat &frame0, cv::Mat &frame1);


#endif