#ifndef PAINTER_H
#define PAINTER_H

#include <opencv2/opencv.hpp>
#include <opencv2/freetype.hpp>

void PaintRollAngleAxis(cv::Mat &frame0, double currRollAngle);
void PaintPitchAngleAxis(cv::Mat &frame0, double currPitchAngle);
void PaintCrossPattern(cv::Mat &frame0, float currRollAngle, float currPitchAngle);
void PaintCoordinate(cv::Mat &frame0);
void PaintViewPara(cv::Mat &frame0);
void PaintTrackerMissDistance(cv::Mat &frame0);
void drawRect(cv::Mat frame, cv::Rect r, cv::Scalar color = cv::Scalar(0,255,255));
void drawLostRect(cv::Mat frame, cv::Rect r);
void PaintWorkStatus(cv::Mat &img, cv::Ptr<cv::freetype::FreeType2> &ft2);
//角度信息
void PaintAngle(cv::Mat &frame0);
//姿态角度信息
void Paint_Arhs(cv::Mat &frame0);
#endif