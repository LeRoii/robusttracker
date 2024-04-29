#ifndef PAINTER_H
#define PAINTER_H

#include <opencv2/opencv.hpp>

void PaintRollAngleAxis(cv::Mat &frame0, double currRollAngle);
void PaintPitchAngleAxis(cv::Mat &frame0, double currPitchAngle);
void PaintCrossPattern(cv::Mat &frame0, float currRollAngle, float currPitchAngle);
void PaintCoordinate(cv::Mat &frame0);
void PaintViewPara(cv::Mat &frame0);
void PaintTrackerMissDistance(cv::Mat &frame0);
void drawRect(cv::Mat frame, cv::Rect r, cv::Scalar color = cv::Scalar(0,255,255));
void drawLostRect(cv::Mat frame, cv::Rect r);
#endif