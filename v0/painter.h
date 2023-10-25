#ifndef PAINTER_H
#define PAINTER_H

#include <opencv2/opencv.hpp>

void PaintRollAngleAxis(cv::Mat &frame0, double currRollAngle);
void PaintPitchAngleAxis(cv::Mat &frame0, double currPitchAngle);
void PaintCrossPattern(cv::Mat &frame0, float currRollAngle, float currPitchAngle);
void PaintCoordinate(cv::Mat &frame0);
void PaintViewPara(cv::Mat &frame0);

#endif