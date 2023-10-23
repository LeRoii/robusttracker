#include "itracker.h"
#include "kcftracker.hpp"

static KCFTracker* trackerPtr = nullptr;

itracker::itracker()
{
    bool HOG = false;
    bool FIXEDWINDOW = false;
    bool MULTISCALE = true;
    bool SILENT = true;
    bool LAB = false;

    trackerPtr = new KCFTracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
}

itracker::~itracker()
{

}

void itracker::init(const cv::Rect &roi, cv::Mat image)
{
    trackerPtr->init(roi, image);
}

void itracker::update(cv::Mat image)
{
    trackerPtr->update(image);
}

void itracker::reset()
{
    if(trackerPtr != nullptr)
    {
        delete trackerPtr;
    }

    bool HOG = false;
    bool FIXEDWINDOW = false;
    bool MULTISCALE = true;
    bool SILENT = true;
    bool LAB = false;

    trackerPtr = new KCFTracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
}