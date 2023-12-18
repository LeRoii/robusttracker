#ifndef _IDETECTOR_H_
#define _IDETECTOR_H_

#include <string>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "model_inference.h"
#include "ThreadPool.hpp"

class idetector
{
public:
    explicit idetector(char *enginepath,int thread_num);
    ~idetector();
    void init();
    // void process(cv::Mat &img);
    void process(cv::Mat &img, std::vector<bbox_t> &boxs);
    void processDebug(cv::Mat &img, std::vector<bbox_t> &boxs);

private:
    modelInference *model_inference;
    char * model_path;
    int frames_index;
    int thread_num;
    std::vector<modelInference *> dets;
    std::queue<std::future<int>> futs;
    dpool::ThreadPool pool;
    struct timeval time;
    long tmpTime, lopTime;


};

#endif