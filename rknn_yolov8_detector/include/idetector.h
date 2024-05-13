/*
 * @Author: WJM
 * @Date: 2024-03-11 11:25:51
 * @LastEditors: WJM
 * @LastEditTime: 2024-03-11 14:01:00
 * @Description:
 * @FilePath: /rknn_yolov8_detector_v2/include/idetector.h
 * @custom_string: http://www.aiar.xjtu.edu.cn/
 */
#ifndef _IDETECTOR_H_
#define _IDETECTOR_H_

#include <string>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "model_inference.h"
#include "ThreadPool.hpp"

class CDetector
{
public:
    explicit CDetector(char *engine_path, int thread_num, int obj_class_num, float nms_threshold, float conf_threshold);
    ~CDetector();
    void Init();
    void ImgInference(cv::Mat img, bbox_t *, int &boxs_count);

private:
    // CModelInference *modelInference;
    char *modelPath;
    int framesIndex;
    int threadNum;
    std::vector<CModelInference *> modelInferences;
    std::queue<std::future<int>> futs;
    dpool::ThreadPool threadpool;
    struct timeval time;
    long tmpTime, lopTime;
    int objClassNum;
    float nmsThreshold;
    float confThreshold;
};

#endif