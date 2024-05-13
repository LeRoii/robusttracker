/*
 * @Author: WJM
 * @Date: 2024-03-11 11:25:59
 * @LastEditors: WJM
 * @LastEditTime: 2024-03-11 14:19:54
 * @Description:
 * @FilePath: /rknn_yolov8_detector_v2/src/idetector.cpp
 * @custom_string: http://www.aiar.xjtu.edu.cn/
 */
#include "idetector.h"

CDetector::CDetector(char *engine_path, int thread_num, int obj_class_num, float nms_threshold, float conf_threshold)
{
    this->modelPath = (char *)malloc(strlen(engine_path) + 1);
    strcpy(modelPath, engine_path);

    this->threadNum = thread_num;
    this->objClassNum = obj_class_num;
    framesIndex = 0;
    this->nmsThreshold = nms_threshold;
    this->confThreshold = conf_threshold;
}

CDetector::~CDetector()
{
    while (!futs.empty())
    {
        if (futs.front().get())
            break;
        futs.pop();
    }

    for (int i = 0; i < threadNum; i++)
        delete modelInferences[i];
    std::cout << "destroy idetector!" << std::endl;

    if (modelPath)
    {
        free(modelPath);
    }

    std::cout << "destroy idetector!" << std::endl;
}

void CDetector::Init()
{
    cv::Mat img = cv::Mat::zeros(1280, 720, CV_8UC3);
    for (int i = 0; i < threadNum; i++)
    {
        CModelInference *modelInference = new CModelInference(modelPath, i % threadNum, objClassNum, nmsThreshold, confThreshold);
        modelInferences.push_back(modelInference);
        futs.push(threadpool.submit(&CModelInference::InferenceModel, modelInferences[i], img));
    }
    gettimeofday(&time, nullptr);
    tmpTime, lopTime = time.tv_sec * 1000 + time.tv_usec / 1000;
}

void CDetector::ImgInference(cv::Mat img, bbox_t *boxs, int &boxs_count)
{
    if (futs.front().get() != 0)
        return;
    futs.pop();

    memcpy(boxs, modelInferences[framesIndex % threadNum]->detectResultGroup.results, sizeof(bbox_t) * OBJ_NUMB_MAX_SIZE);
    boxs_count = modelInferences[framesIndex % threadNum]->detectResultGroup.count;
    futs.push(threadpool.submit(&CModelInference::InferenceModel, modelInferences[framesIndex++ % threadNum], img.clone()));

    if (framesIndex % 60 == 0)
    {
        gettimeofday(&time, nullptr);
        tmpTime = time.tv_sec * 1000 + time.tv_usec / 1000;
        printf("60帧平均帧率:\t%f帧\n", 60000.0 / (float)(tmpTime - lopTime));
        lopTime = tmpTime;
    }
}
