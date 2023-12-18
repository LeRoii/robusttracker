#include <model_inference.h>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <idetector.h>

idetector::idetector(char *model_path, int thread_num)
{
    this->model_path = (char *)malloc(strlen(model_path) + 1);
    strcpy(this->model_path, model_path);
    this->thread_num = thread_num;
    frames_index = 0;
}

void idetector::init()
{
    cv::Mat img = cv::Mat::zeros(1280, 720, CV_8UC3);
    for (int i = 0; i < thread_num; i++)
    {
        modelInference *det = new modelInference(model_path, i % 3);
        dets.push_back(det);
        dets[i]->init();
        dets[i]->img = img.clone();
        futs.push(pool.submit(&modelInference::detect, &(*dets[i])));
    }
    gettimeofday(&time, nullptr);
    tmpTime, lopTime = time.tv_sec * 1000 + time.tv_usec / 1000;
}

void idetector::process(cv::Mat &img, std::vector<bbox_t> &boxs)
{
    if (futs.front().get() != 0)
        return;
    futs.pop();
    
    // cv::imshow("frame", dets[frames_index % thread_num]->img);
    // cv::waitKey(1);
    boxs.clear();
    boxs = dets[frames_index % thread_num]->boxs;
    dets[frames_index % thread_num]->boxs.clear();
    dets[frames_index % thread_num]->img = img.clone();
    // for (auto i = boxs.begin(); i != boxs.end(); i++)
    // {
    //     rectangle(img, cv::Point(i->x, i->y), cv::Point(i->x + i->w, i->y + i->h), cv::Scalar(255, 0, 0), 2);
    // }

    
    futs.push(pool.submit(&modelInference::detect, &(*dets[frames_index++ % thread_num])));
    if (frames_index % 60 == 0)
    {
        gettimeofday(&time, nullptr);
        tmpTime = time.tv_sec * 1000 + time.tv_usec / 1000;
        printf("60帧平均帧率:\t%f帧\n", 60000.0 / (float)(tmpTime - lopTime));
        lopTime = tmpTime;
    }
}

void idetector::processDebug(cv::Mat &img, std::vector<bbox_t> &boxs)
{
    if (futs.front().get() != 0)
        return;
    futs.pop();
    
    // cv::imshow("frame", dets[frames_index % thread_num]->img);
    // cv::waitKey(1);
    boxs.clear();
    boxs = dets[frames_index % thread_num]->boxs;
    dets[frames_index % thread_num]->boxs.clear();
    dets[frames_index % thread_num]->img = img.clone();
    for (auto i = boxs.begin(); i != boxs.end(); i++)
    {
        rectangle(img, cv::Point(i->x, i->y), cv::Point(i->x + i->w, i->y + i->h), cv::Scalar(255, 0, 0), 2);
    }

    
    futs.push(pool.submit(&modelInference::detect, &(*dets[frames_index++ % thread_num])));
    if (frames_index % 60 == 0)
    {
        gettimeofday(&time, nullptr);
        tmpTime = time.tv_sec * 1000 + time.tv_usec / 1000;
        printf("60帧平均帧率:\t%f帧\n", 60000.0 / (float)(tmpTime - lopTime));
        lopTime = tmpTime;
    }
}

idetector::~idetector()
{
    while (!futs.empty())
    {
        if (futs.front().get())
            break;
        futs.pop();
    }
    for (int i = 0; i < thread_num; i++)
        delete dets[i];
    std::cout << "destroy idetector!" << std::endl;

    if (model_path)
    {
        free(model_path);
    }

    std::cout << "destroy idetector!" << std::endl;
}