
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <vector>
#include <thread>
#include <queue>
#include <future>
#include "ThreadPool.hpp"

#include "RgaUtils.h"
#include "im2d.h"
#include <opencv2/opencv.hpp>
#include "v4l2_capture.hpp"
#include "model_inference.h"
int main(int argc, char **argv)
{
    // 线程池操作
    dpool::ThreadPool pool(3);
    std::vector<modelInference *> dets;
    std::queue<std::future<int>> futs;

    char model_path[256] = "/home/rpdzkj/code/yolov8_rknn_Cplusplus/examples/rknn_yolov8_demo_open/model/RK3588/yolov8n_ZQ.rknn";
    char image_path[256] = "/home/rpdzkj/code/obj_detect_multhread/examples/rknn_yolov8_demo_open/test.jpg";
    char save_image_path[256] = "/home/rpdzkj/test_result.jpg";
    cv::Mat img = cv::imread(image_path);

    for (int i = 0; i < 3; i++)
    {
        modelInference *det = new modelInference(model_path, i);
        dets.push_back(det);
        dets[i]->init();
        dets[i]->img = img.clone();
        futs.push(pool.submit(&modelInference::detect, &(*dets[i])));
    }
    // 打开视频文件
    cv::VideoCapture cap("/home/rpdzkj/video/02_1080.mp4");
    // 检查视频是否成功打开
    if (!cap.isOpened())
    {
        std::cout << "无法打开视频文件" << std::endl;
        return -1;
    }
    cv::Mat frame;
    long count = 0;
    struct timeval time;
    gettimeofday(&time, nullptr);
    long tmpTime, lopTime = time.tv_sec * 1000 + time.tv_usec / 1000;

    while (cap.read(frame))
    {
        if (futs.front().get() != 0)
            break;
        futs.pop();
        dets[count % 3]->img = frame.clone();

        // cv::imshow("Video", frame);
        futs.push(pool.submit(&modelInference::detect, &(*dets[count++ % 3])));
        // cv::waitKey(20);
        if (count % 60 == 0)
        {
            gettimeofday(&time, nullptr);
            tmpTime = time.tv_sec * 1000 + time.tv_usec / 1000;
            printf("60帧平均帧率:\t%f帧\n", 60000.0 / (float)(tmpTime - lopTime));
            lopTime = tmpTime;
        }

        // gettimeofday(&stop_time, NULL);
        // printf("once run use %f ms\n", (__get_us(stop_time) - __get_us(start_time)) / 1000);

        // std::cout << count << std::endl;
    }
    for (int i = 0; i < 3; i++)
    {
        delete dets[i];
    }
    while (!futs.empty())
    {
        if (futs.front().get())
            break;
        futs.pop();
    }
    // for (auto &thread : threads)
    // {
    //     thread.join();
    // }
    // 释放视频捕获对象和窗口
    cap.release();
    return 0;
}
