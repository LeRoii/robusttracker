
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "RgaUtils.h"
#include "im2d.h"
#include <opencv2/opencv.hpp>
#include "v4l2_capture.hpp"
#include "idetector.h"
int main(int argc, char **argv)
{
    char model_path[256] = "/home/rpdzkj/code/yolov8_rknn_Cplusplus/examples/rknn_yolov8_demo_open/model/RK3588/yolov8n_ZQ.rknn";
    char image_path[256] = "/home/rpdzkj/code/obj_detect_multhread/examples/rknn_yolov8_demo_open/test.jpg";
    char save_image_path[256] = "/home/rpdzkj/test_result.jpg";
    cv::Mat img = cv::imread(image_path);
    vector<iDetector *> dets;
    for (int i = 1;i<3;i++){
        iDetector * det = new iDetector(model_path,i)
        dets.emplace_back(det);
        dets[i]->init();
    }


    det->init();
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
    struct timeval start_time, stop_time;
    while (cap.read(frame))
    {
        gettimeofday(&start_time, NULL);
        det->detect(frame);
        gettimeofday(&stop_time, NULL);
        printf("once run use %f ms\n", (__get_us(stop_time) - __get_us(start_time)) / 1000);
        cv::imshow("Video", frame);
        cv::waitKey(1);
        count++;
        // std::cout << count << std::endl;
    }
    delete det;
    // 释放视频捕获对象和窗口
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
