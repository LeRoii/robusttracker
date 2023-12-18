
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <vector>
#include <thread>

#include "RgaUtils.h"
#include "im2d.h"
#include <opencv2/opencv.hpp>
#include "idetector.h"
int main(int argc, char **argv)
{
    // 线程池操作
    std::vector<bbox_t> boxs;

    // char model_path[256] = "/home/rpdzkj/code/yolov8_rknn_Cplusplus/examples/rknn_yolov8_demo_open/model/RK3588/yolov8n_ZQ.rknn";
    char model_path[256] = "/home/rpdzkj/code/obj_detect_multhread/examples/rknn_yolov8_demo_open/model/RK3588/yolox_RK3588_i8.rknn";
    char save_image_path[256] = "/home/rpdzkj/test_result.jpg";
    char image_path[256] = "/home/rpdzkj/code/obj_detect_multhread/examples/rknn_yolov8_demo_open/test.jpg";
    cv::Mat img_1 = cv::imread(image_path);
    // 打开视频文件
    cv::VideoCapture cap("/home/rpdzkj/video/02_1080.mp4");
    idetector *idet = new idetector(model_path, 1);
    idet->init();
    // 检查视频是否成功打开
    if (!cap.isOpened())
    {
        std::cout << "无法打开视频文件" << std::endl;
        return -1;
    }
    cv::Mat frame;
    cv::Mat pre_frame;
    pre_frame = cv::Mat::zeros(1920,1080,CV_8UC3);
    while (cap.read(frame))
    // int i=0;
    // while(i<100)
    {
        idet->process(frame,boxs);
        if (boxs.size() > 0)
        {
            for (int i = 0; i < boxs.size(); i++)
            {
                cv::rectangle(pre_frame, cv::Point(boxs[i].x, boxs[i].y), cv::Point(boxs[i].x + boxs[i].w, boxs[i].y + boxs[i].h), cv::Scalar(255,0,0),2);
            }
        }
        cv::imshow("frame", pre_frame);
        cv::waitKey(1);
        pre_frame = frame.clone();
        // if (boxs.size() > 0)
        //     std::cout << boxs.size() << "  " << boxs[0].x << " "<<boxs[0].y <<" "<< boxs[0].w <<" "<< boxs[0].h << std::endl;
        // boxs.clear();
        // i++;
    }
    delete idet;
    std::cout<<"main exit"<<std::endl;
    
    cap.release();
    std::cout<<"main exit"<<std::endl;
    return 0;
}
