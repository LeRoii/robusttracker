/*
 * @Author: WJM
 * @Date: 2024-03-08 15:05:07
 * @LastEditors: WJM
 * @LastEditTime: 2024-03-11 14:43:52
 * @Description:
 * @FilePath: /rknn_yolov8_detector_v2/src/main.cpp
 * @custom_string: http://www.aiar.xjtu.edu.cn/
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <sys/time.h>
#include "model_inference.h"
#include "idetector.h"

int main(int argc, char **argv)
{
    const std::vector<cv::Scalar> predefinedColors = {
        cv::Scalar(255, 0, 0),   // 红色
        cv::Scalar(0, 255, 0),   // 绿色
        cv::Scalar(0, 0, 255),   // 蓝色
        cv::Scalar(255, 255, 0), // 黄色
        cv::Scalar(255, 0, 255), // 粉色
        cv::Scalar(0, 255, 255), // 青色
        cv::Scalar(255, 127, 0), // 橙色
        cv::Scalar(127, 0, 255), // 紫色
        cv::Scalar(0, 127, 255), // 天蓝色
        cv::Scalar(127, 255, 0), // 酸橙色
        cv::Scalar(255, 0, 127), // 玫瑰红
        cv::Scalar(0, 255, 127), // 春绿色
        // ... 或许还可以添加更多的颜色，如果类别有增加
    };
    char model_path[256] = "/home/rpdzkj/wjm/pinlingv2.3.1/pinlingv2.3/rknn_yolov8_detector_v2/model/visdrone_self_s_i8.rknn";
    // CModelInference modelInference(model_path, 1, 10, 0.45, 0.2);
    CDetector *idet = new CDetector(model_path, 3, 10, 0.3, 0.15);
    idet->Init();

    cv::VideoWriter writer;
    int codec = cv::VideoWriter::fourcc('M', 'P', 'E', 'G');         // 选择合适的编解码器
    writer.open("output.avi", codec, 30, cv::Size(1280, 720), true); // 假设我们想保存彩色视频
    cv::Mat frame;
    // 打开视频文件
    cv::VideoCapture cap("/home/rpdzkj/video/2_1.mp4");
    std::cout << "start inference" << std::endl;
    if (!cap.isOpened())
    {
        std::cout << "无法打开视频文件" << std::endl;
        return -1;
    }
    int frame_count = 0;
    bbox_t boxs[OBJ_NUMB_MAX_SIZE];
    int boxs_count = 0;

    while (cap.read(frame) && frame_count < 400)
    {
        if (frame.empty())
        {
            printf("input img empty, quit\n");
            continue;
        }
        idet->ImgInference(frame, boxs, boxs_count);
        // printf("*******************boxs_count: %d\n", boxs_count);
        for (int i = 0; i < boxs_count; i++)
        {
            // printf("%d @ (%d %d %d %d) %.3f\n", boxs[i].obj_id,
            //        boxs[i].x, boxs[i].y,
            //        boxs[i].w, boxs[i].h,
            //        boxs[i].prop);
            int x = boxs[i].x;
            int y = boxs[i].y;
            int w = boxs[i].w;
            int h = boxs[i].h;
            cv::rectangle(frame, cv::Rect(x, y, w, h), predefinedColors[boxs[i].obj_id], 2);
            // 在框的左上角添加obj_id文本
            std::string id_text = std::to_string(boxs[i].obj_id);
            cv::putText(frame, id_text, cv::Point(boxs[i].x, boxs[i].y), cv::FONT_HERSHEY_SIMPLEX, 1, predefinedColors[boxs[i].obj_id], 2);
        }

        // cv::imwrite("img/"+std::to_string(frame_count)+".jpg",frameQueue.front());
        frame_count++;
        // std::cout << frame_count << std::endl;
        writer.write(frame);
    }
    delete idet;
    writer.release();
    std::cout << "main exit" << std::endl;

    cap.release();
    std::cout << "main exit" << std::endl;
    return 0;
}