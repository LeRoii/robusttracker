#include <iostream>
#include "cap.hpp"
int main()
{
    USBCamera *irCapture = new USBCamera("/dev/video20", 256, 192, 50);
    if (!irCapture->openDevice())
    {
        return -1;
    }
    if (!irCapture->queryCapability())
    {
        return -1;
    }

    if (!irCapture->initDevice())
    {
        return -1;
    }
    if (!irCapture->mmap_v4l2_buffer())
    {
        return -1;
    }
    if (!irCapture->startCapture())
    {
        return -1;
    }

    while (true)
    {

        cv::Mat frame = irCapture->writeVideoFrame();
        // 显示或处理frame
        cv::imshow("sdsa",frame);

        if (cv::waitKey(10) == 27)
        {
            break; // 按ESC退出
        }
    }

    return 0;
}