#include "idetector.h"

int main()
{
    // idetector *detector = new idetector("/space/model/yolov8n.engine");
    idetector *detector = new idetector("/home/nx/model/vis-8s-2c.engine");
    detector->init();

    cv::Mat img;// = cv::imread("/space/data/0211/2-755.png");

    cv::VideoCapture cap("/home/nx/1016/IMG_3065.MOV");
    if(!cap.isOpened())
    {
        printf("open failed\n");
        return 0;
    }

    while(1)
    {
        cap >> img;
        if(img.empty())
            break;

        detector->process(img);
        cv::imshow("1", img);
        cv::waitKey(0);
    }

    

    return 0;
}