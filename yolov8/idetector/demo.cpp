#include "idetector.h"

int main()
{
    // idetector *detector = new idetector("/space/model/yolov8n.engine");
    idetector *detector = new idetector("/space/model/visdrone-8s-2c-1011.engine");
    detector->init();

    cv::Mat img = cv::imread("/space/data/0211/2-755.png");

    cv::VideoCapture cap("/space/data/pl/IMG_3573.MOV");
    if(!cap.isOpened())
    {
        printf("open failed\n");
        return 0;
    }
    std::vector<bbox_t> boxs;
    while(1)
    {
        cap >> img;
        if(img.empty())
            break;

        cv::resize(img, img, cv::Size(1280,720));

        detector->process(img, boxs);

        for(auto& box:boxs)
        {
            printf("box-->x:%d, y:%d, w:%d, h:%d, conf:%f, cls:%d\n", box.x, box.y, box.w, box.h, box.prob, box.obj_id);
        }
        cv::imshow("1", img);
        cv::waitKey(0);
    }

    

    return 0;
}