#include "idetector.h"
#include "kcftracker.hpp"
// #include "multitracker.h"
#include <yaml-cpp/yaml.h>
#include "spdlog/spdlog.h"
#include "spdlog/stopwatch.h"
#include "itracker.h"
#include "realtracker.h"

void drawRect(cv::Mat frame, cv::Rect r, cv::Scalar color = cv::Scalar(0,255,255))
{
    cv::Point tl{r.x, r.y};
    cv::Point bl{r.x,r.y + r.height};
    cv::Point tr{r.x + r.width, r.y};
    cv::Point br{r.x + r.width, r.y+r.height};

    int xlinelen = r.width/4;
    int ylinelen = r.height/4;
    int thickness = 3;

    cv::line(frame, tl, cv::Point(tl.x + xlinelen, tl.y), color, thickness);
    cv::line(frame, tl, cv::Point(tl.x, tl.y + ylinelen), color, thickness);

    cv::line(frame, bl, cv::Point(bl.x + xlinelen, bl.y), color, thickness);
    cv::line(frame, bl, cv::Point(bl.x, bl.y - ylinelen), color, thickness);

    cv::line(frame, tr, cv::Point(tr.x - xlinelen, tr.y), color, thickness);
    cv::line(frame, tr, cv::Point(tr.x, tr.y + ylinelen), color, thickness);

    cv::line(frame, br, cv::Point(br.x - xlinelen, br.y), color, thickness);
    cv::line(frame, br, cv::Point(br.x, br.y - ylinelen), color, thickness);
}

void drawLostRect(cv::Mat frame, cv::Rect r)
{
    int s = 3;
    int w = r.width * s;
    int h = r.height * s;
    cv::Point pt(r.x+r.width/2, r.y+r.height/2);
    cv::Rect rc(pt.x - w/2, pt.y - h/2, w, h);
    drawRect(frame, rc, cv::Scalar(0,255,0));
}

cv::Rect box;//矩形对象
bool drawing_box = false;//记录是否在画矩形对象
bool box_complete = false;
cv::Point userPt;
int GateSize = 32;
int minIdx = -1;

bool contain = false;

bool trackerInited = false;
realtracker *rtracker = nullptr;
cv::Mat trackFrame;
cv::Mat dispFrame, trackRet, detFrame, trackRetByDet;
int trackOn;

int gateS = 64;

void onmouse(int event, int x, int y, int flag, void*)//鼠标事件回调函数，鼠标点击后执行的内容应在此
{
    // cv::Mat& image = *(cv::Mat*) img;
    switch (event)
    {
    case cv::EVENT_LBUTTONDOWN://鼠标左键按下事件
        drawing_box = true;//标志在画框
        box = cv::Rect(x, y, 0, 0);//记录矩形的开始的点
        userPt.x = x;    //center point
        userPt.y = y;
        break;
    case cv::EVENT_MOUSEMOVE://鼠标移动事件
        if (drawing_box) {//如果左键一直按着，则表明在画矩形
            box.width = x - box.x;
            box.height = y - box.y;//更新长宽
        }
        break;
    case cv::EVENT_LBUTTONUP://鼠标左键松开事件
    {
        //不在画矩形
                            //这里好像没作用
        if (box.width < 0) {//排除宽为负的情况，在这里判断是为了优化计算，不用再移动时每次更新都要计算长宽的绝对值
            box.x = box.x + box.width;//更新原点位置，使之始终符合左上角为原点
            box.width = -1 * box.width;//宽度取正
        }
        if (box.height < 0) {//同上
            box.y = box.y + box.height;
            box.height = -1 * box.width;
        }
        // g_nCount++;
        // cv::Mat dst = image(box);
        // std::string str_save_name = std::to_string(g_nCount) + ".jpg";
        // cv::imwrite(str_save_name.c_str(), dst);
        printf("mouse btn up event, x:%d,y:%d,w:%d,h:%d\n", box.x, box.y, box.width, box.height);
        drawing_box = true;
        box_complete = true;
        if (trackOn) {
            if (rtracker) {
                rtracker->reset();
                rtracker->init( userPt, dispFrame, dispFrame);
                cv::rectangle(trackFrame, cv::Rect(userPt.x - gateS/2, userPt.y - gateS/2, gateS,gateS),cv::Scalar( 48,48,255 ), 2, 8 );
                trackerInited = true;
                cv::imshow("trackRet", trackFrame);
            }
        }
        
    }
        
        break;
    default:
        break;
    }
}

int main(int argc, char*argv[])
{
    int waitVAL =  argc > 1 ? 10 : 0;
    
    spdlog::set_level(spdlog::level::debug); // Set global log level to debug

    YAML::Node config = YAML::LoadFile("../config.yaml");
    int detOn = config["detection"].as<int>();
    trackOn = config["track"].as<int>();
    std::string engine = config["engine"].as<std::string>();
    std::string videopath = config["videopath"].as<std::string>();
    std::string irEngine = config["irengine"].as<std::string>();

    rtracker = new realtracker("/home/rpdzkj/robusttracker-algodev/v0/tracker.yaml");
    // rtracker->setGateSize(gateS);
    gateS = rtracker->osdw;

    cv::VideoCapture cap(videopath);
    if(!cap.isOpened())
    {
        printf("open failed\n");
        return 0;
    }

    std::vector<bbox_t> boxs;

    cv::Mat frame;
    int nFrames = 0;

    cv::namedWindow("trackRet");
    cv::setMouseCallback("trackRet", onmouse);

    uint8_t trackerStatus[9];
    memset(trackerStatus, 0, 9);

    bbox_t detRet[200];

    cv::VideoWriter video;//("output.avi", fourcc,30.0, cv::Size(640, 512));
    bool recvid = false;

    while(1)
    {
        cap >> frame;
        // frame = cv::imread("/home/rpdzkj/3/model/1.jpg");
        if(frame.empty())
            break;

        // frame = cv::imread("/space/data/123.PNG");
        cv::resize(frame, frame, cv::Size(1280,720));

        trackFrame = frame.clone();
        detFrame = frame.clone();
        dispFrame = frame.clone();
        trackRetByDet = frame.clone();

        printf("=====nframe:%d======\n", nFrames);

        int center_x,center_y;
        cv::Rect trackRect;
        int boxes_count;
        if(trackOn) {
            if (trackerInited) {
                rtracker->update(trackFrame, trackFrame, trackerStatus, center_x, center_y, trackRect);

                 if(rtracker->trackerLost())
                {
                    drawLostRect(trackFrame, trackRect);
                }
                else
                    drawRect(trackFrame, trackRect);

                spdlog::debug("tracker status:{}", trackerStatus[4]);

                cv::resize(trackFrame, trackFrame, cv::Size(640,360));
            }

            cv::putText(trackFrame, std::to_string(nFrames), cv::Point(150, 100), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,255), 2, cv::LINE_AA);

            
            cv::imshow("trackRet", trackFrame);
            if(recvid)
            {
                video.write(frame);
            }
        }
        

        if(detOn)
        {
            rtracker->runDetectorOut(detFrame, detRet, boxes_count);


            for (int i = 0; i < boxes_count; ++i)
            {
                bbox_t &box = detRet[i];
                    drawRect(detFrame, cv::Rect(cv::Point(box.x, box.y), cv::Point(box.x + box.w, box.y + box.h)), cv::Scalar{255,0,0});
            }
            // cv::imshow("final-detRet", detFrame);
        }

        char c = cv::waitKey(waitVAL);
        if(c == 'g')
            waitVAL = 1;
        else if(c == 's')
            waitVAL = 0;
        else if(c == 'r')
        {
            recvid = true;
            video.open(std::to_string(userPt.x) + "-" + std::to_string(userPt.y) + "_.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),30.0, cv::Size(1280,720));
            spdlog::debug("video record start");
        }
        else if(c == 't')
        {
            recvid = false;
            video.release();
            spdlog::debug("video record stop");
        }

        nFrames++;
    }

    return 0;
}