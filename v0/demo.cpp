#include "idetector.h"
#include "kcftracker.hpp"
// #include "multitracker.h"
#include <yaml-cpp/yaml.h>
#include "spdlog/spdlog.h"
#include "spdlog/stopwatch.h"
#include "itracker.h"
#include "realtracker.h"

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
int trackOn;

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
                cv::rectangle(trackFrame, cv::Rect(userPt.x - 16, userPt.y - 16, 32,32),cv::Scalar( 48,48,255 ), 2, 8 );
                rtracker->init( userPt, trackFrame );
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
    int waitVAL =  argc > 1 ? 0 : 10;
    // spdlog::stopwatch sw;    
    // spdlog::info("Welcome to spdlog!");
    // spdlog::error("Some error message with arg: {}", 1);
    
    // spdlog::warn("Easy padding in numbers like {:08d}", 12);
    // spdlog::critical("Support for int: {0:d};  hex: {0:x};  oct: {0:o}; bin: {0:b}", 42);
    // spdlog::info("Support for floats {:03.2f}", 1.23456);
    // spdlog::info("Positional args are {1} {0}..", "too", "supported");
    // spdlog::info("{:<30}", "left aligned");
    
    spdlog::set_level(spdlog::level::debug); // Set global log level to debug
    // spdlog::debug("This message should be displayed..");    
    
    // // change log pattern
    // spdlog::set_pattern("[%H:%M:%S %z] [%n] [%^---%L---%$] [thread %t] %v");
    
    // // Compile time log levels
    // // define SPDLOG_ACTIVE_LEVEL to desired level
    // SPDLOG_TRACE("Some trace message with param {}", 42);
    // SPDLOG_DEBUG("Some debug message");

    
    // spdlog::debug("Elapsed {}", sw);
    // spdlog::debug("Elapsed {:.3}", sw);       

    // return 0;

    YAML::Node config = YAML::LoadFile("../config.yaml");
    int detOn = config["detection"].as<int>();
    trackOn = config["track"].as<int>();
    std::string engine = config["engine"].as<std::string>();
    std::string videopath = config["videopath"].as<std::string>();

    rtracker = new realtracker(engine);

    // idetector *detector = new idetector("/home/nx/model/vis-8s-2c.engine");
    // idetector *detector = new idetector(engine);
    // detector->init();
    // cv::VideoCapture cap("/home/nx/data/IMG_3575.MOV");
    cv::VideoCapture cap(videopath);
    // cv::VideoCapture cap("/space/data/tracking-test.mp4");
    if(!cap.isOpened())
    {
        printf("open failed\n");
        return 0;
    }

	itracker *tracker = new itracker();


    std::vector<bbox_t> boxs;


    cv::Mat frame;
    int nFrames = 0;

    float xMin;
    float yMin;
    float width;
    float height;

    cv::namedWindow("trackRet");
    cv::setMouseCallback("trackRet", onmouse);

    // Tracker results
    cv::Rect result;
    cv::Point pt;

    cv::Mat dispFrame, trackRet, detFrame, trackRetByDet;
    std::vector<TrackingObject> detRet;

    uint8_t trackerStatus[9];
    memset(trackerStatus, 0, 9);

    cv::Mat templt;

    while(1)
    {
        cap >> frame;
        if(frame.empty())
            break;

        // frame = cv::imread("/space/data/123.PNG");
        cv::resize(frame, frame, cv::Size(1280,720));

        trackFrame = frame.clone();
        detFrame = frame.clone();
        dispFrame = frame.clone();
        trackRetByDet = frame.clone();

        printf("=====nframe:%d======\n", nFrames);

#if 0
        if(trackOn)
        {
            if (nFrames == 0) {
                while(1)
                {
                    printf("1111111111\n");
                    auto tmpmat = frame.clone();
                    cv::rectangle( tmpmat, cv::Point(box.x,box.y), cv::Point(box.x+box.width,box.y+box.height), cv::Scalar( 48,48,255 ), 2, 8 );
                    cv::imshow("trackRet", tmpmat);
                    if(box_complete == true)
                    {
                        xMin = box.x;
                        yMin = box.y;
                        width = box.width;
                        height = box.height;
                        break;
                    }
                    cv::waitKey(30);
                }
                printf("WWWWWWW\n");
                // tracker->init( cv::Rect(xMin-GateSize/2, yMin-GateSize/2, GateSize, GateSize), frame );
                // rtracker->init( cv::Rect(xMin-GateSize/2, yMin-GateSize/2, GateSize, GateSize), frame );
                rtracker->init( cv::Point(xMin, yMin), frame );
                // rectangle( frame, Point( xMin, yMin ), Point( xMin+width, yMin+height), Scalar( 0, 255, 255 ), 1, 8 );
                // resultsFile << xMin << "," << yMin << "," << width << "," << height << endl;

                templt = frame(cv::Rect(xMin-GateSize/2, yMin-GateSize/2, GateSize, GateSize)).clone();

                spdlog::debug("tracker init pt:({},{})", xMin, yMin);
            }
            // Update
            else{
                // bool lost;
                // result = tracker->update(frame, lost);
                // // drawCrosshair(frame, cv::Point(result.x+result.width/2,result.y+result.height/2), 0.5);
                // rectangle(trackFrame, cv::Point( result.x, result.y ), cv::Point( result.x+result.width, result.y+result.height), cv::Scalar( 255,0,0 ), 2, 8 );
                // // resultsFile << result.x << "," << result.y << "," << result.width << "," << result.height << endl;
                // userPt.x = result.x+GateSize/2;
                // userPt.y = result.y+GateSize/2;
                // rtracker->runTracker(trackFrame);
                rtracker->update(trackFrame, detRet, trackerStatus);
                

                // cv::Mat templret;
                // matchTemplate(trackFrame,templt,templret,cv::TM_CCOEFF_NORMED);
                // double maxVal,minVal;
                // cv::Point minLoc,maxLoc;
                // minMaxLoc(templret,&minVal,&maxVal,&minLoc,&maxLoc);
                // //回执最佳匹配结果
                // rectangle(trackFrame,cv::Rect(maxLoc.x,maxLoc.y,templt.cols,templt.rows),cv::Scalar(135,32,156),2);


                // spdlog::debug("tracker lost:{}", lost);

                // if(lost || !contain)
                // {
                //     spdlog::debug("reset tracker ");
                //     //reset tracker
                //     cv::Rect brect = frameInfo.m_tracks[0][minIdx].m_rrect.boundingRect();
                //     cv::Point center{brect.tl().x + brect.width/2, brect.tl().y + brect.height/2};
                //     tracker->reset();
                //     tracker->init( cv::Rect(center.x-GateSize/2, center.y-GateSize/2, GateSize, GateSize), frame );
                // }

                spdlog::debug("tracker status:{}", trackerStatus[4]);
                cv::imshow("trackRet", trackFrame);
            }

            spdlog::debug("tracker end");
        }
#endif

        if(trackOn) {
            if (trackerInited) {
                rtracker->update(trackFrame, detRet, trackerStatus);
                spdlog::debug("tracker status:{}", trackerStatus[4]);
            }
            cv::imshow("trackRet", trackFrame);
        }
        nFrames++;

        if(detOn)
        {
            rtracker->runDetector(detFrame, detRet);
            // cv::imshow("show", trackRetByDet);

            // cv::resize(dispFrame, dispFrame, cv::Size(1280,720));
            cv::imshow("final-detRet", detFrame);
        }


        // cv::resize(dispFrame, dispFrame, cv::Size(1280,720));
        // cv::resize(trackFrame, trackFrame, cv::Size(1280,720));
        // cv::imshow("show", dispFrame);
        
        
        cv::waitKey(waitVAL);

    }
    // cv::Mat img = cv::imread("/space/data/0211/2-755.png");


    // detector->process(img);
    // cv::imshow("1", img);
    // cv::waitKey(0);

    return 0;
}