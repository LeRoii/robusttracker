#include "idetector.h"
#include "kcftracker.hpp"
#include "multitracker.h"
#include "sdireader.h"
#include <unistd.h>
#include <signal.h>
#include "jetsonEncoder.h"
#include "serial.h"
#include "common.h"
#include <sstream>
#include <cmath>

extern ST_A1_CONFIG stA1Cfg;
extern ST_A2_CONFIG stA2Cfg;
extern ST_C1_CONFIG stC1Cfg;
extern ST_C2_CONFIG stC2Cfg;
extern ST_E1_CONFIG stE1Cfg;
extern ST_E2_CONFIG stE2Cfg;
extern ST_S1_CONFIG stS1Cfg;
extern ST_S2_CONFIG stS2Cfg;
extern ST_U_CONFIG stUCfg;
extern ST_A1C1E1_CONFIG stA1C1E1Cfg;
extern ST_A2C2E2_CONFIG stA2C2E2Cfg;
extern ST_A1C1E1S1_CONFIG stA1C1E1S1Cfg;
extern ST_A2C2E2S2_CONFIG stA2C2E2S2Cfg;

ST_SYS_STATUS stSysStatus;

static void ParseE1Msg()
{

    // if(stE1Cfg.enBaseOpMode == AiIdentifySwitch)
    // {
    // 	if(stE1Cfg.u8Para2 == 1)
    // 	{
    // 		DetOn = true;
    // 	}
    // 	else
    // 	{
    // 		DetOn = false;
    // 	}
    // }




}

cv::Rect box;//矩形对象
bool drawing_box = false;//记录是否在画矩形对象
bool box_complete = false;
cv::Point userPt;

bool quit = false;

static void signal_handle(int signum)
{
    quit = true;
    // _xdma_reader_ch0.xdma_close();
}

void onmouse(int event, int x, int y, int flag, void*)//鼠标事件回调函数，鼠标点击后执行的内容应在此
{
    // cv::Mat& image = *(cv::Mat*) img;
    switch (event)
    {
    case cv::EVENT_LBUTTONDOWN://鼠标左键按下事件
        drawing_box = true;//标志在画框
        box = cv::Rect(x, y, 0, 0);//记录矩形的开始的点
        userPt.x = x;
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
    }
        
        break;
    default:
        break;
    }
}

Serial serialUp, serialDown;
void SerialTransUp2Down()
{
    uint8_t output[1024] = {0};
    int outLen = 0;
    uint8_t buffRcvData_servo[1024] = {0};
    while(1)
    {
        int retLen = serialUp.serial_recieve(buffRcvData_servo);
        if(retLen > 0)
        {
            std::cout<<"=======>serialUp received "<<std::dec<<retLen<<"bytes"<<std::endl;
            
            // for(int i=0; i< retLen ;i++)
            // {
            // 	printf("[%02X]", buffRcvData_servo[i]);
            // }
            // printf("\n");

            retLen = serialDown.serial_send(buffRcvData_servo, retLen);
            printf("up send to down:%d\n", retLen);

            int rr = serialUp.ProcessSerialData(buffRcvData_servo, retLen, output, outLen);
            if(rr == RET_ERR)
            {
                printf("RET_ERR\n");
                continue;
            }

            // printf("output buf\n");
            // for(int i=0; i< outLen ;i++)
            // {
            // 	printf("[%02X]", output[i]);
            // }
            // printf("\n");

            //check frame header
            // if(!CheckFrameHeader(buffRcvData_servo, retLen))
            // {
            // 	for(int i=0; i< retLen ;i++)
            // 	{
            // 		printf("[%02X]", buffRcvData_servo[i]);
            // 	}
            // 	printf("\n");
            // 	printf("frame header error, drop data\n\n");
            // 	memset(buffRcvData_servo,0,1024);
            // 	continue;
            // }

            EN_DATA_FRAME_TYPE frameType = GetFrameType(output, outLen);
            printf("frame type:%d\n", frameType);
            if(frameType == HeartBeat15)
            {
                printf("heart beat 15 from up serial\n\n");
                continue;
            }
            else if(frameType == HandShake)
            {
                printf("handshake from up serial\n\n");
                continue;
            }
            else if(frameType == FrameS2)
            {
                printf("TGCC Ctrl S2 from up serial\n\n");
                continue;
            }

            for(int i=0; i< outLen ;i++)
            {
                printf("[%02X]", output[i]);
            }
            printf("\n");

            // for(int i=0; i< retLen ;i++)
            // {
            // 	printf("[%02X]", buffRcvData_servo[i]);
            // }
            // printf("\n");

            //checksum
            // uint8_t checksum = viewlink_protocal_checksum(buffRcvData_servo);
            // if(checksum != buffRcvData_servo[retLen - 1])
            // {
            // 	printf("frame checksum error, drop data\n\n");
            // 	memset(buffRcvData_servo,0,1024);
            // 	continue;
            // }

            VL_ParseSerialData(output);

            printf("status->enDispMode:%d, detOn:%d, trackOn:%d, trackerGateSize:%d\n",\
             stSysStatus.enDispMode, stSysStatus.detOn, stSysStatus.trackOn, stSysStatus.trackerGateSize);
            printf("\n\n");
        }
    }
}

void SerialTransDown2Up()
{
    uint8_t buffRcvData_servo[1024] = {0};
    uint8_t output[1024] = {0};
    int outLen = 0;
    while(1)
    {	
        int retLen = serialDown.serial_recieve(buffRcvData_servo);
        if(retLen > 0)
        {
            std::cout<<"<===========serial down received "<<std::dec<<retLen<<"bytes"<<std::endl;
            retLen = serialUp.serial_send(buffRcvData_servo, retLen);
            printf("down send to up:%d\n", retLen);

            int rr = serialDown.ProcessSerialData(buffRcvData_servo, retLen, output, outLen);

            if(rr == RET_ERR)
            {
                printf("RET_ERR\n");
                continue;
            }

            // printf("output buf\n");
            // for(int i=0; i< outLen ;i++)
            // {
            // 	printf("[%02X]", output[i]);
            // }
            // printf("\n");

            // continue;
            
            // if(!CheckFrameHeader(buffRcvData_servo, retLen))
            // {
            // 	for(int i=0; i< retLen ;i++)
            // 	{
            // 		printf("[%02X]", buffRcvData_servo[i]);
            // 	}
            // 	printf("\n");
            // 	printf("frame header error, drop data\n\n");
            // 	memset(buffRcvData_servo,0,1024);
            // 	continue;
            // }

            EN_DATA_FRAME_TYPE frameType = GetFrameType(output, outLen);
            
            if(frameType == Status41 || frameType == Status40)
            {
                printf("Status from down serial frameType:%d\n\n", frameType);
                continue;
            }

            printf("!!!non Status frametype\n");
            for(int i=0; i< outLen ;i++)
            {
                printf("[%02X]", output[i]);
            }
            printf("\n");

            // for(int i=0; i< retLen ;i++)
            // {
            // 	printf("[%02X]", buffRcvData_servo[i]);
            // }
            // printf("\n");

            printf("\n\n");
        }
        
    }
}

static std::string Convert(float Num)
{
    std::ostringstream oss;
    oss<<Num;
    std::string str(oss.str());
    return str;
}

// 绘制画面横滚角度数轴，画面从左到右对应x由小到大
static void PaintRollAngleAxis(cv::Mat &frame0, float currRollAngle)
{
    int fHeight = frame0.rows;
    int fWidth = frame0.cols;
    int xStart = fWidth / 2;
    int yStart = fHeight / 12;
    int curCeil = ceil(currRollAngle);
    int curFloor = floor(currRollAngle);
    uint32_t distance = abs(curCeil % 5);
    int x = xStart - 225 + ((float)curCeil - currRollAngle) * 15;
    
    // 绘制当前横滚角度
    std::string currStr = Convert(currRollAngle);
    cv::line(frame0, cv::Point(xStart, yStart + 10), cv::Point(xStart, yStart + 30), cv::Scalar(0, 255, 0), 2, cv::LINE_8);
    cv::putText(frame0, currStr, cv::Point(xStart - 10, yStart + 45), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2, cv::LINE_8);
    
    // 计算左侧和右侧小刻度值，以及开始大刻度具体数值
    int leftSmallScale = 0;
    int rightSmallScale = 0;
    int tempPos = currRollAngle - 15;
    if (curCeil == curFloor && curCeil % 5 == 0) { // 5的倍数，类似0、5、10等
        tempPos = curCeil - 15;
        leftSmallScale = 0;
        rightSmallScale = 0;
    } else if (curCeil - curFloor == 1 && curCeil % 5 - curFloor % 5 == 1) { // 类似-139.5、0.5、1.5等等
        int temp = currRollAngle / 5 * 5;
        tempPos = temp - 10;
        leftSmallScale = 5 - distance;
        rightSmallScale = 5 - leftSmallScale;
        if (5 - distance == 5) { // 当前位置向上取整为0的
            tempPos -= 5;
            leftSmallScale = 0;
            rightSmallScale = 0;
        }
    } else if (curCeil - curFloor == 1 && curCeil / 5 - curFloor / 5 == 1) { // 类似-0.5、4.5等等向上取整为5的倍数的
        if (curCeil < 0 && curFloor < 0) { // 当前角度小于0，向上向下取整均小于0的
            int temp = curCeil / 5 * 5;
            tempPos = temp - 15;
            leftSmallScale = distance;
            rightSmallScale = 5 - leftSmallScale;
        } else {
            tempPos = curCeil - 15;
            leftSmallScale = 0;
            rightSmallScale = 0;
        }
    }

    // 左侧小刻度绘制
    for (int i = 0; i < leftSmallScale; i++) {
        cv::line(frame0, cv::Point(x, yStart - 5), cv::Point(x, yStart - 12), cv::Scalar(0, 255, 0), 2, cv::LINE_8);
        x += 15;
    }
    
    // 绘制从第一个大刻度起的连续5段刻度
    for (int i = 0; i < 4; i++) {
        cv::line(frame0, cv::Point(x, yStart), cv::Point(x, yStart - 20), cv::Scalar(0, 255, 0), 2, cv::LINE_8);
        cv::putText(frame0, Convert(tempPos), cv::Point(x - 10, yStart - 35), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2, cv::LINE_8);
        tempPos += 5;
        x += 15;
        for (int j = 0; j < 4; j++) {
            cv::line(frame0, cv::Point(x, yStart - 5), cv::Point(x, yStart - 12), cv::Scalar(0, 255, 0), 2, cv::LINE_8);
            x += 15;
        }
    }

    // 接着上面绘制一个大刻度
    cv::line(frame0, cv::Point(x, yStart), cv::Point(x, yStart - 20), cv::Scalar(0, 255, 0), 2, cv::LINE_8);
    cv::putText(frame0, Convert(tempPos), cv::Point(x - 10, yStart - 35), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2, cv::LINE_8);

    // 当前刻度若为5的倍数，则再绘制两段刻度退出，最后一个刻度是大刻度 
    if (curCeil == curFloor && curCeil % 5 == 0) {
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 4; j++) {
                x += 15;
                cv::line(frame0, cv::Point(x, yStart - 5), cv::Point(x, yStart - 12), cv::Scalar(0, 255, 0), 2, cv::LINE_8);
            }
            x += 15;
            tempPos += 5;
            cv::line(frame0, cv::Point(x, yStart), cv::Point(x, yStart - 20), cv::Scalar(0, 255, 0), 2, cv::LINE_8);
            cv::putText(frame0, Convert(tempPos), cv::Point(x - 10, yStart - 35), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2, cv::LINE_8);
        }

        return;
    }

    // 当前刻度若向上取整为5的倍数，则接着需要绘制两段刻度
    int r = (rightSmallScale == 0) ? 2 : 1;
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < 4; j++) {
            x += 15;
            cv::line(frame0, cv::Point(x, yStart - 5), cv::Point(x, yStart - 12), cv::Scalar(0, 255, 0), 2, cv::LINE_8);
        }
        x += 15;
        tempPos += 5;
        cv::line(frame0, cv::Point(x, yStart), cv::Point(x, yStart - 20), cv::Scalar(0, 255, 0), 2, cv::LINE_8);
        cv::putText(frame0, Convert(tempPos), cv::Point(x - 10, yStart - 35), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2, cv::LINE_8);
    }
    
    // 绘制最右侧小刻度
    for (int j = 0; j < rightSmallScale; j++) {
        x += 15;
        cv::line(frame0, cv::Point(x, yStart - 5), cv::Point(x, yStart - 12), cv::Scalar(0, 255, 0), 2, cv::LINE_8);
    }
}

// 绘制俯仰角度刻度数轴，画面从上到下对应y由小到大
static void PaintPitchAngleAxis(cv::Mat &frame0, float currPitchAngle)
{
    int fHeight = frame0.rows;
    int fWidth = frame0.cols;
    int xStart = fWidth / 18;
    int yStart = fHeight / 2;
    
    int curCeil = ceil(currPitchAngle);
    int y = yStart - 150 - 15 - ((float)curCeil - currPitchAngle) * 15;
    uint32_t distance = abs(curCeil % 5);
    
    // 绘制当前俯仰角度
    std::string currStr = Convert(currPitchAngle);
    cv::line(frame0, cv::Point(xStart + 10, yStart), cv::Point(xStart + 20, yStart), cv::Scalar(0, 255, 0), 2, cv::LINE_8);
    cv::putText(frame0, currStr, cv::Point(xStart + 30, yStart), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2, cv::LINE_8);

    // 绘制最上侧小刻度
    for (int i = 0; i < distance; i++) {
        y += 15;
        cv::line(frame0, cv::Point(xStart - 7, y), cv::Point(xStart - 15, y), cv::Scalar(0, 255, 0), 2, cv::LINE_8);
    }

    // 从上到下连续绘制3段刻度
    for (int i = -2; i <= 0; i++) {
        y += 15;
        cv::line(frame0, cv::Point(xStart, y), cv::Point(xStart - 20, y), cv::Scalar(0, 255, 0), 2, cv::LINE_8);
        cv::putText(frame0, Convert(((int)currPitchAngle / 5) * 5 - i * 5), cv::Point(xStart - 65, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2, cv::LINE_8);
        for (int j = 0; j < 4; j++) {
            y += 15;
            cv::line(frame0, cv::Point(xStart - 7, y), cv::Point(xStart - 15, y), cv::Scalar(0, 255, 0), 2, cv::LINE_8);
        }
    }

    // 接着绘制一个大刻度
    y += 15;
    cv::line(frame0, cv::Point(xStart, y), cv::Point(xStart - 20, y), cv::Scalar(0, 255, 0), 2, cv::LINE_8);
    cv::putText(frame0, Convert(((int)currPitchAngle / 5) * 5 - 5), cv::Point(xStart - 65, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2, cv::LINE_8);
    
    // 如果是5的倍数，则再绘制一段刻度退出，最下面一个刻度是大刻度
    if (distance == 0) {
        for (int j = 0; j < 4; j++) {
            y += 15;
            cv::line(frame0, cv::Point(xStart - 7, y), cv::Point(xStart - 15, y), cv::Scalar(0, 255, 0), 2, cv::LINE_8);
        }
        y += 15;
        cv::line(frame0, cv::Point(xStart, y), cv::Point(xStart - 20, y), cv::Scalar(0, 255, 0), 2, cv::LINE_8);
        cv::putText(frame0, Convert(((int)currPitchAngle / 5) * 5 - 10), cv::Point(xStart - 65, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2, cv::LINE_8);
        
        return;
    }

    // 绘制最下侧小刻度
    for (int j = 0; j < (5 - distance); j++) {
        y += 15;
        cv::line(frame0, cv::Point(xStart - 5, y), cv::Point(xStart - 12, y), cv::Scalar(0, 255, 0), 2, cv::LINE_8);
    }
}

static void PaintCrossPattern(cv::Mat &frame0, float currRollAngle, float  currPitchAngle)
{
    int fHeight = frame0.rows;
    int fWidth = frame0.cols;
    int x = fWidth / 2;
    int y = fHeight / 2;
    int lineLen = fHeight / 12;

    // 绘制x线
    cv::line(frame0, cv::Point(x - lineLen, y), cv::Point(x - lineLen / 4, y), cv::Scalar(0, 255, 0), 2, cv::LINE_8);
    cv::line(frame0, cv::Point(x + lineLen, y), cv::Point(x + lineLen / 4, y), cv::Scalar(0, 255, 0), 2, cv::LINE_8);

    // 绘制中心点
    cv::circle(frame0, cv::Point(x, y), 2, cv::Scalar(0, 255, 0), -1, cv::LINE_8);

    // 绘制y线
    cv::line(frame0, cv::Point(x, y - lineLen), cv::Point(x, y - lineLen / 4), cv::Scalar(0, 255, 0), 2, cv::LINE_8);
    cv::line(frame0, cv::Point(x, y + lineLen), cv::Point(x, y + lineLen / 4), cv::Scalar(0, 255, 0), 2, cv::LINE_8);
}

int main()
{
//*******************************serial init*************************
    
    serialUp.set_serial(1);	//"/dev/ttyTHS1"
    // serialUp.OnStart();
    serialDown.set_serial(2);	//"/dev/ttyUSB0"
    // serialDown.OnStart();

    std::thread serialThUp2Down = std::thread(SerialTransUp2Down);
    serialThUp2Down.detach();
    std::thread serialThDown2Up = std::thread(SerialTransDown2Up);
    serialThDown2Up.detach();
//*******************************serial end*************************

    jetsonEncoder *encoder = new jetsonEncoder(8554);

    struct sigaction sig_action;
    sig_action.sa_handler = signal_handle;
    sigemptyset(&sig_action.sa_mask);
    sig_action.sa_flags = 0;
    sigaction(SIGINT, &sig_action, NULL);

    idetector *detector = new idetector("/home/nx/model/vis-8s-2c.engine");
    // detector->init();
    // cv::VideoCapture cap("/home/nx/1016/IMG_3065.MOV");
    // cv::VideoCapture cap("/space/data/tracking-test.mp4");
    // if(!cap.isOpened())
    // {
    //     printf("open failed\n");
    //     return 0;
    // }

    //*******************************multitracker init*************************
    size_t m_batchSize = 1;
    float m_fps = 25;
    const int minStaticTime = 5;

    cv::Mat tmp = cv::Mat(720, 1280, CV_8UC3);
    FrameInfo frameInfo(m_batchSize);
    frameInfo.m_frames.resize(frameInfo.m_batchSize);
    frameInfo.m_frameInds.resize(frameInfo.m_batchSize);
    frameInfo.m_frames[0].GetMatBGRWrite() = tmp;
    cv::UMat umatFrame = frameInfo.m_frames[0].GetUMatBGR();

    std::unique_ptr<BaseTracker> mtracker;
    TrackerSettings settings;
    genTrackerSettings(settings);
    mtracker = BaseTracker::CreateTracker(settings);
    frameInfo.CleanRegions();
    frameInfo.CleanTracks();
    regions_t regions;
//*******************************multitracker init end *************************

    std::vector<bbox_t> boxs;

    bool HOG = false;
    bool FIXEDWINDOW = false;
    bool MULTISCALE = true;
    bool SILENT = true;
    bool LAB = false;
    KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

    cv::Mat frame,frame0,frame1;
    int nFrames = 0;

    float xMin;
    float yMin;
    float width;
    float height;

    // cv::namedWindow("show");
    // cv::setMouseCallback("show", onmouse);

    // Tracker results
    cv::Rect result;

    cv::Mat dispFrame, trackFrame, detFrame;

    // Sdireader_Init("/etc/jetsoncfg/NXConfig.ini");
    // sleep(2);

    frame = cv::imread("/home/nx/data/1.png");

    while(!quit)
    {
        // cap >> frame;
        // if(frame.empty())
        //     break;

        // Sdireader_GetFrame(frame0, frame1); 
        // frame = frame0;
        // 在界面上绘制OSD
        float currRollAngle = 0; // 需要修改为吊舱返回的当前横滚角度
        PaintRollAngleAxis(frame, currRollAngle);

        float currPitchAngle = 0; // 需要修改为吊舱返回的当前俯仰角度
        PaintPitchAngleAxis(frame, currPitchAngle);

        // 绘制中心十字
        PaintCrossPattern(frame, currRollAngle, currPitchAngle);
        cv::resize(frame, dispFrame, cv::Size(1280,720));

        // trackFrame = frame.clone();
        // detFrame = frame.clone();
        // dispFrame = frame.clone();

        // printf("=====nframe:%d======\n", nFrames);

        // if(stSysStatus.detOn)
        // {
        // 	detector->process(dispFrame, boxs);
        // }

        // if(stSysStatus.trackOn)
        // {
        // 	cv::Rect initRect = cv::Rect{(1280-stSysStatus.trackerGateSize)/2, (720-stSysStatus.trackerGateSize)/2, stSysStatus.trackerGateSize, stSysStatus.trackerGateSize};
        // 	if(!stSysStatus.trackerInited)
        // 	{
        // 		tracker.init(initRect, dispFrame);
        // 		stSysStatus.trackerInited = true;
        // 	}
        // 	else
        // 	{
        // 		result = tracker.update(dispFrame);
        // 		rectangle(dispFrame, cv::Point( result.x, result.y ), cv::Point( result.x+result.width, result.y+result.height), cv::Scalar( 0,0,255 ), 2, 8 );
        // 	}
        // }

        // if (nFrames == 0) {
        // 	while(1)
        // 	{
        // 		printf("1111111111\n");
        // 		auto tmpmat = frame.clone();
        // 		cv::rectangle( tmpmat, cv::Point(box.x,box.y), cv::Point(box.x+box.width,box.y+box.height), cv::Scalar( 48,48,255 ), 2, 8 );
        // 		cv::imshow("show", tmpmat);
        // 		if(box_complete == true)
        // 		{
        // 			xMin = box.x;
        // 			yMin = box.y;
        // 			width = box.width;
        // 			height = box.height;
        // 			break;
        // 		}
        // 		cv::waitKey(30);
        // 	}
        // 	printf("WWWWWWW\n");
        // 	tracker.init( cv::Rect(xMin, yMin, width, height), frame );
        // 	// rectangle( frame, Point( xMin, yMin ), Point( xMin+width, yMin+height), Scalar( 0, 255, 255 ), 1, 8 );
        // 	// resultsFile << xMin << "," << yMin << "," << width << "," << height << endl;
        // }
        // // Update
        // else{
        // 	result = tracker.update(frame);
        // 	// drawCrosshair(frame, cv::Point(result.x+result.width/2,result.y+result.height/2), 0.5);
        // 	rectangle( dispFrame, cv::Point( result.x, result.y ), cv::Point( result.x+result.width, result.y+result.height), cv::Scalar( 0,0,255 ), 2, 8 );
        // 	// resultsFile << result.x << "," << result.y << "," << result.width << "," << result.height << endl;
        // }

        // nFrames++;

        // // frameInfo.m_frames[0].GetMatBGRWrite() = dispFrame.clone();
        // detector->process(dispFrame, boxs);
        // detector->process(detFrame, boxs);

        // auto detret = dispFrame = detFrame.clone();

//*******************************multitracker process*************************

        // frameInfo.CleanRegions();

        // printf("frameInfo.m_regions[0] size%d, boxs :%d\n", frameInfo.m_regions[0].size(), boxs.size());
        // for(auto& box:boxs)
        // {
        // 	printf("box-->x:%d, y:%d, w:%d, h:%d\n", box.x, box.y, box.w, box.h);
        // }
        // regions.clear();
        // for(auto &box:boxs)
        // {
        //     regions.emplace_back(cv::Rect(cvRound(1.0*box.x), cvRound(1.0*box.y), cvRound(1.0*box.w), cvRound(1.0*box.h)), (box.obj_id), box.prob);
        // }

        // printf("frameInfo.m_regions[0] size%d, regions:%d\n", frameInfo.m_regions[0].size(), regions.size());
        // frameInfo.m_regions[0] = regions;
        // mtracker->Update(frameInfo.m_regions[0], frameInfo.m_frames[0].GetUMatGray(), m_fps);
        // printf("track size:%d\n", frameInfo.m_tracks[0].size());
        // mtracker->GetTracks(frameInfo.m_tracks[0]);

        // DrawData(frameInfo.m_frames[0].GetMatBGR(), frameInfo.m_tracks[0], frameInfo.m_frameInds[0], 0);
        // frame = frameInfo.m_frames[0].GetMatBGR().clone();
        // dispFrame = frameInfo.m_frames[0].GetMatBGR();

			// printf("size:%d, id:%d\n", frameInfo.m_tracks[0].size(), frameInfo.m_tracks[0][0].m_ID);

        // Tracks2Boxs(frameInfo.m_tracks[0], boxs);
        
//*******************************multitracker process end *************************

        //To do, draw OSD

        // cv::resize(dispFrame, dispFrame, cv::Size(960,540));
        // cv::imshow("show", dispFrame);
        encoder->process(dispFrame);
        // cv::imshow("det", detret);
        // cv::imwrite("1.png", frame);
        cv::waitKey(10);

    }
    // cv::Mat img = cv::imread("/space/data/0211/2-755.png");

    // detector->process(img);
    // cv::imshow("1", img);
    // cv::waitKey(0);

    return 0;
}