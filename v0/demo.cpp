#include "idetector.h"
#include "kcftracker.hpp"
#include "multitracker.h"

#include <unistd.h>
#include <signal.h>
#include "jetsonEncoder.h"
#include "serial.h"
#include "common.h"
#include <sstream>
#include <cmath>
#include "camera.h"
#include "painter.h"
#include<arpa/inet.h>

const int DEBUG_SERIAL = 0;

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
extern ST_T1F1B1D1_CONFIG stT1F1B1D1Cfg;
extern ST_T2F2B2D2_CONFIG stT2F2B2D2Cfg;

ST_SYS_STATUS stSysStatus;

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
#if DEBUG_SERIAL
            for(int i=0; i< retLen ;i++)
            {
            	printf("[%02X]", buffRcvData_servo[i]);
            }
            printf("\n");
#endif

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
            else if(frameType == HeartBeat14)
            {
                printf("heart beat 14 from up serial\n\n");
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
#if DEBUG_SERIAL
            for(int i=0; i< retLen ;i++)
            {
            	printf("[%02X]", buffRcvData_servo[i]);
            }
            printf("\n");
#endif

            retLen = serialUp.serial_send(buffRcvData_servo, retLen);
            printf("down send to up:%d\n", retLen);

            int rr = serialDown.ProcessSerialData(buffRcvData_servo, retLen, output, outLen);

            if(rr == RET_ERR)
            {
                printf("RET_ERR\n");
                continue;
            }

            printf("output buf\n");
            for(int i=0; i< outLen ;i++)
            {
                printf("[%02X]", output[i]);
            }
            printf("\n");

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

            VL_ParseSerialData(output);

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

static void cvtIrImg(cv::Mat &img, EN_IRIMG_MODE mode)
{
    
    if(mode == EN_IRIMG_MODE::BLACKHOT)
    {
        cv::cvtColor(img, img, cv::COLOR_RGB2GRAY);
        img = 255 - img;
        cv::cvtColor(img, img, cv::COLOR_GRAY2RGB);
    }
    else if(mode == EN_IRIMG_MODE::PSEUDOCOLOR)
    {
        cv::applyColorMap(img, img, cv::COLORMAP_HOT);
    }
}

int main()
{
//*******************************read config *************************

    // YAML::Node config = YAML::LoadFile("../config.yaml");
	// int detOn = config["detection"].as<int>();
	// int trackOn = config["track"].as<int>();

    // std::string inputVideoType = config["inputVideoType"].as<string>();;
    // std::string irStreamAdd = config["irStreamAdd"].as<string>();
    // std::string visStreamAdd = config["visStreamAdd"].as<string>();
    

//*******************************read config end *************************


//*******************************serial init*************************
    
    serialUp.set_serial(1);    //"/dev/ttyTHS1"
    // serialUp.OnStart();
    serialDown.set_serial(2);    //"/dev/ttyUSB0"
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


	// cv::VideoCapture IRCamera1("rtspsrc location=rtsp://192.168.2.119:554/live1 latency=0 ! rtph264depay ! h264parse ! omxh264dec ! videoconvert ! appsink max-buffers=1 drop=true sync=false", cv::CAP_GSTREAMER);
	// cv::VideoCapture IrCam("rtsp://192.168.2.119:554/live2");
	// cv::VideoCapture ViCam("rtsp://192.168.2.119:554/live1");
	// cv::VideoCapture IrCam("rtsp://192.168.168.119:554/stream1");
	// cv::VideoCapture ViCam("rtsp://192.168.168.119:554/stream0");

	cv::Mat oriIrImg, viImg, irImg;

    irImg = cv::Mat(720, 1280, CV_8UC3);
    irImg.setTo(0);

	stSysStatus.enDispMode = Vision;

    int pipPosX, pipPosY;

    Camera *cam = CreateCamera("../config.yaml");

    if(cam != nullptr)
    {
        cam->Init();
    }
    else
    {
        printf("camera inti failed\n");
        return 0;
    }

    cam->GetFrame(viImg, oriIrImg);
    if(oriIrImg.empty() || viImg.empty())
    {
        printf("input img empty, quit\n");
        return 0;
    }

    pipPosX = (viImg.cols - oriIrImg.cols)/2;
    pipPosY = (viImg.rows - oriIrImg.rows)/2;


    while(!quit)
    {

        // usleep(1000000);
        // continue;
		// printf("while\n");
        cam->GetFrame(viImg, oriIrImg);
        // IrCam >> oriIrImg;
        // ViCam >> viImg;

        

        if(oriIrImg.empty() || viImg.empty())
        {
            printf("input img empty, quit\n");
        }

        cvtIrImg(oriIrImg, stSysStatus.enIrImgMode);

        // printf("oriIrImg w:%d, oriIrImg h:%d\n", oriIrImg.cols, oriIrImg.rows);
        // printf("viImg w:%d, viImg h:%d\n", viImg.cols, viImg.rows);

        irImg.setTo(0);
        oriIrImg.copyTo(irImg(cv::Rect(pipPosX, pipPosY, oriIrImg.cols, oriIrImg.rows)));

        // printf("irImg w:%d, irImg h:%d\n", irImg.cols, irImg.rows);
        // printf("viImg w:%d, viImg h:%d\n", viImg.cols, viImg.rows);

		switch(stSysStatus.enDispMode)
		{
			case Vision:    //0x01
				frame = viImg;
				break;
			case Ir:        //0x02
				frame = irImg;
				break;
			case VisIrPip:  //0x03
				cv::resize(oriIrImg, oriIrImg, cv::Size(480, 360));
				oriIrImg.copyTo(viImg(cv::Rect(1280-480, 0, 480, 360)));
				frame = viImg;
				break;
			case IrVisPip:  //0x04
				cv::resize(viImg, viImg, cv::Size(480, 360));
				viImg.copyTo(irImg(cv::Rect(1280-480, 0, 480, 360)));
				frame = irImg;
				break;
			default:
				frame = viImg;
				break;
		}

        if(frame.empty())
		{
			printf("empty frame\n");
            break;
		}

        // Sdireader_GetFrame(frame0, frame1); 
        // frame = frame0;
        // 在界面上绘制OSD
        //float currRollAngle = -78.5; // 需要修改为吊舱返回的当前横滚角度
        PaintRollAngleAxis(frame, stSysStatus.rollAngle);

        //float currPitchAngle = 0; // 需要修改为吊舱返回的当前俯仰角度
        PaintPitchAngleAxis(frame, stSysStatus.pitchAngle);

        // 绘制中心十字
        PaintCrossPattern(frame, stSysStatus.rollAngle, stSysStatus.pitchAngle);

        // 绘制经纬度、海拔高度等坐标参数
        PaintCoordinate(frame);

        // 绘制界面上其他参数
        PaintViewPara(frame);
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

        // nFrames++;

        // frameInfo.m_frames[0].GetMatBGRWrite() = dispFrame.clone();

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
        cv::imshow("show", dispFrame);
        encoder->process(dispFrame);
        // cv::imshow("det", detret);
        // cv::imwrite("1.png", frame);
        cv::waitKey(30);
        // usleep(30000);

    }
    // cv::Mat img = cv::imread("/space/data/0211/2-755.png");

    // detector->process(img);
    // cv::imshow("1", img);
    // cv::waitKey(0);

    return 0;
}