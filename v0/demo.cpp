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
#include <yaml-cpp/yaml.h>
#include "realtracker.h"


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

    std::vector<bbox_t> boxs;

    cv::Mat frame,frame0,frame1;
    int nFrames = 0;

    cv::Mat dispFrame, trackFrame, detFrame;
	cv::Mat oriIrImg, viImg, irImg;

    irImg = cv::Mat(720, 1280, CV_8UC3);
    irImg.setTo(0);

	stSysStatus.enDispMode = Vision;
    stSysStatus.trackOn = false;
    stSysStatus.detOn = true;

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


    YAML::Node config = YAML::LoadFile("../config.yaml");
	std::string engine = config["engine"].as<std::string>();
    realtracker *rtracker = new realtracker(engine);

    std::vector<TrackingObject> detRet;


    cv::VideoWriter writer = new cv::VideoWriter(filePath, CV_FOURCC('M','J','P','G'),  20, cv::Size(1920, 1080));

    while(!quit)
    {
        // usleep(1000000);
        // continue;
		printf("while\n");
        cam->GetFrame(viImg, oriIrImg);

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

        // trackFrame = frame.clone();
        // detFrame = frame.clone();
        // dispFrame = frame.clone();

        printf("=====nframe:%d======\n", nFrames);

        if(stSysStatus.detOn)
        {
            rtracker->runDetector(frame, detRet);
        }

        if(stSysStatus.trackOn)
        {
        	cv::Rect initRect = cv::Rect{(1280-stSysStatus.trackerGateSize)/2, (720-stSysStatus.trackerGateSize)/2, stSysStatus.trackerGateSize, stSysStatus.trackerGateSize};
        	if(!stSysStatus.trackerInited)
        	{
        		// tracker.init(initRect, dispFrame);
                rtracker->init(initRect, frame );
        		stSysStatus.trackerInited = true;
        	}
        	else
        	{
                rtracker->runTracker(frame);
        	}
        }

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

        nFrames++;

        cv::imshow("show", dispFrame);
        encoder->process(dispFrame);
        // cv::imshow("det", detret);
        // cv::imwrite("1.png", frame);
        cv::waitKey(30);
        // usleep(30000);

    }

    return 0;
}