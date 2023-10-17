#include "idetector.h"
#include "kcftracker.hpp"
#include "multitracker.h"
#include "sdireader.h"
#include <unistd.h>
#include <signal.h>
#include "jetsonEncoder.h"
#include "serial.h"

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

int TrackerGateSize = 32;
bool TrackingOn = false;
bool TrackerInited = false;

static void SetTrackerGateSize()
{
	switch(stE1Cfg.enBaseOpMode)
	{
		case 0x20:
			TrackerGateSize = 16;
			break;
		case 0x22:
			TrackerGateSize = 64;
			break;
		case 0x23:
			TrackerGateSize = 128;
			break;
		default:
			TrackerGateSize = 32;
			break;
	}
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

int main()
{
//*******************************serial init*************************
	Serial serialP1;
	serialP1.set_serial(1);
	serialP1.OnStart();
//*******************************serial end*************************

	jetsonEncoder *encoder = new jetsonEncoder(8554);

	struct sigaction sig_action;
	sig_action.sa_handler = signal_handle;
	sigemptyset(&sig_action.sa_mask);
	sig_action.sa_flags = 0;
	sigaction(SIGINT, &sig_action, NULL);

    idetector *detector = new idetector("/home/nx/model/vis-8s-2c.engine");
    detector->init();
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

    cv::namedWindow("show");
	cv::setMouseCallback("show", onmouse);

    // Tracker results
	cv::Rect result;

	cv::Mat dispFrame, trackFrame, detFrame;

	Sdireader_Init("/etc/jetsoncfg/NXConfig.ini");
	// sleep(2);

    while(!quit)
    {
        // cap >> frame;
        // if(frame.empty())
        //     break;

		Sdireader_GetFrame(frame0, frame1);
		frame = frame0;
		cv::resize(frame, dispFrame, cv::Size(1280,720));

		// trackFrame = frame.clone();
		// detFrame = frame.clone();
		// dispFrame = frame.clone();

		// printf("=====nframe:%d======\n", nFrames);

		if(stE1Cfg.enBaseOpMode == AiIdentifySwitch && stE1Cfg.u8Para2 == 1)
		{
			detector->process(frame, boxs);
		}


		if(TrackingOn)
		{
			cv::Rect initRect = cv::Rect{(1280-TrackerGateSize)/2, (720-TrackerGateSize)/2, TrackerGateSize, TrackerGateSize};
			if(!TrackerInited)
			{
				tracker.init(initRect, frame);
				TrackerInited = true;
			}
			else
			{
				result = tracker.update(frame);
				rectangle(dispFrame, cv::Point( result.x, result.y ), cv::Point( result.x+result.width, result.y+result.height), cv::Scalar( 0,0,255 ), 2, 8 );
			}
		}

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
        cv::imshow("show", dispFrame);
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