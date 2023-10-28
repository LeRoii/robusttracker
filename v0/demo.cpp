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

void onmouse(int event, int x, int y, int flag, void*)//鼠标事件回调函数，鼠标点击后执行的内容应在此
{
	// cv::Mat& image = *(cv::Mat*) img;
	switch (event)
	{
	case cv::EVENT_LBUTTONDOWN://鼠标左键按下事件
		drawing_box = true;//标志在画框
		box = cv::Rect(x, y, 0, 0);//记录矩形的开始的点
        userPt.x = x;	//center point
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

inline double getDistance (cv::Point point1, cv::Point point2)
{
    return  sqrtf(powf((point1.x - point2.x),2) + powf((point1.y - point2.y),2));
}

int main()
{
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
	int trackOn = config["track"].as<int>();
	std::string engine = config["engine"].as<std::string>();
	std::string videopath = config["videopath"].as<std::string>();

	realtracker *rtracker = new realtracker(engine);

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
    // KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

    cv::Mat frame;
    int nFrames = 0;

    float xMin;
	float yMin;
	float width;
	float height;

    cv::namedWindow("show");
	cv::setMouseCallback("show", onmouse);

    // Tracker results
	cv::Rect result;

	cv::Mat dispFrame, trackFrame, trackRet, detFrame, trackRetByDet;

    while(1)
    {
        // cap >> frame;
        // if(frame.empty())
        //     break;

		frame = cv::imread("/space/data/bdd1.jpg");
		cv::resize(frame, frame, cv::Size(1280,720));

		

		trackFrame = frame.clone();
		detFrame = frame.clone();
		dispFrame = frame.clone();
		trackRetByDet = frame.clone();

		printf("=====nframe:%d======\n", nFrames);

		if(trackOn)
		{
			if (nFrames == 0) {
				while(1)
				{
					printf("1111111111\n");
					auto tmpmat = frame.clone();
					cv::rectangle( tmpmat, cv::Point(box.x,box.y), cv::Point(box.x+box.width,box.y+box.height), cv::Scalar( 48,48,255 ), 2, 8 );
					cv::imshow("show", tmpmat);
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
				rtracker->init( cv::Rect(xMin-GateSize/2, yMin-GateSize/2, GateSize, GateSize), frame );
				// rectangle( frame, Point( xMin, yMin ), Point( xMin+width, yMin+height), Scalar( 0, 255, 255 ), 1, 8 );
				// resultsFile << xMin << "," << yMin << "," << width << "," << height << endl;

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

				rtracker->runTracker(trackFrame);

				// spdlog::debug("tracker lost:{}", lost);

				// if(lost || !contain)
				// {
				// 	spdlog::debug("reset tracker ");
				// 	//reset tracker
				// 	cv::Rect brect = frameInfo.m_tracks[0][minIdx].m_rrect.boundingRect();
				// 	cv::Point center{brect.tl().x + brect.width/2, brect.tl().y + brect.height/2};
				// 	tracker->reset();
				// 	tracker->init( cv::Rect(center.x-GateSize/2, center.y-GateSize/2, GateSize, GateSize), frame );
				// }
			}

			spdlog::debug("KCF end");
		}
		nFrames++;

		if(detOn)
		{
			// frameInfo.m_frames[0].GetMatBGRWrite() = dispFrame.clone();
			// detector->process(detFrame, boxs);

			// // detret = detFrame.clone();

			// frameInfo.CleanRegions();

			// // printf("frameInfo.m_regions[0] size%d, boxs :%d\n", frameInfo.m_regions[0].size(), boxs.size());
			// // for(auto& box:boxs)
			// // {
			// // 	printf("box-->x:%d, y:%d, w:%d, h:%d\n", box.x, box.y, box.w, box.h);
			// // }
			// regions.clear();
			// for(auto &box:boxs)
			// {
			// 	regions.emplace_back(cv::Rect(cvRound(1.0*box.x), cvRound(1.0*box.y), cvRound(1.0*box.w), cvRound(1.0*box.h)), (box.obj_id), box.prob);
			// }

			// printf("frameInfo.m_regions[0] size%d, regions:%d\n", frameInfo.m_regions[0].size(), regions.size());
			// frameInfo.m_regions[0] = regions;
			// mtracker->Update(frameInfo.m_regions[0], frameInfo.m_frames[0].GetUMatGray(), m_fps);
			// printf("track size:%d\n", frameInfo.m_tracks[0].size());
			// mtracker->GetTracks(frameInfo.m_tracks[0]);

			// DrawData(frameInfo.m_frames[0].GetMatBGR(), frameInfo.m_tracks[0], frameInfo.m_frameInds[0], 0);
			// // frame = frameInfo.m_frames[0].GetMatBGR().clone();
			// dispFrame = frameInfo.m_frames[0].GetMatBGR();

			// // printf("size:%d, id:%d\n", frameInfo.m_tracks[0].size(), frameInfo.m_tracks[0][0].m_ID);

			// // Tracks2Boxs(frameInfo.m_tracks[0], boxs);

			// double minDist = 10000.f;
			// for(int i=0; i< frameInfo.m_tracks[0].size(); ++i)
			// {
			// 	cv::Rect brect = frameInfo.m_tracks[0][i].m_rrect.boundingRect();
			// 	cv::Point center{brect.tl().x + brect.width/2, brect.tl().y + brect.height/2};
			// 	double dist = getDistance(userPt, center);
			// 	// printf("obj pos:(%d, %d), dist:%f\n", brect.tl().x, brect.tl().y, dist);
			// 	if(dist < minDist)
			// 	{
			// 		minDist = dist;
			// 		minIdx = i;
			// 	}
			// }

			// spdlog::debug("best det ret:id:{}, dist:{}", frameInfo.m_tracks[0][minIdx].m_ID.ID2Str(), minDist);

			// if(minIdx != -1)
			// {
			// 	cv::Point2f rectPoints[4];
			// 	frameInfo.m_tracks[0][minIdx].m_rrect.points(rectPoints);
			// 	for (int i = 0; i < 4; ++i)
			// 	{
			// 		cv::line(trackRetByDet, rectPoints[i], rectPoints[(i+1) % 4], cv::Scalar(255, 0, 255), 2);
			// 	}

			// 	contain = frameInfo.m_tracks[0][minIdx].m_rrect.boundingRect().contains(userPt);
			// 	spdlog::debug("contains:{}", contain);
			// }


			// cv::circle(trackRetByDet, userPt, 2, (0,255, 255), 2);

			std::vector<TrackingObject> detRet;
			rtracker->runDetector(detFrame);
			// cv::imshow("show", trackRetByDet);

			cv::resize(dispFrame, dispFrame, cv::Size(1280,720));
			cv::imshow("raw-detRet", detFrame);
			cv::imshow("final-detRet", dispFrame);
		}


		// cv::resize(dispFrame, dispFrame, cv::Size(1280,720));
		cv::resize(trackFrame, trackFrame, cv::Size(1280,720));
        // cv::imshow("show", dispFrame);
        cv::imshow("trackRet", trackFrame);
        
        cv::waitKey(0);

    }
    // cv::Mat img = cv::imread("/space/data/0211/2-755.png");


    // detector->process(img);
    // cv::imshow("1", img);
    // cv::waitKey(0);

    return 0;
}