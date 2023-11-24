#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "kcftracker.hpp"

#include <dirent.h>

#include <opencv2/highgui/highgui_c.h>

using namespace std;
using namespace cv;

cv::Rect box;//矩形对象
bool drawing_box = false;//记录是否在画矩形对象
bool box_complete = false;

void onmouse(int event, int x, int y, int flag, void*)//鼠标事件回调函数，鼠标点击后执行的内容应在此
{
	// cv::Mat& image = *(cv::Mat*) img;
	switch (event)
	{
	case CV_EVENT_LBUTTONDOWN://鼠标左键按下事件
		drawing_box = true;//标志在画框
		box = Rect(x, y, 0, 0);//记录矩形的开始的点
		break;
	case CV_EVENT_MOUSEMOVE://鼠标移动事件
		if (drawing_box) {//如果左键一直按着，则表明在画矩形
			box.width = x - box.x;
			box.height = y - box.y;//更新长宽
		}
		break;
	case CV_EVENT_LBUTTONUP://鼠标左键松开事件
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

void drawCrosshair(cv::Mat &frame, cv::Point pt, double scale = 1)
{
	int crosshairW = 150;
	int crosshairH = 100;
	auto imgi = cv::Mat(crosshairH, crosshairW, CV_8UC3);
	imgi.setTo(0);

	int w = imgi.cols;
	int h = imgi.rows;

	auto centerPt = cv::Point(w/2, h/2);

	auto color = cv::Scalar(0,255,0);
	auto thickness = 4;
	cv::circle(imgi, centerPt, 1, color, 3);

	cv::Point leftPtStart = cv::Point(0, centerPt.y);
	cv::Point leftPtEnd = cv::Point(0 + w/2 - 10, centerPt.y);

	cv::Point upPtStart = cv::Point(centerPt.x, 0);
	cv::Point upPtEnd = cv::Point(centerPt.x, h/2 - 10);

	cv::Point rightPtStart = cv::Point(w/2 + 10, centerPt.y);
	cv::Point rightPtEnd = cv::Point(w, centerPt.y);

	cv::Point downPtStart = cv::Point(centerPt.x, h/2 + 10);
	cv::Point downPtEnd = cv::Point(centerPt.x, h);

	//crosshair
	cv::line(imgi, leftPtStart, leftPtEnd, color, thickness);
	cv::line(imgi, upPtStart, upPtEnd, color, thickness);
	cv::line(imgi, rightPtStart, rightPtEnd, color, thickness);
	cv::line(imgi, downPtStart, downPtEnd, color, thickness);


	int cornerW = 30;
	int cornerH = 35;
	int leftupX = 20;
	int leftupY = 10;
	int leftdownX = leftupX;
	int leftdownY = h - leftupY;
	int rightupX = w - leftupX;
	int rightupY = leftupY;
	int rightdownX = rightupX;
	int rightdownY = leftdownY;
	
	//left up corner
	cv::line(imgi, cv::Point(leftupX,leftupY), cv::Point(leftupX, leftupY + cornerH), color, thickness);
	cv::line(imgi, cv::Point(leftupX,leftupY), cv::Point(leftupX + cornerW,leftupY), color, thickness);

	//left down corner
	cv::line(imgi, cv::Point(leftdownX, leftdownY), cv::Point(leftdownX, leftdownY - cornerH), color, thickness);
	cv::line(imgi, cv::Point(leftdownX, leftdownY), cv::Point(leftdownX + cornerW, leftdownY), color, thickness);

	//right up corner
	cv::line(imgi, cv::Point(rightupX,rightupY), cv::Point(rightupX - cornerW, rightupY), color, thickness);
	cv::line(imgi, cv::Point(rightupX,rightupY), cv::Point(rightupX, rightupY + cornerH), color, thickness);

	//right down corner
	cv::line(imgi, cv::Point(rightdownX,rightdownY), cv::Point(rightdownX - cornerW, rightdownY), color, thickness);
	cv::line(imgi, cv::Point(rightdownX,rightdownY), cv::Point(rightdownX, rightdownY - cornerH), color, thickness);

	cv::Mat mask;// = imgi.clone();
	// mask.setTo(1);

	cv::Mat resizedTemplate;
	cv::resize(imgi, resizedTemplate, cv::Size(), scale, scale);

	printf("w:%d,h:%d\n", resizedTemplate.cols, resizedTemplate.rows);
	// cv::imwrite("111.png", resizedTemplate);
	mask = resizedTemplate.clone();
	if(pt.x <= resizedTemplate.cols/2)
		pt.x = resizedTemplate.cols/2;
	if(pt.y <= resizedTemplate.rows/2)
		pt.y = resizedTemplate.rows/2;
	if(pt.x >= (frame.cols - resizedTemplate.cols/2))
		pt.x = frame.cols - resizedTemplate.cols/2 - 1;
	if(pt.y >= (frame.rows - resizedTemplate.rows/2))
		pt.y = frame.rows - resizedTemplate.rows/2 - 1;
	resizedTemplate.copyTo(frame(cv::Rect(pt.x - resizedTemplate.cols/2, pt.y - resizedTemplate.rows/2, resizedTemplate.cols, resizedTemplate.rows)), mask);
	
}


std::string imgpath = "/home/u20/data/16/16/";
int i = 0;
int main(int argc, char* argv[]){

	// auto imgi = cv::Mat(1080, 1920, CV_8UC3);
	// imgi.setTo(255);
	// rectangle( imgi, Point(959,539), Point(960,540), Scalar(0,0,0));
	// cv::imwrite("123.png", imgi);
	// return 0;

	// auto img = cv::imread("/home/zpwang/code/ass/i.png");
	
	// drawCrosshair(img, cv::Point(400,322), 0.3);

	// // cv::Mat ret;
	// // cv::addWeighted(img(cv::Rect(0,0,600,400)), 1, imgi, 1, 0, ret);

	

	// cv::imshow("1", img);
	// cv::waitKey(0);
	// return 0;

	if (argc > 5) return -1;

	bool HOG = false;
	bool FIXEDWINDOW = false;
	bool MULTISCALE = true;
	bool SILENT = true;
	bool LAB = false;

	for(int i = 0; i < argc; i++){
		if ( strcmp (argv[i], "hog") == 0 )
			HOG = true;
		if ( strcmp (argv[i], "fixed_window") == 0 )
			FIXEDWINDOW = true;
		if ( strcmp (argv[i], "singlescale") == 0 )
			MULTISCALE = false;
		if ( strcmp (argv[i], "show") == 0 )
			SILENT = false;
		if ( strcmp (argv[i], "lab") == 0 ){
			LAB = true;
			HOG = true;
		}
		if ( strcmp (argv[i], "gray") == 0 )
			HOG = false;
	}
	
	// Create KCFTracker object
	KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

	// Frame readed
	Mat frame;

	// Tracker results
	Rect result;

	// Path to list.txt
	// ifstream listFile;
	// string fileName = "images.txt";
  	// listFile.open(fileName);

  	// // Read groundtruth for the 1st frame
  	// ifstream groundtruthFile;
	// string groundtruth = "region.txt";
  	// groundtruthFile.open(groundtruth);
  	// string firstLine;
  	// getline(groundtruthFile, firstLine);
	// groundtruthFile.close();
  	
  	// istringstream ss(firstLine);

  	// Read groundtruth like a dumb
  	float x1, y1, x2, y2, x3, y3, x4, y4;
  	// char ch;
	// ss >> x1;
	// ss >> ch;
	// ss >> y1;
	// ss >> ch;
	// ss >> x2;
	// ss >> ch;
	// ss >> y2;
	// ss >> ch;
	// ss >> x3;
	// ss >> ch;
	// ss >> y3;
	// ss >> ch;
	// ss >> x4;
	// ss >> ch;
	// ss >> y4; 

	// Using min and max of X and Y for groundtruth rectangle
	float xMin =  min(x1, min(x2, min(x3, x4)));
	float yMin =  min(y1, min(y2, min(y3, y4)));
	float width = max(x1, max(x2, max(x3, x4))) - xMin;
	float height = max(y1, max(y2, max(y3, y4))) - yMin;

	
	// Read Images
	ifstream listFramesFile;
	string listFrames = "images.txt";
	listFramesFile.open(listFrames);
	string frameName;


	// Write Results
	// ofstream resultsFile;
	string resultsPath = "output.txt";
	// resultsFile.open(resultsPath);

	// Frame counter
	int nFrames = 0;

	cv::VideoCapture capture;
	// capture.open("/home/zpwang/data/ir/REC_0027.mp4");
	// capture.open("/home/zpwang/data/ir/data9.mp4");
	// capture.open("./1.mp4");
	capture.open("/space/data/pl/IMG_3567.MOV");

	int videoW = capture.get(CAP_PROP_FRAME_WIDTH);
	int videoH = capture.get(CAP_PROP_FRAME_HEIGHT);

	videoW = 1280;
	videoH = 1024;

	cv::VideoWriter writer;

	// writer.open("trackret.avi",cv::VideoWriter::fourcc('M','J','P','G'),
	// 		20, 
	// 		Size(videoW,videoH),
	// 		true);
    // if (!writer.isOpened())
    // {
	// 	printf("writer open failed\n");
    //     return 0;
    // }

	namedWindow("show");
	setMouseCallback("show", onmouse);


	xMin = 714;
	yMin = 661;
	width = 50;
	height = 45;
	// printf("asdasdasdasd");

	// cv::Mat a = cv::imread("D:\\code\\tracking\\pysot\\demo\\1.png");
	// cv::imshow("a", a);
	// cv::waitKey(0);

	// return 0;;
	if(!capture.isOpened())
	{
		std::cout<<"isOpened false"<<std::endl;
		return 0;
	}
	box = cv::Rect(0,0,0,0);
	while (1){
		// frameName = frameName;
		capture >> frame;
		// frame = cv::imread(imgpath+std::to_string(i++)+".bmp");
		if(frame.empty())
        {
            std::cout<<"no video frame"<<std::endl;
            break;
        }

		// cv::resize(frame, frame, cv::Size(960,540));

		// Read each frame from the list
		// frame = imread(frameName);

		// First frame, give the groundtruth to the tracker
		if (nFrames == 0) {
			while(1)
			{
				printf("1111111111\n");
				auto tmpmat = frame.clone();
				cv::rectangle( tmpmat, Point(box.x,box.y), Point(box.x+box.width,box.y+box.height), Scalar( 48,48,255 ), 2, 8 );
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
			tracker.init( Rect(xMin, yMin, width, height), frame );
			// rectangle( frame, Point( xMin, yMin ), Point( xMin+width, yMin+height), Scalar( 0, 255, 255 ), 1, 8 );
			// resultsFile << xMin << "," << yMin << "," << width << "," << height << endl;
		}
		// Update
		else{
			result = tracker.update(frame);
			// drawCrosshair(frame, cv::Point(result.x+result.width/2,result.y+result.height/2), 0.5);
			rectangle( frame, Point( result.x, result.y ), Point( result.x+result.width, result.y+result.height), Scalar( 255,0,0 ), 2, 8 );
			// resultsFile << result.x << "," << result.y << "," << result.width << "," << result.height << endl;
		}

		nFrames++;

		printf("nFrames:%d\n", nFrames);

		imshow("show", frame);
		waitKey(20);
		// writer<<frame;
	}
	// writer.release();
	// resultsFile.close();

	// listFile.close();

}
