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
static int m_gateSm = 0;
static int m_gateW = 32;
static int m_gateH = 32;


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
                if(m_gateSm == 0)
                {
                    rtracker->init( userPt, dispFrame, dispFrame);
                    cv::rectangle(trackFrame, cv::Rect(userPt.x - gateS/2, userPt.y - gateS/2, gateS,gateS),cv::Scalar( 48,48,255 ), 2, 8 );
                }
                else if(m_gateSm == 1)
                {
                    cv::Rect rect = cv::Rect(userPt.x - m_gateW/2, userPt.y - m_gateH/2, m_gateW, m_gateH);
                    rtracker->init( rect, dispFrame);
                    cv::rectangle(trackFrame, rect,cv::Scalar( 48,48,255 ), 2, 8 );
                }
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

std::string extractFileName(const std::string& filePath) {
    size_t lastSlashPos = filePath.find_last_of("/\\");
    
    if (lastSlashPos == std::string::npos) {
        return filePath; 
    }
    
    return filePath.substr(lastSlashPos + 1);
}

int main(int argc, char*argv[])
{
    if (setenv("DISPLAY", "192.168.4.1:0.0", 1) != 0) {
        std::cerr << "Failed to set DISPLAY environment variable." << std::endl;
        return 1;
    }

    int waitVAL =  argc > 1 ? 10 : 0;
    
    spdlog::set_level(spdlog::level::debug); // Set global log level to debug

    YAML::Node config = YAML::LoadFile("/home/rpdzkj/robusttracker-algodev/v0/config.yaml");
    int detOn = config["detection"].as<int>();
    trackOn = config["track"].as<int>();
    std::string engine = config["engine"].as<std::string>();
    std::string videopath = config["videopath"].as<std::string>();
    std::string irEngine = config["irengine"].as<std::string>();

    m_gateSm = config["gateSizeMode"].as<int>();
    m_gateW = config["gateW"].as<int>();
    m_gateH = config["gateH"].as<int>();

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
        {
            printf("empty\n");
            video.release();
            printf("return\n");
            return 0;
        }

        // frame = cv::imread("/space/data/123.PNG");
        cv::resize(frame, frame, cv::Size(1280,720));
        // cv::resize(frame, frame, cv::Size(3840,2160));

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

                // cv::resize(trackFrame, trackFrame, cv::Size(640,360));
            }

            cv::putText(trackFrame, std::to_string(nFrames), cv::Point(150, 100), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,255), 2, cv::LINE_AA);

            
            
            if(recvid)
            {
                video.write(trackFrame);
            }
            else
            {
                cv::imshow("trackRet", trackFrame);
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
            video.open(extractFileName(videopath) + ".avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),30.0, cv::Size(1280,720));
            spdlog::debug("video record start");
            waitVAL = 1;
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
/*
Thanks Nghia Ho for his excellent code.
And,I modified the smooth step using a simple kalman filter .
So,It can processes live video streaming.
modified by chen jia.
email:chenjia2013@foxmail.com
*/

// #include <opencv2/opencv.hpp>
// #include <iostream>
// #include <cassert>
// #include <cmath>
// #include <fstream>

// using namespace std;
// using namespace cv;

// // This video stablisation smooths the global trajectory using a sliding average window

// //const int SMOOTHING_RADIUS = 15; // In frames. The larger the more stable the video, but less reactive to sudden panning
// const int HORIZONTAL_BORDER_CROP = 20; // In pixels. Crops the border to reduce the black borders from stabilisation being too noticeable.

// // 1. Get previous to current frame transformation (dx, dy, da) for all frames
// // 2. Accumulate the transformations to get the image trajectory
// // 3. Smooth out the trajectory using an averaging window
// // 4. Generate new set of previous to current transform, such that the trajectory ends up being the same as the smoothed trajectory
// // 5. Apply the new transformation to the video

// struct TransformParam
// {
//     TransformParam() {}
//     TransformParam(double _dx, double _dy, double _da) {
//         dx = _dx;
//         dy = _dy;
//         da = _da;
//     }

//     double dx;
//     double dy;
//     double da; // angle
// };

// struct Trajectory
// {
//     Trajectory() {}
//     Trajectory(double _x, double _y, double _a) {
//         x = _x;
//         y = _y;
//         a = _a;
//     }
// 	// "+"
// 	friend Trajectory operator+(const Trajectory &c1,const Trajectory  &c2){
// 		return Trajectory(c1.x+c2.x,c1.y+c2.y,c1.a+c2.a);
// 	}
// 	//"-"
// 	friend Trajectory operator-(const Trajectory &c1,const Trajectory  &c2){
// 		return Trajectory(c1.x-c2.x,c1.y-c2.y,c1.a-c2.a);
// 	}
// 	//"*"
// 	friend Trajectory operator*(const Trajectory &c1,const Trajectory  &c2){
// 		return Trajectory(c1.x*c2.x,c1.y*c2.y,c1.a*c2.a);
// 	}
// 	//"/"
// 	friend Trajectory operator/(const Trajectory &c1,const Trajectory  &c2){
// 		return Trajectory(c1.x/c2.x,c1.y/c2.y,c1.a/c2.a);
// 	}
// 	//"="
// 	Trajectory operator =(const Trajectory &rx){
// 		x = rx.x;
// 		y = rx.y;
// 		a = rx.a;
// 		return Trajectory(x,y,a);
// 	}

//     double x;
//     double y;
//     double a; // angle
// };
// //
// int main(int argc, char **argv)
// {
// 	if(argc < 2) {
// 		cout << "./VideoStab [video.avi]" << endl;
// 		return 0;
// 	}
// 	// For further analysis
// 	ofstream out_transform("prev_to_cur_transformation.txt");
// 	ofstream out_trajectory("trajectory.txt");
// 	ofstream out_smoothed_trajectory("smoothed_trajectory.txt");
// 	ofstream out_new_transform("new_prev_to_cur_transformation.txt");

// 	VideoCapture cap(argv[1]);
// 	assert(cap.isOpened());

// 	Mat cur, cur_grey;
// 	Mat prev, prev_grey;

// 	cap >> prev;//get the first frame.ch
// 	cvtColor(prev, prev_grey, COLOR_BGR2GRAY);
	
// 	// Step 1 - Get previous to current frame transformation (dx, dy, da) for all frames
// 	vector <TransformParam> prev_to_cur_transform; // previous to current
// 	// Accumulated frame to frame transform
// 	double a = 0;
// 	double x = 0;
// 	double y = 0;
// 	// Step 2 - Accumulate the transformations to get the image trajectory
// 	vector <Trajectory> trajectory; // trajectory at all frames
// 	//
// 	// Step 3 - Smooth out the trajectory using an averaging window
// 	vector <Trajectory> smoothed_trajectory; // trajectory at all frames
// 	Trajectory X;//posteriori state estimate
// 	Trajectory	X_;//priori estimate
// 	Trajectory P;// posteriori estimate error covariance
// 	Trajectory P_;// priori estimate error covariance
// 	Trajectory K;//gain
// 	Trajectory	z;//actual measurement
// 	double pstd = 4e-3;//can be changed
// 	double cstd = 0.25;//can be changed
// 	Trajectory Q(pstd,pstd,pstd);// process noise covariance
// 	Trajectory R(cstd,cstd,cstd);// measurement noise covariance 
// 	// Step 4 - Generate new set of previous to current transform, such that the trajectory ends up being the same as the smoothed trajectory
// 	vector <TransformParam> new_prev_to_cur_transform;
// 	//
// 	// Step 5 - Apply the new transformation to the video
// 	//cap.set(CV_CAP_PROP_POS_FRAMES, 0);
// 	Mat T(2,3,CV_64F);

// 	int vert_border = HORIZONTAL_BORDER_CROP * prev.rows / prev.cols; // get the aspect ratio correct
// 	// VideoWriter outputVideo; 
// 	// outputVideo.open("compare.avi" , CV_FOURCC('X','V','I','D'), 24,cvSize(cur.rows, cur.cols*2+10), true);  
// 	//
// 	int k=1;
// 	int max_frames;// = cap.get(CV_CAP_PROP_FRAME_COUNT);
// 	Mat last_T;
// 	Mat prev_grey_,cur_grey_;
	 
// 	while(true) {

// 		cap >> cur;
// 		if(cur.data == NULL) {
// 			break;
// 		}

// 		cvtColor(cur, cur_grey, COLOR_BGR2GRAY);

// 		// vector from prev to cur
// 		vector <Point2f> prev_corner, cur_corner;
// 		vector <Point2f> prev_corner2, cur_corner2;
// 		vector <uchar> status;
// 		vector <float> err;

// 		goodFeaturesToTrack(prev_grey, prev_corner, 200, 0.01, 30);
// 		calcOpticalFlowPyrLK(prev_grey, cur_grey, prev_corner, cur_corner, status, err);

// 		// weed out bad matches
// 		for(size_t i=0; i < status.size(); i++) {
// 			if(status[i]) {
// 				prev_corner2.push_back(prev_corner[i]);
// 				cur_corner2.push_back(cur_corner[i]);
// 			}
// 		}

// 		// translation + rotation only
// 		Mat T = estimateRigidTransform(prev_corner2, cur_corner2, false); // false = rigid transform, no scaling/shearing

// 		// in rare cases no transform is found. We'll just use the last known good transform.
// 		if(T.data == NULL) {
// 			last_T.copyTo(T);
// 		}

// 		T.copyTo(last_T);

// 		// decompose T
// 		double dx = T.at<double>(0,2);
// 		double dy = T.at<double>(1,2);
// 		double da = atan2(T.at<double>(1,0), T.at<double>(0,0));
// 		//
// 		//prev_to_cur_transform.push_back(TransformParam(dx, dy, da));

// 		out_transform << k << " " << dx << " " << dy << " " << da << endl;
// 		//
// 		// Accumulated frame to frame transform
// 		x += dx;
// 		y += dy;
// 		a += da;
// 		//trajectory.push_back(Trajectory(x,y,a));
// 		//
// 		out_trajectory << k << " " << x << " " << y << " " << a << endl;
// 		//
// 		z = Trajectory(x,y,a);
// 		//
// 		if(k==1){
// 			// intial guesses
// 			X = Trajectory(0,0,0); //Initial estimate,  set 0
// 			P =Trajectory(1,1,1); //set error variance,set 1
// 		}
// 		else
// 		{
// 			//time update（prediction）
// 			X_ = X; //X_(k) = X(k-1);
// 			P_ = P+Q; //P_(k) = P(k-1)+Q;
// 			// measurement update（correction）
// 			K = P_/( P_+R ); //gain;K(k) = P_(k)/( P_(k)+R );
// 			X = X_+K*(z-X_); //z-X_ is residual,X(k) = X_(k)+K(k)*(z(k)-X_(k)); 
// 			P = (Trajectory(1,1,1)-K)*P_; //P(k) = (1-K(k))*P_(k);
// 		}
// 		//smoothed_trajectory.push_back(X);
// 		out_smoothed_trajectory << k << " " << X.x << " " << X.y << " " << X.a << endl;
// 		//-
// 		// target - current
// 		double diff_x = X.x - x;//
// 		double diff_y = X.y - y;
// 		double diff_a = X.a - a;

// 		dx = dx + diff_x;
// 		dy = dy + diff_y;
// 		da = da + diff_a;

// 		//new_prev_to_cur_transform.push_back(TransformParam(dx, dy, da));
// 		//
// 		out_new_transform << k << " " << dx << " " << dy << " " << da << endl;
// 		//
// 		T.at<double>(0,0) = cos(da);
// 		T.at<double>(0,1) = -sin(da);
// 		T.at<double>(1,0) = sin(da);
// 		T.at<double>(1,1) = cos(da);

// 		T.at<double>(0,2) = dx;
// 		T.at<double>(1,2) = dy;

// 		Mat cur2;
		
// 		warpAffine(prev, cur2, T, cur.size());

// 		cur2 = cur2(Range(vert_border, cur2.rows-vert_border), Range(HORIZONTAL_BORDER_CROP, cur2.cols-HORIZONTAL_BORDER_CROP));

// 		// Resize cur2 back to cur size, for better side by side comparison
// 		resize(cur2, cur2, cur.size());

// 		// Now draw the original and stablised side by side for coolness
// 		Mat canvas = Mat::zeros(cur.rows, cur.cols*2+10, cur.type());

// 		prev.copyTo(canvas(Range::all(), Range(0, cur2.cols)));
// 		cur2.copyTo(canvas(Range::all(), Range(cur2.cols+10, cur2.cols*2+10)));

// 		// If too big to fit on the screen, then scale it down by 2, hopefully it'll fit :)
// 		if(canvas.cols > 1920) {
// 			resize(canvas, canvas, Size(canvas.cols/2, canvas.rows/2));
// 		}
// 		//outputVideo<<canvas;
// 		imshow("before and after", canvas);

// 		waitKey(10);
// 		//
// 		prev = cur.clone();//cur.copyTo(prev);
// 		cur_grey.copyTo(prev_grey);

// 		cout << "Frame: " << k << "/" << max_frames << " - good optical flow: " << prev_corner2.size() << endl;
// 		k++;

// 	}
// 	return 0;
// }