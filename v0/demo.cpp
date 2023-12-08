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
cv::Mat dispFrame, trackRet, detFrame, trackRetByDet;
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
                rtracker->init( userPt, dispFrame );
                cv::rectangle(trackFrame, cv::Rect(userPt.x - 16, userPt.y - 16, 32,32),cv::Scalar( 48,48,255 ), 2, 8 );
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

// cv::Scalar CalcMSSIM(cv::Mat  inputimage1, cv::Mat inputimage2)
// {
//     cv::Mat i1 = inputimage1;
//     cv::Mat i2 = inputimage2;
//     const double C1 = 6.5025, C2 = 58.5225;
//     int d = CV_32F;
//     cv::Mat I1, I2;
//     i1.convertTo(I1, d);
//     i2.convertTo(I2, d);
//     cv::Mat I2_2 = I2.mul(I2);
//     cv::Mat I1_2 = I1.mul(I1);
//     cv::Mat I1_I2 = I1.mul(I2);
//     cv::Mat mu1, mu2;
//     GaussianBlur(I1, mu1, cv::Size(11, 11), 1.5);
//     GaussianBlur(I2, mu2, cv::Size(11, 11), 1.5);
//     cv::Mat mu1_2 = mu1.mul(mu1);
//     cv::Mat mu2_2 = mu2.mul(mu2);
//     cv::Mat mu1_mu2 = mu1.mul(mu2);
//     cv::Mat sigma1_2, sigma2_2, sigma12;
//     cv::GaussianBlur(I1_2, sigma1_2, cv::Size(11, 11), 1.5);
//     sigma1_2 -= mu1_2;
//     cv::GaussianBlur(I2_2, sigma2_2, cv::Size(11, 11), 1.5);
//     sigma2_2 -= mu2_2;
//     cv::GaussianBlur(I1_I2, sigma12, cv::Size(11, 11), 1.5);
//     sigma12 -= mu1_mu2;
//     cv::Mat t1, t2, t3;
//     t1 = 2 * mu1_mu2 + C1;
//     t2 = 2 * sigma12 + C2;
//     t3 = t1.mul(t2);
//     t1 = mu1_2 + mu2_2 + C1;
//     t2 = sigma1_2 + sigma2_2 + C2;
//     t1 = t1.mul(t2);
//     cv::Mat ssim_map;
//     divide(t3, t1, ssim_map);
//     cv::Scalar mssim = mean(ssim_map);
//     return mssim;
// }

// double calculateSSIM(const cv::Mat& img1, const cv::Mat& img2) {
//     const double C1 = 6.5025, C2 = 58.5225;

//     std::vector<cv::Mat> channels1, channels2;
//     cv::split(img1, channels1);
//     cv::split(img2, channels2);

//     double ssimTotal = 0.0;

//     for (int i = 0; i < 3; ++i) {
        
//         cv::Mat I1, I2;
//         channels1[i].convertTo(I1, CV_32F);  // Convert to floating-point
//         channels2[i].convertTo(I2, CV_32F);

//         printf("111\n");

//         // std::cout<<"I1 :"<<I1<<std::endl;
//         // std::cout<<"I2 :"<<I2<<std::endl;

//         cv::Mat I1I2, I1Sq, I2Sq;
//         cv::multiply(I1, I2, I1I2);
//         cv::multiply(I1, I1, I1Sq);
//         cv::multiply(I2, I2, I2Sq);

//         printf("111\n");

//         cv::Mat mu1, mu2;
//         cv::GaussianBlur(I1, mu1, cv::Size(11, 11), 1.5);
//         cv::GaussianBlur(I2, mu2, cv::Size(11, 11), 1.5);

//         printf("222\n");

//         cv::Mat mu1Sq, mu2Sq, mu1mu2;
//         cv::multiply(mu1, mu1, mu1Sq);
//         cv::multiply(mu2, mu2, mu2Sq);
//         cv::multiply(mu1, mu2, mu1mu2);

//         printf("333\n");

//         cv::Mat sigma1Sq, sigma2Sq, sigma12;
//         cv::GaussianBlur(I1Sq, sigma1Sq, cv::Size(11, 11), 1.5);
//         cv::GaussianBlur(I2Sq, sigma2Sq, cv::Size(11, 11), 1.5);
//         cv::GaussianBlur(I1I2, sigma12, cv::Size(11, 11), 1.5);

//         printf("444\n");

//         sigma1Sq -= mu1Sq;
//         sigma2Sq -= mu2Sq;
//         sigma12 -= mu1mu2;

//         cv::Mat ssimMap;
//         cv::divide((2 * mu1mu2 + C1) * (2 * sigma12 + C2), (mu1Sq + mu2Sq + C1) * (sigma1Sq + sigma2Sq + C2), ssimMap);

//         cv::Scalar ssim = cv::mean(ssimMap);
//         ssimTotal += ssim[0];
//     }

//     return ssimTotal / 3.0;
// }

// double calculateSSIM(const cv::Mat& img1, const cv::Mat& img2) {
//     // 分离通道
//     std::vector<cv::Mat> channels1, channels2;
//     cv::split(img1, channels1);
//     cv::split(img2, channels2);

//     double ssim = 0.0;

//     // 计算每个通道的SSIM
//     for (int i = 0; i < 3; ++i) {
//         // 转换为double类型
//         channels1[i].convertTo(channels1[i], CV_64F);
//         channels2[i].convertTo(channels2[i], CV_64F);

//         // 计算均值
//         double mean1 = cv::mean(channels1[i])[0];
//         double mean2 = cv::mean(channels2[i])[0];

//         // 计算方差
//         cv::Mat var1, var2;
//         cv::multiply(channels1[i] - mean1, channels1[i] - mean1, var1);
//         cv::multiply(channels2[i] - mean2, channels2[i] - mean2, var2);
//         double var1_scalar = cv::mean(var1)[0];
//         double var2_scalar = cv::mean(var2)[0];

//         // 计算协方差
//         cv::Mat covar;
//         cv::multiply(channels1[i] - mean1, channels2[i] - mean2, covar);
//         double covar_scalar = cv::mean(covar)[0];

//         // 计算SSIM
//         double c1 = 0.01 * 255 * 0.01 * 255;
//         double c2 = 0.03 * 255 * 0.03 * 255;
//         double channel_ssim = (2 * mean1 * mean2 + c1) * (2 * covar_scalar + c2) / ((mean1 * mean1 + mean2 * mean2 + c1) * (var1_scalar + var2_scalar + c2));

//         // 累加每个通道的SSIM
//         ssim += channel_ssim;
//     }

//     // 求取均值
//     ssim /= 3.0;

//     return ssim;
// }


cv::Mat resizeImage(const cv::Mat& inputImage, const cv::Size& targetSize) {
    cv::Mat resizedImage;
    cv::resize(inputImage, resizedImage, targetSize);
    return resizedImage;
}


int main(int argc, char*argv[])
{
    // std::cout << cv::getBuildInformation() << std::endl; 

    // // cv::Ptr<cv::SURF> surf = cv::SURF::create();
    // cv::Ptr<cv::SIFT> detector = cv::SIFT::create(20);
    // // vector<KeyPoint> keypoints, keypoints2;
    // // detector->detect(src, keypoints);

    // return 0;
    // cv::Mat src_1 = cv::imread("/home/nx/code/robusttracker/v0/build/detrect.png");
    // // cv::Mat src_2 = cv::imread("/home/nx/code/robusttracker/v0/build/patch.png");
    // cv::Mat src_2 = cv::imread("/home/nx/code/robusttracker/v0/build/initdet.png");

    // cv::Size targetSize(std::min(src_1.cols, src_2.cols), std::min(src_1.rows, src_2.rows));
    // src_1 = resizeImage(src_1, targetSize);
    // src_2 = resizeImage(src_2, targetSize);

    // // cv::Scalar re = CalcMSSIM(src_1,src_2);
    // double ssim = calculateSSIM(src_1, src_2);
    // // std::cout<<"the r g b channles similarity is :"<<re<<std::endl;
    // std::cout<<"the r g b channles similarity is :"<<ssim<<std::endl;

    // return 0;


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

    
    std::vector<bbox_t> detRet;
    // std::vector<TrackingObject> detRet;

    uint8_t trackerStatus[9];
    memset(trackerStatus, 0, 9);

    cv::Mat templt;

    // rtracker->setIrFrame(true);

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
                // rtracker->update(trackFrame, detRet, trackerStatus);
                rtracker->update(frame, detRet, trackerStatus);
                spdlog::debug("tracker status:{}", trackerStatus[4]);
            }
            cv::imshow("trackRet", frame);
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
        
        
        char c = cv::waitKey(waitVAL);
        if(c == 'g')
            waitVAL = 10;
        else if(c == 's')
            waitVAL = 0;

    }
    // cv::Mat img = cv::imread("/space/data/0211/2-755.png");


    // detector->process(img);
    // cv::imshow("1", img);
    // cv::waitKey(0);

    return 0;
}