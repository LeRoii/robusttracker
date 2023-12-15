#include "realtracker.h"
#include <arpa/inet.h>
#include "spdlog/spdlog.h"

#define TRACKER_DEBUG 1

void DrawFilledRect(cv::Mat& frame, const cv::Rect& rect, cv::Scalar cl, int alpha)
{
    if (alpha)
    {
        const int alpha_1 = 255 - alpha;
        const int nchans = frame.channels();
        int color[3] = { cv::saturate_cast<int>(cl[0]), cv::saturate_cast<int>(cl[1]), cv::saturate_cast<int>(cl[2]) };
        for (int y = rect.y; y < rect.y + rect.height; ++y)
        {
            uchar* ptr = frame.ptr(y) + nchans * rect.x;
            for (int x = rect.x; x < rect.x + rect.width; ++x)
            {
                for (int i = 0; i < nchans; ++i)
                {
                    ptr[i] = cv::saturate_cast<uchar>((alpha_1 * ptr[i] + alpha * color[i]) / 255);
                }
                ptr += nchans;
            }
        }
    }
    else
    {
        // cv::rectangle(frame, rect, cl, cv::FILLED);
        cv::rectangle(frame, rect, cl, 2);
    }
}

void DrawTrack(cv::Mat frame,
                             const TrackingObject& track,
                             bool drawTrajectory,
                             int framesCounter)
{

    // printf("DrawTrack\n");
    cv::Scalar color = track.m_isStatic ? cv::Scalar(255, 0, 255) : cv::Scalar(0, 255, 255);
    cv::Point2f rectPoints[4];
    track.m_rrect.points(rectPoints);
    // std::cout << "track.m_rrect: " << track.m_rrect.center << ", " << track.m_rrect.angle << ", " << track.m_rrect.size << std::endl;
    for (int i = 0; i < 4; ++i)
    {
        cv::line(frame, rectPoints[i], rectPoints[(i+1) % 4], color, 2);
    }
    if (drawTrajectory)
    {
        cv::Scalar cl = cv::Scalar(0,255,255);

        for (size_t j = 0; j < track.m_trace.size() - 1; ++j)
        {
            const TrajectoryPoint& pt1 = track.m_trace.at(j);
            const TrajectoryPoint& pt2 = track.m_trace.at(j + 1);
#if (CV_VERSION_MAJOR >= 4)
            cv::line(frame, pt1.m_prediction, pt2.m_prediction, cl, 1, cv::LINE_AA);
#else
            cv::line(frame, pt1.m_prediction, pt2.m_prediction, cl, 1, CV_AA);
#endif
            if (!pt2.m_hasRaw)
            {
#if (CV_VERSION_MAJOR >= 4)
                cv::circle(frame, pt2.m_prediction, 4, cl, 1, cv::LINE_AA);
#else
                cv::circle(frame, pt2.m_prediction, 4, cl, 1, CV_AA);
#endif
            }
        }
    }

    // printf("111111111111\n");
    cv::Rect brect = track.m_rrect.boundingRect();
    std::string label = track.m_ID.ID2Str();
    // if (track.m_type != bad_type)
    //     label += " (" + TypeConverter::Type2Str(track.m_type) + ")";

    // printf("2222222222\n");
    int baseLine = 0;
    double fontScale = (frame.cols < 1920) ? 0.5 : 0.7;
    cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_TRIPLEX, fontScale, 1, &baseLine);
    if (brect.x < 0)
    {
        brect.width = std::min(brect.width, frame.cols - 1);
        brect.x = 0;
    }
    else if (brect.x + brect.width >= frame.cols)
    {
        brect.x = std::max(0, frame.cols - brect.width - 1);
        brect.width = std::min(brect.width, frame.cols - 1);
    }
    if (brect.y - labelSize.height < 0)
    {
        brect.height = std::min(brect.height, frame.rows - 1);
        brect.y = labelSize.height;
    }
    else if (brect.y + brect.height >= frame.rows)
    {
        brect.y = std::max(0, frame.rows - brect.height - 1);
        brect.height = std::min(brect.height, frame.rows - 1);
    }
    // printf("m_isStatic\n");
    // DrawFilledRect(frame, cv::Rect(cv::Point(brect.x, brect.y - labelSize.height), cv::Size(labelSize.width, labelSize.height + baseLine)), cv::Scalar(200, 200, 200), 0);
    // cv::putText(frame, label+ std::to_string(track.m_confidence), brect.tl(), cv::FONT_HERSHEY_TRIPLEX, fontScale, cv::Scalar(0, 0, 0));
    cv::putText(frame, label, brect.tl(), cv::FONT_HERSHEY_TRIPLEX, fontScale, cv::Scalar(0, 0, 0));

}

// void Tracks2Boxs(const std::vector<TrackingObject>& tracks, std::vector<bbox_t> &boxs)
// {
//     boxs.clear();
//     // printf("track size:%d\n", tracks.size());
//     for (const auto& track : tracks)
//     {   
//         cv::Rect brect = track.m_rrect.boundingRect();
//         boxs.emplace_back(brect.x, brect.y, brect.width, brect.height, track.m_type, track.m_ID.m_val, track.m_confidence);
//     }
// }

void DrawData(cv::Mat frame, const std::vector<TrackingObject>& tracks, int framesCounter, int currTime)
    {

        for (const auto& track : tracks)
        {

            // printf("track id:%d, velo x:%f, velo y:%f, speed:%f, type:%d, isStatic:%d, statis time:%d, out of frame:%d,\
            // lastRobust:%d\n", track.m_ID, track.m_velocity[0], track.m_velocity[1], sqrt(sqr(track.m_velocity[0]) + sqr(track.m_velocity[1])), track.m_type, track.m_isStatic,\
            // track.m_isStaticTime, track.m_outOfTheFrame, track.m_lastRobust);
            if (track.m_isStatic)
            {
                // printf("m_isStatic\n");
                DrawTrack(frame, track, false, framesCounter);

				std::string label = "abandoned " + track.m_ID.ID2Str();
				int baseLine = 0;
				cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_TRIPLEX, 0.5, 1, &baseLine);

				cv::Rect brect = track.m_rrect.boundingRect();
				if (brect.x < 0)
				{
					brect.width = std::min(brect.width, frame.cols - 1);
					brect.x = 0;
				}
				else if (brect.x + brect.width >= frame.cols)
				{
					brect.x = std::max(0, frame.cols - brect.width - 1);
					brect.width = std::min(brect.width, frame.cols - 1);
				}
				if (brect.y - labelSize.height < 0)
				{
					brect.height = std::min(brect.height, frame.rows - 1);
					brect.y = labelSize.height;
				}
				else if (brect.y + brect.height >= frame.rows)
				{
					brect.y = std::max(0, frame.rows - brect.height - 1);
					brect.height = std::min(brect.height, frame.rows - 1);
				}
				DrawFilledRect(frame, cv::Rect(cv::Point(brect.x, brect.y - labelSize.height), cv::Size(labelSize.width, labelSize.height + baseLine)), cv::Scalar(255, 0, 255), 150);
				cv::putText(frame, label, brect.tl(), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 0));
            }
            else
            {
				auto velocity = sqrt(sqr(track.m_velocity[0]) + sqr(track.m_velocity[1]));

                // printf("!!!!m_isStatic\n");
				// if (track.IsRobust(4,             // Minimal trajectory size
				// 	0.3f,                         // Minimal ratio raw_trajectory_points / trajectory_lenght
				// 	cv::Size2f(0.2f, 5.0f)) &&    // Min and max ratio: width / height
				// 	velocity > 30)                // Velocity more than 30 pixels per second
				{
					track_t mean = 0;
					track_t stddev = 0;
					TrackingObject::LSParams lsParams;
					// if (track.LeastSquares2(20, mean, stddev, lsParams) && mean > stddev)
					{
						DrawTrack(frame, track, false, framesCounter);
					}
				}
            }
        }
    }


void genTrackerSettings(TrackerSettings &settings)
{
    cv::Mat tmp = cv::Mat(720, 1280, CV_8UC3);
    FrameInfo frameInfo(1);
	frameInfo.m_frames.resize(frameInfo.m_batchSize);
	frameInfo.m_frameInds.resize(frameInfo.m_batchSize);
    frameInfo.m_frames[0].GetMatBGRWrite() = tmp;
    cv::UMat umatFrame = frameInfo.m_frames[0].GetUMatBGR();
    float m_fps = 25;
    // settings.SetDistance(tracking::DistJaccard);
    settings.SetDistance(tracking::DistCenters);
    settings.m_kalmanType = tracking::KalmanLinear;
    settings.m_filterGoal = tracking::FilterCenter;
    settings.m_lostTrackType = tracking::TrackCSRT; // Use KCF tracker for collisions resolving. Used if m_filterGoal == tracking::FilterRect
    settings.m_matchType = tracking::MatchHungrian;
    settings.m_dt = 0.3f;                           // Delta time for Kalman filter
    settings.m_accelNoiseMag = 0.2f;                // Accel noise magnitude for Kalman filter
    settings.m_distThres = 0.8f;                    // Distance threshold between region and object on two frames
    settings.m_minAreaRadiusPix = umatFrame.rows / 20.f;
    settings.m_maximumAllowedSkippedFrames = cvRound(2 * m_fps); // Maximum allowed skipped frames
    settings.m_maxSpeedForStatic = 10;

    settings.m_useAbandonedDetection = false;
    if (settings.m_useAbandonedDetection)
    {
        settings.m_minStaticTime = 5;
        settings.m_maxStaticTime = 60;
        settings.m_maximumAllowedSkippedFrames = cvRound(settings.m_minStaticTime * m_fps); // Maximum allowed skipped frames
        settings.m_maxTraceLength = 2 * settings.m_maximumAllowedSkippedFrames;        // Maximum trace length
    }
    else
    {
        settings.m_maximumAllowedSkippedFrames = cvRound(10 * m_fps); // Maximum allowed skipped frames
        settings.m_maxTraceLength = cvRound(4 * m_fps);              // Maximum trace length
    }
    // settings.SetDistance(tracking::DistJaccard);
    settings.SetDistance(tracking::DistCenters);

}

void prepareCrosshair(cv::Mat &oriCrosshair)
{
    int crosshairW = 150;
	int crosshairH = 100;
	oriCrosshair = cv::Mat(crosshairH, crosshairW, CV_8UC3);
	oriCrosshair.setTo(0);

	int w = oriCrosshair.cols;
	int h = oriCrosshair.rows;

	auto centerPt = cv::Point(w/2, h/2);

	auto color = cv::Scalar(0,0,255);
	auto thickness = 4;
	cv::circle(oriCrosshair, centerPt, 1, color, 2);

	cv::Point leftPtStart = cv::Point(0, centerPt.y);
	cv::Point leftPtEnd = cv::Point(0 + w/2 - 10, centerPt.y);

	cv::Point upPtStart = cv::Point(centerPt.x, 0);
	cv::Point upPtEnd = cv::Point(centerPt.x, h/2 - 10);

	cv::Point rightPtStart = cv::Point(w/2 + 10, centerPt.y);
	cv::Point rightPtEnd = cv::Point(w, centerPt.y);

	cv::Point downPtStart = cv::Point(centerPt.x, h/2 + 10);
	cv::Point downPtEnd = cv::Point(centerPt.x, h);

	//crosshair
	cv::line(oriCrosshair, leftPtStart, leftPtEnd, color, thickness);
	cv::line(oriCrosshair, upPtStart, upPtEnd, color, thickness);
	cv::line(oriCrosshair, rightPtStart, rightPtEnd, color, thickness);
	cv::line(oriCrosshair, downPtStart, downPtEnd, color, thickness);


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
	cv::line(oriCrosshair, cv::Point(leftupX,leftupY), cv::Point(leftupX, leftupY + cornerH), color, thickness);
	cv::line(oriCrosshair, cv::Point(leftupX,leftupY), cv::Point(leftupX + cornerW,leftupY), color, thickness);

	//left down corner
	cv::line(oriCrosshair, cv::Point(leftdownX, leftdownY), cv::Point(leftdownX, leftdownY - cornerH), color, thickness);
	cv::line(oriCrosshair, cv::Point(leftdownX, leftdownY), cv::Point(leftdownX + cornerW, leftdownY), color, thickness);

	//right up corner
	cv::line(oriCrosshair, cv::Point(rightupX,rightupY), cv::Point(rightupX - cornerW, rightupY), color, thickness);
	cv::line(oriCrosshair, cv::Point(rightupX,rightupY), cv::Point(rightupX, rightupY + cornerH), color, thickness);

	//right down corner
	cv::line(oriCrosshair, cv::Point(rightdownX,rightdownY), cv::Point(rightdownX - cornerW, rightdownY), color, thickness);
	cv::line(oriCrosshair, cv::Point(rightdownX,rightdownY), cv::Point(rightdownX, rightdownY - cornerH), color, thickness);

}

//input para pt is rect centerpoint
void drawCrosshair(cv::Mat &frame, cv::Point pt, double scale)
{
    static cv::Mat oriCrosshair;
    if(oriCrosshair.empty())
        prepareCrosshair(oriCrosshair);

	cv::Mat mask;// = imgi.clone();
	// mask.setTo(1);

	cv::Mat resizedTemplate;
	cv::resize(oriCrosshair, resizedTemplate, cv::Size(), scale, scale);

	// printf("w:%d,h:%d\n", resizedTemplate.cols, resizedTemplate.rows);
	// cv::imwrite("111.png", resizedTemplate);
    // mask = cv::Mat::zeros(resizedTemplate.size(), CV_8UC1);
	mask = resizedTemplate.clone();
    cv::cvtColor(mask, mask, CV_RGB2GRAY);
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

static double calculateHistogramSimilarity(const cv::Mat& image1, const cv::Mat& image2) {
    // cv::Mat hsvImage1, hsvImage2;
    // cv::cvtColor(image1, hsvImage1, cv::COLOR_BGR2HSV);
    // cv::cvtColor(image2, hsvImage2, cv::COLOR_BGR2HSV);

    // int hBins = 30;
    // int sBins = 32;
    // int histSize[] = {hBins, sBins};
    // float hRanges[] = {0, 180};
    // float sRanges[] = {0, 256};
    // const float* ranges[] = {hRanges, sRanges};
    // int channels[] = {0, 1};

    // cv::MatND hist1, hist2;
    // cv::calcHist(&hsvImage1, 1, channels, cv::Mat(), hist1, 2, histSize, ranges, true, false);
    // cv::calcHist(&hsvImage2, 1, channels, cv::Mat(), hist2, 2, histSize, ranges, true, false);

    // double similarity = cv::compareHist(hist1, hist2, cv::HISTCMP_BHATTACHARYYA);

    // return similarity;

    int bins = 64;
    std::vector<int> histSize;
    std::vector<float> ranges;
    std::vector<int> channels;

    for (int j = 0, stop = image1.channels(); j < stop; ++j)
    {
        histSize.push_back(bins);
        ranges.push_back(0);
        ranges.push_back(255);
        channels.push_back(j);
    }
    // cv::Rect roi = cv::Rect{box.x,box.y,box.w,box.h};
    // Clamp(roi.x, roi.width, frame.cols);
    // Clamp(roi.y, roi.height, frame.rows);
    std::vector<cv::Mat> img1 = { image1 };
    std::vector<cv::Mat> img2 = { image2 };
    cv::Mat hist1, hist2;
    cv::calcHist(img1, channels, cv::Mat(), hist1, histSize, ranges, false);
    cv::calcHist(img2, channels, cv::Mat(), hist2, histSize, ranges, false);
    cv::normalize(hist1, hist1, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
    cv::normalize(hist2, hist2, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

    return cv::compareHist(hist1, hist2, cv::HISTCMP_BHATTACHARYYA);

}

cv::Mat CalcHist(cv::Mat img, cv::Rect box)
{
    int bins = 64;
    std::vector<int> histSize;
    std::vector<float> ranges;
    std::vector<int> channels;

    for (int j = 0, stop = img.channels(); j < stop; ++j)
    {
        histSize.push_back(bins);
        ranges.push_back(0);
        ranges.push_back(255);
        channels.push_back(j);
    }
    cv::Rect roi = cv::Rect{box.x,box.y,box.width,box.height};
    // Clamp(roi.x, roi.width, frame.cols);
    // Clamp(roi.y, roi.height, frame.rows);
    std::vector<cv::Mat> regROI = { img(roi) };
    cv::Mat hist;
    cv::calcHist(regROI, channels, cv::Mat(), hist, histSize, ranges, false);
    cv::normalize(hist, hist, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

    return hist;

}

double calculateSSIM(const cv::Mat& imgg1, const cv::Mat& imgg2)
{
    // printf("realtracker::calculateSSIM start\n");
    // std::cout<<"img1 :"<<img1.size()<<std::endl;
    // std::cout<<"img2 :"<<img2.size()<<std::endl;
    cv::Mat img1, img2;

    cv::Size targetSize(std::min(imgg1.cols, imgg2.cols), std::min(imgg1.rows, imgg2.rows));
    cv::resize(imgg1, img1, targetSize);
    cv::resize(imgg2, img2, targetSize);

    // printf("realtracker::calculateSSIM\n");

    // 分离通道
    std::vector<cv::Mat> channels1, channels2;
    cv::split(img1, channels1);
    cv::split(img2, channels2);

    double ssim = 0.0;

    // 计算每个通道的SSIM
    for (int i = 0; i < 3; ++i) {
        // 转换为double类型
        channels1[i].convertTo(channels1[i], CV_64F);
        channels2[i].convertTo(channels2[i], CV_64F);

        // 计算均值
        double mean1 = cv::mean(channels1[i])[0];
        double mean2 = cv::mean(channels2[i])[0];

        // 计算方差
        cv::Mat var1, var2;
        cv::multiply(channels1[i] - mean1, channels1[i] - mean1, var1);
        cv::multiply(channels2[i] - mean2, channels2[i] - mean2, var2);
        double var1_scalar = cv::mean(var1)[0];
        double var2_scalar = cv::mean(var2)[0];

        // 计算协方差
        cv::Mat covar;
        cv::multiply(channels1[i] - mean1, channels2[i] - mean2, covar);
        double covar_scalar = cv::mean(covar)[0];

        // 计算SSIM
        double c1 = 0.01 * 255 * 0.01 * 255;
        double c2 = 0.03 * 255 * 0.03 * 255;
        double channel_ssim = (2 * mean1 * mean2 + c1) * (2 * covar_scalar + c2) / ((mean1 * mean1 + mean2 * mean2 + c1) * (var1_scalar + var2_scalar + c2));

        // 累加每个通道的SSIM
        ssim += channel_ssim;
    }

    // 求取均值
    ssim /= 3.0;

    return ssim;
}

int Clamp(int& v, int& size, int hi)
{
    int res = 0;
    if (v < 0)
    {
        res = v;
        v = 0;
        return res;
    }
    else if (v + size > hi - 1)
    {
        res = v;
        v = hi - 1 - size;
        if (v < 0)
        {
            size += v;
            v = 0;
        }
        res -= v;
        return res;
    }
    return res;
};


void trackObj::init(const bbox_t &box, cv::Mat frame)
{
    m_rect = cv::Rect{box.x, box.y, box.w, box.h};
    m_prob = box.prob;
    m_cls = box.obj_id;
    m_age = 1;
    m_lostCnt = 0;
    m_trace.clear();
    m_trace.emplace_back(cv::Point(box.x+box.w/2, box.y + box.h/2));

    int bins = 64;
    std::vector<int> histSize;
    std::vector<float> ranges;
    std::vector<int> channels;

    for (int j = 0, stop = frame.channels(); j < stop; ++j)
    {
        histSize.push_back(bins);
        ranges.push_back(0);
        ranges.push_back(255);
        channels.push_back(j);
    }
    cv::Rect roi = cv::Rect{box.x,box.y,box.w,box.h};
    // Clamp(roi.x, roi.width, frame.cols);
    // Clamp(roi.y, roi.height, frame.rows);
    std::vector<cv::Mat> regROI = { frame(roi) };
    cv::calcHist(regROI, channels, cv::Mat(), m_hist, histSize, ranges, false);
    cv::normalize(m_hist, m_hist, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

    m_lastPos = m_rect;

    // m_veloBUf.resize(10);
    m_strackerLost = false;
    m_dtrackerLost = false;

    m_patch = frame(m_rect);

    m_initRect = m_rect;
    m_velo[0] = m_velo[1] = 0;

    std::cout<<" trackObj::init:::"<<m_rect<<std::endl;


}

cv::Point trackObj::center()
{
    return cv::Point(m_rect.x+m_rect.width/2, m_rect.y + m_rect.height/2);
}

inline void trackObj::calcVelo()
{
    float sumx, sumy;
    sumx = sumy = 0.0;
    for(auto& velo:m_veloBuf)
    {
        sumx += velo.first;
        sumy += velo.second;
    }
    m_velo[0] = sumx/m_veloBuf.size();
    m_velo[1] = sumy/m_veloBuf.size();
}

void trackObj::update(const bbox_t &box)
{
#if 0
    printf("trackObj::update:\n");
    m_rect = cv::Rect{box.x, box.y, box.w, box.h};
    std::cout<<m_rect<<std::endl;
    if(m_trace.size() > 100)
        m_trace.pop_front();
    m_trace.emplace_back(cv::Point(box.x+box.w/2, box.y + box.h/2));
    m_age++;
    m_lostCnt = 0;

    if(m_veloBuf.size() > 5)
        m_veloBuf.pop_front();
    m_veloBuf.emplace_back(std::pair<int,int>{box.x - m_lastPos.x, box.y - m_lastPos.y});

    calcVelo();
    printf("velo x:%f, velo y:%f\n", m_velo[0], m_velo[1]);
    m_lastPos = m_rect;

    // cv::Vec4f lines;//存放拟合后的直线
    // std::vector<cv::Point2f>point;//待检测是否存在直线的所有点
    // //将所有点存放在vector中，用于输入函数中
    // for(auto &pt:m_trace){
    //     point.push_back(static_cast<cv::Point2f>(pt));
    // }
    // double param=0;//距离模型中的数值参数C
    // double reps=0.01;//坐标原点与直线之间的距离
    // double aeps=0.01;//角度精度
    // cv::fitLine(point,lines,cv::DIST_L1,0,0.01,0.01);
    // double k=lines[1]/lines[0];//直线斜率
    // std::cout<<"直线斜率: "<<k<<std::endl;
    // cout<<"直线上一点坐标x: "<<lines[2]<<",y: "<<lines[3]<<endl;
    // cout<<"直线解析式: y="<<k<<"(x-"<<lines[2]<<")+"<<lines[3]<<endl;
#endif
}

void trackObj::update(cv::Mat img, const cv::Rect &box, double ssim)
{
    printf("trackObj::update:\n");
    m_rect = box;
    int sizeDif = abs(m_rect.width - m_lastPos.width) + abs(m_rect.height - m_lastPos.height);
#if TRACKER_DEBUG
    std::cout<<"curPos:"<<m_rect<<std::endl;
    std::cout<<"m_lastPos:"<<m_lastPos<<std::endl;
    printf("w diff:%d, h diff:%d\n", abs(m_rect.width - m_lastPos.width), abs(m_rect.height - m_lastPos.height));
#endif
    if(m_trace.size() > 100)
        m_trace.pop_front();
    m_trace.emplace_back(cv::Point(box.x+box.width/2, box.y + box.height/2));
    m_age++;
    
    if(m_lostCnt == 0 && sizeDif < 8)
    {
        if(m_veloBuf.size() > 10)
            m_veloBuf.pop_front();
        m_veloBuf.emplace_back(std::pair<int,int>{box.x - m_lastPos.x, box.y - m_lastPos.y});

        calcVelo();
    }

    m_lostCnt = 0;
    m_lastPos = m_rect;

    Clamp(m_rect.x, m_rect.width, img.cols);
    Clamp(m_rect.y, m_rect.height, img.rows);
    if(ssim > 0.7)
        m_patch = img(m_rect).clone();
#if TRACKER_DEBUG
    printf("inst velo x:%d, inst velo y:%d\n", m_veloBuf.back().first, m_veloBuf.back().second);
    // std::cout<<"patch size:"<<m_patch.size()<<std::endl;
    // cv::imwrite("patch.png", m_patch);
#endif

    // m_hist = CalcHist(img, box);
}

void trackObj::updateWithoutDet()
{
    m_rect.x += (int)round(m_velo[0]);
    m_rect.y += (int)round(m_velo[1]);
    m_lostCnt++;
}

void trackObj::predict()
{
    m_rect.x += (int)round(m_velo[0]);
    m_rect.y += (int)round(m_velo[1]);
}

bool trackObj::isLost()
{
    if(m_trace.size() < 5)
    {
        return m_lostCnt > 1;
    }
    else if(m_trace.size() < 10)
    {
        return m_lostCnt > 3;
    }
    else if(m_trace.size() < 25)
    {
        return m_lostCnt > 5;
    }
    else
    {
        return m_lostCnt > 25;
    }
}

static cv::Mat safeCrop(cv::Mat img, cv::Rect roi)
{
    Clamp(roi.x, roi.width, img.cols);
    Clamp(roi.y, roi.height, img.rows);

    return img(roi);
}





realtracker::realtracker(std::string enginepath):m_frameInfo()
{
    m_stracker = new itracker();
    m_stracker->setGateSize(32);

    m_detector = new idetector(enginepath);
    m_detector->init();

    m_fps = 25;
    cv::Mat tmp = cv::Mat(720, 1280, CV_8UC3);
	m_frameInfo.m_frames.resize(m_frameInfo.m_batchSize);
	m_frameInfo.m_frameInds.resize(m_frameInfo.m_batchSize);
    m_frameInfo.m_frames[0].GetMatBGRWrite() = tmp;
    // cv::UMat umatFrame = frameInfo.m_frames[0].GetUMatBGR();

    // std::unique_ptr<BaseTracker> mtracker;
    TrackerSettings settings;
    genTrackerSettings(settings);
    m_mtracker = BaseTracker::CreateTracker(settings);
    m_frameInfo.CleanRegions();
    m_frameInfo.CleanTracks();

    m_state = EN_TRACKER_FSM::LOST;
    m_frameScale = 1.f;

    m_trackerOffsetLimit = 150;

    m_dtrackerLost = false;
    m_strackerLost = false;

    m_irFrame = false;

    m_dtrackerLostCnt = 0;
}

realtracker::~realtracker()
{

}
inline double getDistance (cv::Point point1, cv::Point point2)
{
    return  sqrtf(powf((point1.x - point2.x),2) + powf((point1.y - point2.y),2));
}



void realtracker::init(const cv::Rect &roi, cv::Mat image)
{
    // m_stracker->init(roi, image);
}

void realtracker::init(const cv::Point &pt, cv::Mat image)
{
    double x = (double)pt.x * m_frameScale;
    double y = (double)pt.y * m_frameScale;
    cv::Point iniPt = cv::Point{(int)x, (int)y};

    m_stracker->init(iniPt, image);
    if(m_irFrame)
    {
        m_state = EN_TRACKER_FSM::STRACK;
        return;
    }

    spdlog::warn("realtracker::init x:%d,y:%d, ptx:%d, pty:%d\n", iniPt.x, iniPt.y, pt.x, pt.y);

    std::vector<bbox_t> detRet;
#if TRACKER_DEBUG
    runDetector(image, detRet);
#else
    runDetectorNoDraw(image, detRet);
#endif
    float minDist = 1000.f;
    int minIdx = -1;
    for(int i=0; i< detRet.size(); ++i)
    {
        cv::Point center{detRet[i].x + detRet[i].w/2, detRet[i].y + detRet[i].h/2};
        double dist = getDistance(iniPt, center);
        // printf("obj pos:(%d, %d), dist:%f\n", brect.tl().x, brect.tl().y, dist);
        if(dist < minDist)
        {
            minDist = dist;
            minIdx = i;
        }
    }
    printf("realtracker::init minIdx = %d\n", minIdx);

    m_state = EN_TRACKER_FSM::STRACK;
    if(minIdx != -1)
    {
        cv::Rect closestRect{detRet[minIdx].x, detRet[minIdx].y, detRet[minIdx].w, detRet[minIdx].h};
        if(closestRect.contains(iniPt))
        {
            
            m_state = EN_TRACKER_FSM::DTRACK;
            m_initTarget = image(closestRect);
#if TRACKER_DEBUG
            printf("\n\ninit pt with det\n");
            cv::imwrite("initdet.png", m_initTarget);
#endif
            m_trackCls = detRet[minIdx].obj_id;
            // lastId = m_frameInfo.m_tracks[0][minIdx].m_ID.m_val;
            m_trackObj.init(detRet[minIdx], image);
        }
    }


    minDistThres = 15.0f;
    areaDifThres = 25.0f;

    

}


void realtracker::FSM_PROC_SEARCH(cv::Mat &frame)
{
    printf("\nSM_PROC_SEARCH\n");
    static int searchFrameCnt = 0;
    searchFrameCnt++;
    if(searchFrameCnt > 3)
    {
        m_state = EN_TRACKER_FSM::LOST;
        searchFrameCnt = 0;
        m_mtrackerLostCnt = 0;
        m_stracker->reset();
    }
}

int intersectionArea(cv::Rect r1, cv::Rect r2)
{
    cv::Rect intersection = r1 & r2;
    return intersection.area();

}

void realtracker::FSM_PROC_STRACK(cv::Mat &frame)
{
    printf("\nFSM_PROC_STRACK\n");
    std::vector<bbox_t> detRet;

    if(!m_irFrame)
    {
        // auto detimg = frame.clone();
        // runDetector(detimg, detRet);

#if TRACKER_DEBUG
        auto detimg = frame.clone();
        runDetector(detimg, detRet);
#else
        runDetectorNoDraw(frame, detRet);
#endif

    }
    
    runTracker(frame);
// #if TRACKER_DEBUG
//     auto detimg = frame.clone();
//     runDetector(detimg, detRet);
// #else
//     runDetectorNoDraw(frame, detRet);
// #endif

    if(m_stracker->isLost())
    {
        printf("STRACK lost\n\n");
        m_state = EN_TRACKER_FSM::SEARCH;
    }
    else
    {
        if(m_irFrame)
        {
            m_state = EN_TRACKER_FSM::STRACK;
            return;
        }
        for(int i=0; i< detRet.size(); ++i)
        {
            if(detRet[i].w < 32 && detRet[i].h < 32)
                continue;
            cv::Rect brect = cv::Rect{detRet[i].x, detRet[i].y, detRet[i].w, detRet[i].h};
            cv::Rect strackerRect = cv::Rect(m_stracker->centerPt().x - m_stracker->m_GateSize/2, m_stracker->centerPt().y - m_stracker->m_GateSize, m_stracker->m_GateSize, m_stracker->m_GateSize);
            int area = intersectionArea(brect, strackerRect);
            // printf("STRACK dist:%d\n", area);

            //is inside
            if(brect.contains(m_stracker->centerPt()))
            {
                m_state = EN_TRACKER_FSM::DTRACK;
                m_trackObj.init(detRet[i],frame);
                printf("STRACK to DTRACK\n");
#if TRACKER_DEBUG
                rectangle(frame, brect, cv::Scalar(0,255,255), 3, 8 );
#endif
                return;
            }
            
            if(area > 0)
            {
                
                double sim = calculateHistogramSimilarity(frame(brect), frame(strackerRect));
                // printf("FSM_PROC_STRACK sim:%f\n", sim);
                if(sim > 0.5)
                	break;
                m_state = EN_TRACKER_FSM::DTRACK;
                // m_trackCls = m_frameInfo.m_tracks[0][i].m_type;
                // lastId = m_frameInfo.m_tracks[0][i].m_ID.m_val;

                m_trackObj.init(detRet[i],frame);

                printf("STRACK to DTRACK\n");
#if TRACKER_DEBUG
                rectangle(frame, brect, cv::Scalar(0,255,255), 3, 8 );
#endif
                return;

            }
        }
        rectangle(frame, m_strackerRet, cv::Scalar(255,255,255), 3, 8 );

        m_state = EN_TRACKER_FSM::STRACK;
    }
}

// auto Clamp = [](int& v, int& size, int hi) -> int
// int Clamp(int& v, int& size, int hi)
// {
//     int res = 0;
//     if (v < 0)
//     {
//         res = v;
//         v = 0;
//         return res;
//     }
//     else if (v + size > hi - 1)
//     {
//         res = v;
//         v = hi - 1 - size;
//         if (v < 0)
//         {
//             size += v;
//             v = 0;
//         }
//         res -= v;
//         return res;
//     }
//     return res;
// };


void realtracker::FSM_PROC_DTRACK(cv::Mat &frame)
{
    printf("\nFSM_PROC_DTRACK\n");
    std::vector<bbox_t> detRet;
#if TRACKER_DEBUG
    auto detimg = frame.clone();
    auto debugimg = frame.clone();
    runDetector(detimg, detRet);
    runTracker(frame, false);
#else
    runDetectorNoDraw(frame, detRet);
    runTrackerNoDraw(frame, false);
#endif
    m_trackObj.predict();

    std::cout<<"FSM_PROC_DTRACK start:::"<<m_trackObj.m_rect<<std::endl;

    cv::Rect finalRect;
    cv::Rect m_dtrackerRet;
    cv::Scalar color = cv::Scalar(255,255,255);

    m_strackerLost = m_stracker->isLost();

    printf("detRet size :%d\n", detRet.size());
    if(detRet.size() == 0)
    {
        if(m_strackerLost)
        {
            m_trackObj.updateWithoutDet();
            color = cv::Scalar(0,0,0);
        }
        else
        {
            finalRect.x =  m_strackerRet.x + m_strackerRet.width/2 - m_trackObj.m_rect.width/2;
            finalRect.y = m_strackerRet.y + m_strackerRet.height/2 - m_trackObj.m_rect.height/2;
            finalRect.width = m_trackObj.m_rect.width;
            finalRect.height = m_trackObj.m_rect.height;
            color = cv::Scalar(0,0,255);

            m_trackObj.update(frame, finalRect, 0);
        }

        if(m_trackObj.isLost())
        {
            spdlog::debug("tracker lost");
            m_state = EN_TRACKER_FSM::SEARCH;
            return;
        }

        cv::rectangle(frame, m_trackObj.m_rect, color, 2);
        printf("m_trackObj age:%d, lostcnt:%d, trace size:%d, velo x:%f, velo y:%f\n", 
            m_trackObj.m_age, m_trackObj.m_lostCnt, m_trackObj.m_trace.size(), m_trackObj.m_velo[0], m_trackObj.m_velo[1]);
        std::cout<<m_trackObj.m_rect<<std::endl;

        m_state = EN_TRACKER_FSM::DTRACK;

        return;
    }
    
    int minIdx = -1;
    double minDist = 10000.f;
    bool findLast = false;

    // auto trackObj = m_trackObj;
    
    auto cmpDist = [this](bbox_t box1, bbox_t box2)
    {
        cv::Point center1{box1.x + box1.w/2, box1.y + box1.h/2};
        double dist1 = getDistance(m_trackObj.center(), center1);
        cv::Point center2{box2.x + box2.w/2, box2.y + box2.h/2};
        double dist2 = getDistance(m_trackObj.center(), center2);
        if(box1.obj_id != 1)
            dist1 = 1000;
        if(box2.obj_id != 1)
            dist2 = 1000;

        return dist1 < dist2;
    };

    std::sort(detRet.begin(), detRet.end(), cmpDist);

    cv::Point center{detRet.front().x + detRet.front().w/2, detRet.front().y + detRet.front().h/2};
    minDist = getDistance(m_trackObj.center(), center);
    cv::Rect closestSSIMRect;
    // int areaDif = m_trackObj.m_initRect.width - detRet.front().w + m_trackObj.m_initRect.height - detRet.front().h;
    int areaDif = m_trackObj.m_rect.width - detRet.front().w + m_trackObj.m_rect.height - detRet.front().h;
    areaDif = abs(areaDif);
    double maxSSIM = 0;
    double maxSSIMDif = 0;
    // minDist = 0.01;
#if TRACKER_DEBUG

    // cv::Ptr<cv::SIFT> detector = cv::SIFT::create(20);
    // std::vector<cv::KeyPoint> keypoints1, keypoints2;
    // cv::Rect siftRoi = cv::Rect(m_trackObj.m_rect.x-100,m_trackObj.m_rect.y-100,150,150);
    // cv::Mat framePatch = safeCrop(frame,siftRoi);
    // detector->detect(m_trackObj.m_patch, keypoints1);
    // detector->detect(framePatch, keypoints2);

    // // cv::drawKeypoints(m_trackObj.m_patch, keypoints1, m_trackObj.m_patch);
    // // cv::imshow("drawsrc", m_trackObj.m_patch);
    // // cv::drawKeypoints(frame(siftRoi), keypoints2, frame(siftRoi));
    // // cv::imshow("drawsrc2", frame(siftRoi));

    // cv::Mat dstSIFT, dstSIFT2;
    // cv::Ptr<cv::SiftDescriptorExtractor> descriptor = cv::SiftDescriptorExtractor::create();
    // descriptor->compute(m_trackObj.m_patch, keypoints1, dstSIFT);
    // descriptor->compute(framePatch, keypoints2, dstSIFT2);

    // //进行BFMatch暴力匹配
    // cv::BFMatcher matcher(cv::NORM_L2);
    // //定义匹配结果变量
    // std::vector<cv::DMatch> matches;
    // //实现描述符之间的匹配
    // matcher.match(dstSIFT, dstSIFT2, matches);
    
    
    // //定义向量距离的最大值与最小值
    // double max_dist = 0;
    // double min_dist = 1000;
    // for (int i = 1; i < dstSIFT.rows; ++i)
    // {
    //     //通过循环更新距离，距离越小越匹配
    //     double dist = matches[i].distance;
    //     if (dist > max_dist)
    //     max_dist = dist;
    //     if (dist < min_dist)
    //     min_dist = dist;
    // }
    // // cout << "min_dist=" << min_dist << endl;
    // // cout << "max_dist=" << max_dist << endl;
    // //匹配结果筛选    
    // std::vector<cv::DMatch> goodMatches;
    // for (int i = 0; i < matches.size(); ++i)
    // {
    //     double dist = matches[i].distance;
    //     if (dist < 2 * min_dist)
    //     goodMatches.push_back(matches[i]);
    // }
    // std::cout << "goodMatches:" << goodMatches.size() << std::endl;

    // cv::Mat result;
    // //匹配特征点天蓝色，单一特征点颜色随机
    // cv::drawMatches(m_trackObj.m_patch, keypoints1, framePatch, keypoints2, goodMatches, result, 
    //     cv::Scalar(255, 255, 0), cv::Scalar::all(-1));
    // cv::imshow("Result", result);


    // calculate hist start
    {
    int histCalCnt = detRet.size() > 3 ? 3 : detRet.size();
    std::vector<cv::Mat> detHists; 
    detHists.resize(histCalCnt);
    cv::Scalar colormap[3] = {{0,255,0},{255,0,0},{0,0,255}};
    int closestSSIMIdx = -1;
    
    
    for(int i=0;i<histCalCnt; ++i)
    {
        // int bins = 64;
        // std::vector<int> histSize;
        // std::vector<float> ranges;
        // std::vector<int> channels;

        // for (int j = 0, stop = frame.channels(); j < stop; ++j)
        // {
        //     histSize.push_back(bins);
        //     ranges.push_back(0);
        //     ranges.push_back(255);
        //     channels.push_back(j);
        // }

        cv::Rect roi = cv::Rect{detRet[i].x,detRet[i].y,detRet[i].w,detRet[i].h};
        Clamp(roi.x, roi.width, frame.cols);
        Clamp(roi.y, roi.height, frame.rows);
        // std::vector<cv::Mat> regROI = { frame(roi) };
        // cv::calcHist(regROI, channels, cv::Mat(), detHists[i], histSize, ranges, false);
        // cv::normalize(detHists[i], detHists[i], 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

        // double res = cv::compareHist(detHists[i], m_trackObj.m_hist, cv::HISTCMP_BHATTACHARYYA);
        double res = calculateSSIM(frame(roi), m_trackObj.m_patch);

        cv::Point center{detRet[i].x + detRet[i].w/2, detRet[i].y + detRet[i].h/2};
        cv::line(debugimg, center, m_trackObj.center(), colormap[i], 2);
        cv::putText(debugimg, std::to_string(res), center, cv::FONT_HERSHEY_COMPLEX, 0.8, colormap[i], 1, 8, 0);
        if(res > maxSSIM)
        {
            maxSSIMDif = res - maxSSIM;
            maxSSIM = res;
            closestSSIMIdx = i;
            closestSSIMRect = roi;
        }
    }
    //calculate hist end
    }
#endif

    printf("realtracker::FSM_PROC_DTRACK dist:%f, areaDif:%d\n", minDist, areaDif);
    // if((minIdx != -1 && minDist < 70) || findLast)
    // double minDistThres = 15.0;//m_trackObj.m_lostCnt > 10 ? 50.0 : 15.0;
    // double areaDifThres = 25.0f;
    
    // if(m_dtrackerLost && m_strackerLost)
    // {
    //     minDistThres = std::max(70.0, minDistThres*2);
    //     areaDifThres = 60;
    // }
    // else if(m_dtrackerLost)
    // {
    //     minDistThres = std::max(50.0, minDistThres);
    //     areaDifThres = 50;
    // }

    if(m_dtrackerLost)
    {
        m_dtrackerLostCnt++;
        if(m_dtrackerLostCnt > 3)
        {
            minDistThres = std::min(70.0, minDistThres*2);
            areaDifThres = 50;
        }
    }
    else
    {
        m_dtrackerLostCnt = 0;
    }

    spdlog::debug("minDistThres:{}, areaDifThres:{}", minDistThres, areaDifThres);
    cv::Rect derRect = cv::Rect{detRet.front().x, detRet.front().y, detRet.front().w, detRet.front().h};
    Clamp(derRect.x, derRect.width, frame.cols);
    Clamp(derRect.y, derRect.height, frame.rows);

    // std::cout<<"closest detRet:"<<derRect<<std::endl;

#if TRACKER_DEBUG
    cv::rectangle(debugimg, m_trackObj.m_rect, cv::Scalar(0, 0, 0), 2);
    cv::rectangle(debugimg, derRect, cv::Scalar(255, 0, 127), 2);
    cv::imwrite("detrect.png", frame(derRect));
    cv::imwrite("patch.png", m_trackObj.m_patch);
    // cv::imwrite("m_trackObj.m_rect.png", frame(m_trackObj.m_rect));

    // double sim = calculateHistogramSimilarity(frame(m_trackObj.m_rect), frame(derRect));
    // printf("FSM_PROC_DTRACKdddddddddddddddddddddd sim:%f\n", sim);

    // double sim = calculateHistogramSimilarity(m_trackObj.m_patch, frame(derRect));
    // printf("FSM_PROC_DTRACK 11111111111sim:%f\n", sim);

    // sim = calculateHistogramSimilarity(m_trackObj.m_patch, frame(m_trackObj.m_rect));
    // printf("FSM_PROC_DTRACK 22222222222222222222ssssssssssss:%f\n", sim);

    // std::cout<<"derRect:"<<derRect<<std::endl;
    // std::cout<<"m_trackObj.m_rect:"<<m_trackObj.m_rect<<std::endl;
    // std::cout<<"patch size:"<<m_trackObj.m_patch.size()<<std::endl;
    // std::cout<<"m_trackObj.m_patch:"<<derRect<<std::endl;

#endif

    if((minDist < minDistThres) && areaDif < areaDifThres)
    {
        m_dtrackerRet = derRect;
        int area = intersectionArea(closestSSIMRect, derRect);
        if(area == 0 && maxSSIM > 0.65 && maxSSIMDif > 0.3)
        {
            spdlog::debug("DTRACKER switch due to SSIM");
            m_dtrackerRet = closestSSIMRect;
        }

        m_dtrackerLost = false;
        minDistThres = 15.0f;
        areaDifThres = 25.0f;
    }
    else
    {
        spdlog::debug("mtracker find nearest failed, find by feature");
        if(maxSSIM > 0.65)
        {
            m_dtrackerRet = closestSSIMRect;
            m_dtrackerLost = false;
            minDistThres = 15.0f;
            areaDifThres = 25.0f;
        }
        else
        {
            m_dtrackerLost = true;
        }
    }

    
    if(m_strackerLost && m_dtrackerLost)
    {
        spdlog::debug("all tracker lost");
        m_trackObj.updateWithoutDet();
        color = cv::Scalar(0,0,0);
    }
    else
    {
        if(m_strackerLost && !m_dtrackerLost)
        {
            static int strackerWait = 0;
            spdlog::debug("s tracker lost");
            finalRect = m_dtrackerRet;
            color = cv::Scalar(0,255,255);
            m_stracker->reset();
            if(strackerWait++ > 5)
            {
                spdlog::debug("s tracker restart");
                m_stracker->init(cv::Point(m_dtrackerRet.x + m_dtrackerRet.width/2, m_dtrackerRet.y + m_dtrackerRet.height/2), frame);
                strackerWait = 0;
            }
            
        }
        else if(!m_strackerLost && m_dtrackerLost)
        {
            spdlog::debug("dtracker lost");
            finalRect.x =  m_strackerRet.x + m_strackerRet.width/2 - m_trackObj.m_rect.width/2;
            finalRect.y = m_strackerRet.y + m_strackerRet.height/2 - m_trackObj.m_rect.height/2;
            finalRect.width = m_trackObj.m_rect.width;
            finalRect.height = m_trackObj.m_rect.height;

            color = cv::Scalar(0,0,255);

        }
        else
        {
            spdlog::debug("all tracker good");
            cv::Point center = cv::Point{m_strackerRet.x + m_strackerRet.width/2, m_strackerRet.y + m_strackerRet.height/2};
            int ret = m_dtrackerRet.contains(center);
            static int containRetCnt = 0;
            if(!ret)
            {
                containRetCnt++;
                if(containRetCnt > 3)
                {
                    spdlog::debug("reset stracker");
                    m_stracker->reset();
                    m_stracker->init(cv::Point(m_dtrackerRet.x + m_dtrackerRet.width/2, m_dtrackerRet.y + m_dtrackerRet.height/2), frame);
                    containRetCnt = 0;
                }
            }
            else
            {
                containRetCnt = 0;
            }
            

            finalRect = m_dtrackerRet;
            printf("dtracker contains stracker:%d\n", ret);

        }
        // printf("FSM_PROC_DTRACK calculateHistogramSimilarity\n");
        // double sim = calculateHistogramSimilarity(frame(m_trackObj.m_rect), frame(finalRect));
        // printf("FSM_PROC_DTRACK sim:%f\n", sim);
        m_trackObj.update(frame, finalRect, maxSSIM);
    }

    if(m_trackObj.isLost())
    {
        spdlog::debug("tracker lost");
        m_state = EN_TRACKER_FSM::SEARCH;
        return;
    }

#if TRACKER_DEBUG
    cv::imshow("debugImg", debugimg);
#endif
    
    cv::rectangle(frame, derRect, cv::Scalar(255, 0, 127), 2);
    cv::rectangle(frame, m_trackObj.m_rect, color, 2);
    printf("m_trackObj age:%d, lostcnt:%d, trace size:%d, velo x:%f, velo y:%f\n", 
        m_trackObj.m_age, m_trackObj.m_lostCnt, m_trackObj.m_trace.size(), m_trackObj.m_velo[0], m_trackObj.m_velo[1]);
    std::cout<<m_trackObj.m_rect<<std::endl;

    m_state = EN_TRACKER_FSM::DTRACK;
}

void realtracker::fsmUpdate(cv::Mat &frame)
{
    
    switch(m_state)
    {
        case EN_TRACKER_FSM::STRACK:
            FSM_PROC_STRACK(frame);
            break;
        case EN_TRACKER_FSM::DTRACK:
            FSM_PROC_DTRACK(frame);
            break;  
        case EN_TRACKER_FSM::SEARCH:
            FSM_PROC_SEARCH(frame);
            break;
        default:
            break;
    }
    
}

EN_TRACKER_FSM realtracker::update(cv::Mat &frame, std::vector<TrackingObject> &detRet, uint8_t *trackerStatus)
{
    return m_state;
}

EN_TRACKER_FSM realtracker::update(cv::Mat &frame, std::vector<bbox_t> &detRet, uint8_t *trackerStatus)
{
    printf("realtracker::update start\n");

    fsmUpdate(frame);
    // detRet = m_frameInfo.m_tracks[0];

    // uint8_t trackerStatus[9];
    memset(trackerStatus, 0, 9);

    // if(m_state == EN_TRACKER_FSM::STRACK || m_state == EN_TRACKER_FSM::DTRACK)
    if(m_state == EN_TRACKER_FSM::DTRACK || m_state == EN_TRACKER_FSM::STRACK)
    {
        trackerStatus[4] |= 0x02;   //0000 0010
        if(m_state == EN_TRACKER_FSM::DTRACK)
        {
            trackerStatus[4] |= 0x04;   //0000 0100
            cv::Rect bbox = m_trackObj.m_rect;
            uint16_t w = bbox.width;
            uint16_t h = bbox.height;
            w = ntohs(w);
            h = ntohs(h);
            memcpy(trackerStatus+5, &w, 2);
            memcpy(trackerStatus+7, &h, 2);
        }

        int16_t x = m_trackObj.center().x - 960;
        int16_t y = m_trackObj.center().y - 540;
        // int16_t x = m_strackerRet.x+m_stracker->m_GateSize/2- 960;
        // int16_t y = m_strackerRet.y+m_stracker->m_GateSize/2 - 540;

        if(x > m_trackerOffsetLimit)
            x = m_trackerOffsetLimit;
        if(x < -m_trackerOffsetLimit)
            x = -m_trackerOffsetLimit;
        if(y > m_trackerOffsetLimit)
            y = m_trackerOffsetLimit;
        if(y < -m_trackerOffsetLimit)
            y = -m_trackerOffsetLimit;

        // printf("tracker center pt x:%d, y:%d\n", m_stracker->centerPt().x, m_stracker->centerPt().y);
        printf("tracker offset x:%d, y:%d\n", x, y);

        x = ntohs(x);
        y = ntohs(y);

        // printf("tracker offset after ntohs x:%d, y:%d\n", x, y);
        memcpy(trackerStatus, &x, 2);
        memcpy(trackerStatus+2, &y, 2);

    }
    else if(m_state == EN_TRACKER_FSM::SEARCH)
    {
        printf("realtracker::update in search\n");
        trackerStatus[4] |= 0x02;   //0000 0010
        int16_t x = 0;
        int16_t y = 0;
        memcpy(trackerStatus, &x, 2);
        memcpy(trackerStatus+2, &y, 2);
    }

    return m_state;

}

void realtracker::runTracker(cv::Mat &frame, bool alone)
{
    // printf("realtracker::runTracker\n");
    cv::Rect kcfResult, templateRet;
    // cv::Point ScreenCenter = cv::Point(960,540);
    kcfResult = m_stracker->update(frame, alone);
#if TRACKER_DEBUG
    cv::Mat strakerRet = frame.clone(); 
    rectangle(strakerRet, cv::Point(kcfResult.x, kcfResult.y ), cv::Point( kcfResult.x+kcfResult.width, kcfResult.y+kcfResult.height), cv::Scalar(145,79,59), 3, 8 );
    cv::imshow("strakerRet", strakerRet);
#endif
    // double kcfDist = getDistance(cv::Point(kcfResult.tl().x + m_stracker->m_GateSize/2, kcfResult.tl().y + m_stracker->m_GateSize/2), ScreenCenter);
    // templateRet = m_stracker->updateTP(frame);
    // double templateDist = getDistance(cv::Point(templateRet.tl().x + m_stracker->m_GateSize/2, templateRet.tl().y + m_stracker->m_GateSize/2), ScreenCenter);
    // rectangle(frame, cv::Point(templateRet.x, templateRet.y ), cv::Point( templateRet.x+templateRet.width, templateRet.y+templateRet.height), cv::Scalar(255,255,255), 2, 8 );
    // printf("stracker rect, kcfdist:%f, templatedist:%f\n", kcfDist, templateDist);
    // std::cout<<result<<std::endl;

    // if(kcfDist > templateDist)
    // {
    //     m_stracker->reset();
    //     m_stracker->init(templateRet, frame);
    //     m_strackerRet = templateRet;
    // }
    // else
    // {
    //     m_strackerRet = kcfResult;
    //     cv::Mat temp = frame(kcfResult);
    //     m_stracker->resetTemplate(temp);
    // }

    m_strackerRet = kcfResult;

    // m_strackerRet = (kcfDist > templateDist ? templateRet : kcfResult);

}
void realtracker::runTrackerNoDraw(cv::Mat &frame, bool alone)
{
    // printf("realtracker::runTracker\n");
    // cv::Rect result;
    m_strackerRet = m_stracker->update(frame, alone);

}

static inline int boxDif(bbox_t &box1, bbox_t &box2)
{
    // int i = abs(box1.x - box2.x);
    return (abs(int(box1.x - box2.x)) + abs(int(box1.y - box2.y)) + abs(int(box1.w - box2.w)) + abs(int(box1.h - box2.h))); 

}

void realtracker::runDetector(cv::Mat &frame, std::vector<bbox_t> &detRet)
{
    printf("realtracker::runDetector\n");
    std::vector<bbox_t> boxs;
    cv::Mat finalDet, rawDet;
    rawDet = frame.clone();
    
    m_frameInfo.m_frames[0].GetMatBGRWrite() = rawDet;
    // m_frameInfo.m_frames[0].GetMatBGRWrite() = frame.clone();
    m_detector->process(rawDet, boxs);
    for(int i=0;i<boxs.size();i++)
    {
        // printf("box-->x:%d, y:%d, w:%d, h:%d, conf:%f, cls:%d\n", boxs[i].x, boxs[i].y, boxs[i].w, boxs[i].h, boxs[i].prob, boxs[i].obj_id);
        for(int j=i+1;j<boxs.size();++j)
        {
            // printf("\tbox-->x:%d, y:%d, w:%d, h:%d, conf:%f, cls:%d, diff:%d\n", boxs[j].x, boxs[j].y, boxs[j].w, boxs[j].h, boxs[j].prob, boxs[j].obj_id, boxDif(boxs[i], boxs[j]));

            if(boxDif(boxs[i], boxs[j]) < 5)
            {
                boxs.erase(boxs.begin()+j);
                --j;
            }
        }
    }
    
    cv::imshow("rawDet", rawDet);

    detRet = boxs;

    return;
    // detret = detFrame.clone();
    // m_frameInfo.CleanRegions();

    // // printf("frameInfo.m_regions[0] size%d, boxs :%d\n", frameInfo.m_regions[0].size(), boxs.size());
    // // for(auto& box:boxs)
    // // {
    // // 	printf("box-->x:%d, y:%d, w:%d, h:%d, conf:%f, cls:%d\n", box.x, box.y, box.w, box.h, box.prob, box.obj_id);
    // // }
    // m_regions.clear();
    // for(auto &box:boxs)
    // {
    //     m_regions.emplace_back(cv::Rect(cvRound(1.0*box.x), cvRound(1.0*box.y), cvRound(1.0*box.w), cvRound(1.0*box.h)), (box.obj_id), box.prob);
    // }

    // printf("frameInfo.m_regions[0] size%d, regions:%d\n", m_frameInfo.m_regions[0].size(), m_regions.size());
    // m_frameInfo.m_regions[0] = m_regions;
    // m_mtracker->Update(m_frameInfo.m_regions[0], m_frameInfo.m_frames[0].GetUMatGray(), m_fps);
    // printf("track size:%d\n", m_frameInfo.m_tracks[0].size());
    // m_mtracker->GetTracks(m_frameInfo.m_tracks[0]);
    // detRet = m_frameInfo.m_tracks[0];

    // DrawData(m_frameInfo.m_frames[0].GetMatBGR(), m_frameInfo.m_tracks[0], m_frameInfo.m_frameInds[0], 0);
    // // frame = frameInfo.m_frames[0].GetMatBGR().clone();
    // finalDet = m_frameInfo.m_frames[0].GetMatBGR();

    // cv::imshow("finaldet", finalDet);
}
void realtracker::runDetectorOut(cv::Mat &frame, std::vector<bbox_t> &detRet)
{
    printf("realtracker::runDetector\n");
    std::vector<bbox_t> boxs;
    
    // m_frameInfo.m_frames[0].GetMatBGRWrite() = frame.clone();
    m_detector->process(frame, boxs);
    for(int i=0;i<boxs.size();i++)
    {
        // printf("box-->x:%d, y:%d, w:%d, h:%d, conf:%f, cls:%d\n", boxs[i].x, boxs[i].y, boxs[i].w, boxs[i].h, boxs[i].prob, boxs[i].obj_id);
        for(int j=i+1;j<boxs.size();++j)
        {
            // printf("\tbox-->x:%d, y:%d, w:%d, h:%d, conf:%f, cls:%d, diff:%d\n", boxs[j].x, boxs[j].y, boxs[j].w, boxs[j].h, boxs[j].prob, boxs[j].obj_id, boxDif(boxs[i], boxs[j]));

            if(boxDif(boxs[i], boxs[j]) < 5)
            {
                boxs.erase(boxs.begin()+j);
                --j;
            }
        }
    }

    for(auto &box:boxs)
    {
        cv::rectangle(frame, cv::Point(box.x, box.y ), cv::Point(box.x+box.w, box.y+box.h), cv::Scalar(0,255,255), 2, 8 );

    }
    

    detRet = boxs;

    return;
}
void realtracker::runDetectorNoDraw(cv::Mat &frame, std::vector<bbox_t> &detRet)
{
    printf("realtracker::runDetector\n");
    // std::vector<bbox_t> boxs;
    detRet.clear();
    m_frameInfo.m_frames[0].GetMatBGRWrite() = frame;
    // m_frameInfo.m_frames[0].GetMatBGRWrite() = frame.clone();
    m_detector->process(frame, detRet);
    for(int i=0;i<detRet.size();i++)
    {
        // printf("box-->x:%d, y:%d, w:%d, h:%d, conf:%f, cls:%d\n", boxs[i].x, boxs[i].y, boxs[i].w, boxs[i].h, boxs[i].prob, boxs[i].obj_id);
        for(int j=i+1;j<detRet.size();++j)
        {
            // printf("\tbox-->x:%d, y:%d, w:%d, h:%d, conf:%f, cls:%d, diff:%d\n", boxs[j].x, boxs[j].y, boxs[j].w, boxs[j].h, boxs[j].prob, boxs[j].obj_id, boxDif(boxs[i], boxs[j]));

            if(boxDif(detRet[i], detRet[j]) < 5)
            {
                detRet.erase(detRet.begin()+j);
                --j;
            }
        }
    }
    // cv::Mat rawDet;
    // rawDet = frame.clone();
    // cv::imshow("rawDet", rawDet);
    // detret = detFrame.clone();
    // m_frameInfo.CleanRegions();

    // // printf("frameInfo.m_regions[0] size%d, boxs :%d\n", frameInfo.m_regions[0].size(), boxs.size());
    // // for(auto& box:boxs)
    // // {
    // // 	printf("box-->x:%d, y:%d, w:%d, h:%d, conf:%f, cls:%d\n", box.x, box.y, box.w, box.h, box.prob, box.obj_id);
    // // }
    // m_regions.clear();
    // for(auto &box:boxs)
    // {
    //     m_regions.emplace_back(cv::Rect(cvRound(1.0*box.x), cvRound(1.0*box.y), cvRound(1.0*box.w), cvRound(1.0*box.h)), (box.obj_id), box.prob);
    // }

    // printf("frameInfo.m_regions[0] size%d, regions:%d\n", m_frameInfo.m_regions[0].size(), m_regions.size());
    // m_frameInfo.m_regions[0] = m_regions;
    // m_mtracker->Update(m_frameInfo.m_regions[0], m_frameInfo.m_frames[0].GetUMatGray(), m_fps);
    // printf("track size:%d\n", m_frameInfo.m_tracks[0].size());
    // m_mtracker->GetTracks(m_frameInfo.m_tracks[0]);
    // detRet = m_frameInfo.m_tracks[0];

}

void realtracker::reset()
{
    m_mtrackerLostCnt = 0;
    m_stracker->reset();
}

void realtracker::setFrameScale(double s)
{
    m_frameScale = s;
}

void realtracker::setGateSize(int s)
{
    m_stracker->setGateSize(s);
}

void realtracker::setIrFrame(bool ir)
{
    m_irFrame = ir;
}