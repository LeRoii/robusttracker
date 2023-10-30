#include "realtracker.h"
// #include <stdlib.h>

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
    cv::Scalar color = track.m_isStatic ? cv::Scalar(255, 0, 255) : cv::Scalar(0, 255, 0);
    cv::Point2f rectPoints[4];
    track.m_rrect.points(rectPoints);
    // std::cout << "track.m_rrect: " << track.m_rrect.center << ", " << track.m_rrect.angle << ", " << track.m_rrect.size << std::endl;
    for (int i = 0; i < 4; ++i)
    {
        cv::line(frame, rectPoints[i], rectPoints[(i+1) % 4], color, 2);
    }
    if (drawTrajectory)
    {
        cv::Scalar cl = cv::Scalar(0,255,0);

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


realtracker::realtracker(std::string enginepath):m_frameInfo()
{
    m_kcf = new itracker();
    m_kcf->setGateSize(32);

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

}

realtracker::~realtracker()
{

}

void realtracker::init(const cv::Rect &roi, cv::Mat image)
{
    m_kcf->init(roi, image);
}

inline double getDistance (cv::Point point1, cv::Point point2)
{
    return  sqrtf(powf((point1.x - point2.x),2) + powf((point1.y - point2.y),2));
}


int realtracker::update(cv::Mat &frame, std::vector<TrackingObject> &detRet, cv::Point &pt)
{
    static cv::Mat trackerFrame, detFrame;
    trackerFrame = detFrame = frame.clone();
    printf("realtracker::update\n");
    runDetector(detFrame, detRet);
    runTracker(trackerFrame);

    static int lastId = -1;
    static cv::Point lastPt{0,0};

    if(m_kcf->isLost())
    {
        printf("m_kcf->isLost()\n");
        if(lastId != -1)
        {
            for(auto &obj:m_frameInfo.m_tracks[0])
            {
                if(obj.m_ID == lastId)
                {
                    cv::Point2f rectPoints[4];
        	        obj.m_rrect.points(rectPoints);
                    for (int i = 0; i < 4; ++i)
                    {
                        cv::line(frame, rectPoints[i], rectPoints[(i+1) % 4], cv::Scalar(255, 0, 255), 2);
                    }

                    printf("\n\ntracker reset\n");
                    m_kcf->reset();
                    const cv::Point2f &pt = obj.m_rrect.center;
                    cv::Rect initRect = cv::Rect(pt.x - 16, pt.y - 16, 32, 32);
                    m_kcf->init(initRect, frame);
                    break;
                }
            }
        }
        else
        {
            printf("tracker lost\n\n");
            return 0;
        }
    }
    else
    {
        int minIdx = -1;
        m_kcf->centerPt();

        double minDist = 10000.f;
        for(int i=0; i< m_frameInfo.m_tracks[0].size(); ++i)
        {
            if(lastId == m_frameInfo.m_tracks[0][i].m_ID.m_val)
            {
                minIdx = i;
                break;
            }

        	cv::Rect brect = m_frameInfo.m_tracks[0][i].m_rrect.boundingRect();
        	cv::Point center{brect.tl().x + brect.width/2, brect.tl().y + brect.height/2};
        	double dist = getDistance(m_kcf->centerPt(), center);
        	// printf("obj pos:(%d, %d), dist:%f\n", brect.tl().x, brect.tl().y, dist);
        	if(dist < minDist)
        	{
        		minDist = dist;
        		minIdx = i;
        	}
        }

        printf("minIdx = %d\n", minIdx);
        lastId = -1;
        if(minIdx != -1)
        {
            cv::circle(frame, m_kcf->centerPt(), 2, cv::Scalar(0,255,255), 2);

            if(!m_frameInfo.m_tracks[0][minIdx].m_rrect.boundingRect().contains(m_kcf->centerPt()))
            {
                printf("\n\ntracker reset\n");
                m_kcf->reset();
                m_kcf->init(m_frameInfo.m_tracks[0][minIdx].m_rrect.center, frame);
            }

        	cv::Point2f rectPoints[4];
        	m_frameInfo.m_tracks[0][minIdx].m_rrect.points(rectPoints);
        	for (int i = 0; i < 4; ++i)
        	{
        		cv::line(frame, rectPoints[i], rectPoints[(i+1) % 4], cv::Scalar(255, 0, 255), 2);
        	}

            lastId = m_frameInfo.m_tracks[0][minIdx].m_ID.m_val;
        }
    }

    pt.x = m_kcf->centerPt().x - lastPt.x;
    pt.y = m_kcf->centerPt().y - lastPt.y;
    lastPt = m_kcf->centerPt();

    return 1;

    

    // 	contain = frameInfo.m_tracks[0][minIdx].m_rrect.boundingRect().contains(userPt);
    // 	spdlog::debug("contains:{}", contain);
    // }
}

void realtracker::runTracker(cv::Mat &frame)
{
    // printf("realtracker::runTracker\n");
    cv::Rect result;
    result = m_kcf->update(frame);
    rectangle(frame, cv::Point(result.x, result.y ), cv::Point( result.x+result.width, result.y+result.height), cv::Scalar( 255,0,0 ), 2, 8 );
    // m_kcfRet.x = result.x + m_GateSize/2;
	// m_kcfRet.y = result.y + m_GateSize/2;

}

static inline int boxDif(bbox_t &box1, bbox_t &box2)
{
    // int i = abs(box1.x - box2.x);
    return (abs(int(box1.x - box2.x)) + abs(int(box1.y - box2.y)) + abs(int(box1.w - box2.w)) + abs(int(box1.h - box2.h))); 

}

void realtracker::runDetector(cv::Mat &frame, std::vector<TrackingObject> &detRet)
{
    printf("realtracker::runDetector\n");
    std::vector<bbox_t> boxs;
    m_frameInfo.m_frames[0].GetMatBGRWrite() = frame;
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
    // cv::Mat rawDet;
    // rawDet = frame.clone();
    // cv::imshow("rawDet", rawDet);
    // detret = detFrame.clone();
    m_frameInfo.CleanRegions();

    // printf("frameInfo.m_regions[0] size%d, boxs :%d\n", frameInfo.m_regions[0].size(), boxs.size());
    // for(auto& box:boxs)
    // {
    // 	printf("box-->x:%d, y:%d, w:%d, h:%d, conf:%f, cls:%d\n", box.x, box.y, box.w, box.h, box.prob, box.obj_id);
    // }
    m_regions.clear();
    for(auto &box:boxs)
    {
        m_regions.emplace_back(cv::Rect(cvRound(1.0*box.x), cvRound(1.0*box.y), cvRound(1.0*box.w), cvRound(1.0*box.h)), (box.obj_id), box.prob);
    }

    printf("frameInfo.m_regions[0] size%d, regions:%d\n", m_frameInfo.m_regions[0].size(), m_regions.size());
    m_frameInfo.m_regions[0] = m_regions;
    m_mtracker->Update(m_frameInfo.m_regions[0], m_frameInfo.m_frames[0].GetUMatGray(), m_fps);
    printf("track size:%d\n", m_frameInfo.m_tracks[0].size());
    m_mtracker->GetTracks(m_frameInfo.m_tracks[0]);
    detRet = m_frameInfo.m_tracks[0];

    DrawData(m_frameInfo.m_frames[0].GetMatBGR(), m_frameInfo.m_tracks[0], m_frameInfo.m_frameInds[0], 0);
    // frame = frameInfo.m_frames[0].GetMatBGR().clone();
    frame = m_frameInfo.m_frames[0].GetMatBGR();
}

void realtracker::reset()
{
    m_kcf->reset();
}