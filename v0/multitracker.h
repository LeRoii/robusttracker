#ifndef _MULTITRACKER_H_
#define _MULTITRACKER_H_

#include <opencv2/imgproc/types_c.h>
#include <mtracking/Ctracker.h>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <chrono>
#include <condition_variable>
#include <thread>
#include <atomic>

/************************mt start*********************************/

///
/// \brief The Frame struct
///
class Frame
{
public:
    Frame() = default;
    Frame(cv::Mat imgBGR)
    {
        m_mBGR = imgBGR;
    }

    ///
    bool empty() const
    {
        return m_mBGR.empty();
    }

    ///
    const cv::Mat& GetMatBGR()
    {
        return m_mBGR;
    }
    ///
    cv::Mat& GetMatBGRWrite()
    {
        m_umBGRGenerated = false;
        m_mGrayGenerated = false;
        m_umGrayGenerated = false;
        return m_mBGR;
    }
    ///
    const cv::Mat& GetMatGray()
    {
        if (m_mGray.empty() || !m_mGrayGenerated)
        {
            if (m_umGray.empty() || !m_umGrayGenerated)
                cv::cvtColor(m_mBGR, m_mGray, cv::COLOR_BGR2GRAY);
            else
                m_mGray = m_umGray.getMat(cv::ACCESS_READ);
            m_mGrayGenerated = true;
        }
        return m_mGray;
    }
    ///
    const cv::UMat& GetUMatBGR()
    {
        std::thread::id lastThreadID = std::this_thread::get_id();

        if (m_umBGR.empty() || !m_umBGRGenerated || lastThreadID != m_umBGRThreadID)
        {
            m_umBGR = m_mBGR.getUMat(cv::ACCESS_READ);
            m_umBGRGenerated = true;
            m_umBGRThreadID = lastThreadID;
        }
        return m_umBGR;
    }
    ///
    const cv::UMat& GetUMatGray()
    {
        std::thread::id lastThreadID = std::this_thread::get_id();

        if (m_umGray.empty() || !m_umGrayGenerated || lastThreadID != m_umGrayThreadID)
        {
            if (m_mGray.empty() || !m_mGrayGenerated)
            {
                if (m_umBGR.empty() || !m_umBGRGenerated || lastThreadID != m_umGrayThreadID)
                    cv::cvtColor(m_mBGR, m_umGray, cv::COLOR_BGR2GRAY);
                else
                    cv::cvtColor(m_umBGR, m_umGray, cv::COLOR_BGR2GRAY);
            }
            else
            {
                m_umGray = m_mGray.getUMat(cv::ACCESS_READ);
            }
            m_umGrayGenerated = true;
            m_umGrayThreadID = lastThreadID;
        }
        return m_umGray;
    }

private:
    cv::Mat m_mBGR;
    cv::Mat m_mGray;
    cv::UMat m_umBGR;
    cv::UMat m_umGray;
    bool m_umBGRGenerated = false;
    bool m_mGrayGenerated = false;
    bool m_umGrayGenerated = false;
    std::thread::id m_umBGRThreadID;
    std::thread::id m_umGrayThreadID;
};

///
/// \brief The FrameInfo struct
///
struct FrameInfo
{
    ///
    FrameInfo()
    {
        m_frames.reserve(m_batchSize);
        m_regions.reserve(m_batchSize);
        m_frameInds.reserve(m_batchSize);
    }
    ///
    FrameInfo(size_t batchSize)
        : m_batchSize(batchSize)
    {
        m_frames.reserve(m_batchSize);
        m_regions.reserve(m_batchSize);
        m_frameInds.reserve(m_batchSize);
    }

    ///
    void SetBatchSize(size_t batchSize)
    {
        m_batchSize = batchSize;
        m_frames.reserve(m_batchSize);
        m_regions.reserve(m_batchSize);
        m_frameInds.reserve(m_batchSize);
    }

    ///
    void CleanRegions()
    {
        if (m_regions.size() != m_batchSize)
            m_regions.resize(m_batchSize);
        for (auto& regions : m_regions)
        {
            regions.clear();
        }
    }

    ///
    void CleanTracks()
    {
        if (m_tracks.size() != m_batchSize)
            m_tracks.resize(m_batchSize);
        for (auto& tracks : m_tracks)
        {
            tracks.clear();
        }
    }

    std::vector<Frame> m_frames;
    std::vector<regions_t> m_regions;
    std::vector<std::vector<TrackingObject>> m_tracks;
    std::vector<int> m_frameInds;

    size_t m_batchSize = 1;

    int64 m_dt = 0;

    std::condition_variable m_cond;
    std::mutex m_mutex;
    std::atomic<bool> m_captured { false };
};

// struct bbox_t {
//     unsigned int x, y, w, h;       // (x,y) - top-left corner, (w, h) - width & height of bounded box
//     float prob;                    // confidence - probability that the object was found correctly
//     unsigned int obj_id;           // class of object - from range [0, classes-1]
//     unsigned int track_id;         // tracking id for video (0 - untracked, 1 - inf - tracked object)
//     unsigned int frames_counter;   // counter of frames on which the object was detected
//     float x_3d, y_3d, z_3d;        // center of object (in Meters) if ZED 3D Camera is used
// };

void DrawFilledRect(cv::Mat& frame, const cv::Rect& rect, cv::Scalar cl, int alpha);
// {
//     if (alpha)
//     {
//         const int alpha_1 = 255 - alpha;
//         const int nchans = frame.channels();
//         int color[3] = { cv::saturate_cast<int>(cl[0]), cv::saturate_cast<int>(cl[1]), cv::saturate_cast<int>(cl[2]) };
//         for (int y = rect.y; y < rect.y + rect.height; ++y)
//         {
//             uchar* ptr = frame.ptr(y) + nchans * rect.x;
//             for (int x = rect.x; x < rect.x + rect.width; ++x)
//             {
//                 for (int i = 0; i < nchans; ++i)
//                 {
//                     ptr[i] = cv::saturate_cast<uchar>((alpha_1 * ptr[i] + alpha * color[i]) / 255);
//                 }
//                 ptr += nchans;
//             }
//         }
//     }
//     else
//     {
//         // cv::rectangle(frame, rect, cl, cv::FILLED);
//         cv::rectangle(frame, rect, cl, 2);
//     }
// }

void DrawTrack(cv::Mat frame,
                             const TrackingObject& track,
                             bool drawTrajectory,
                             int framesCounter);
// {

//     // printf("DrawTrack\n");
//     cv::Scalar color = track.m_isStatic ? cv::Scalar(255, 0, 255) : cv::Scalar(0, 255, 0);
//     cv::Point2f rectPoints[4];
//     track.m_rrect.points(rectPoints);
//     // std::cout << "track.m_rrect: " << track.m_rrect.center << ", " << track.m_rrect.angle << ", " << track.m_rrect.size << std::endl;
//     for (int i = 0; i < 4; ++i)
//     {
//         cv::line(frame, rectPoints[i], rectPoints[(i+1) % 4], color, 2);
//     }
//     if (drawTrajectory)
//     {
//         cv::Scalar cl = cv::Scalar(0,255,0);

//         for (size_t j = 0; j < track.m_trace.size() - 1; ++j)
//         {
//             const TrajectoryPoint& pt1 = track.m_trace.at(j);
//             const TrajectoryPoint& pt2 = track.m_trace.at(j + 1);
// #if (CV_VERSION_MAJOR >= 4)
//             cv::line(frame, pt1.m_prediction, pt2.m_prediction, cl, 1, cv::LINE_AA);
// #else
//             cv::line(frame, pt1.m_prediction, pt2.m_prediction, cl, 1, CV_AA);
// #endif
//             if (!pt2.m_hasRaw)
//             {
// #if (CV_VERSION_MAJOR >= 4)
//                 cv::circle(frame, pt2.m_prediction, 4, cl, 1, cv::LINE_AA);
// #else
//                 cv::circle(frame, pt2.m_prediction, 4, cl, 1, CV_AA);
// #endif
//             }
//         }
//     }

//     // printf("111111111111\n");
//     cv::Rect brect = track.m_rrect.boundingRect();
//     std::string label = track.m_ID.ID2Str();
//     // if (track.m_type != bad_type)
//     //     label += " (" + TypeConverter::Type2Str(track.m_type) + ")";

//     // printf("2222222222\n");
//     int baseLine = 0;
//     double fontScale = (frame.cols < 1920) ? 0.5 : 0.7;
//     cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_TRIPLEX, fontScale, 1, &baseLine);
//     if (brect.x < 0)
//     {
//         brect.width = std::min(brect.width, frame.cols - 1);
//         brect.x = 0;
//     }
//     else if (brect.x + brect.width >= frame.cols)
//     {
//         brect.x = std::max(0, frame.cols - brect.width - 1);
//         brect.width = std::min(brect.width, frame.cols - 1);
//     }
//     if (brect.y - labelSize.height < 0)
//     {
//         brect.height = std::min(brect.height, frame.rows - 1);
//         brect.y = labelSize.height;
//     }
//     else if (brect.y + brect.height >= frame.rows)
//     {
//         brect.y = std::max(0, frame.rows - brect.height - 1);
//         brect.height = std::min(brect.height, frame.rows - 1);
//     }
//     // printf("m_isStatic\n");
//     // DrawFilledRect(frame, cv::Rect(cv::Point(brect.x, brect.y - labelSize.height), cv::Size(labelSize.width, labelSize.height + baseLine)), cv::Scalar(200, 200, 200), 0);
//     // cv::putText(frame, label+ std::to_string(track.m_confidence), brect.tl(), cv::FONT_HERSHEY_TRIPLEX, fontScale, cv::Scalar(0, 0, 0));
//     cv::putText(frame, label, brect.tl(), cv::FONT_HERSHEY_TRIPLEX, fontScale, cv::Scalar(0, 0, 0));

// }

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

void DrawData(cv::Mat frame, const std::vector<TrackingObject>& tracks, int framesCounter, int currTime);
// {

//     for (const auto& track : tracks)
//     {

//         // printf("track id:%d, velo x:%f, velo y:%f, speed:%f, type:%d, isStatic:%d, statis time:%d, out of frame:%d,\
//         // lastRobust:%d\n", track.m_ID, track.m_velocity[0], track.m_velocity[1], sqrt(sqr(track.m_velocity[0]) + sqr(track.m_velocity[1])), track.m_type, track.m_isStatic,\
//         // track.m_isStaticTime, track.m_outOfTheFrame, track.m_lastRobust);
//         if (track.m_isStatic)
//         {
//             // printf("m_isStatic\n");
//             DrawTrack(frame, track, false, framesCounter);

//             std::string label = "abandoned " + track.m_ID.ID2Str();
//             int baseLine = 0;
//             cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_TRIPLEX, 0.5, 1, &baseLine);

//             cv::Rect brect = track.m_rrect.boundingRect();
//             if (brect.x < 0)
//             {
//                 brect.width = std::min(brect.width, frame.cols - 1);
//                 brect.x = 0;
//             }
//             else if (brect.x + brect.width >= frame.cols)
//             {
//                 brect.x = std::max(0, frame.cols - brect.width - 1);
//                 brect.width = std::min(brect.width, frame.cols - 1);
//             }
//             if (brect.y - labelSize.height < 0)
//             {
//                 brect.height = std::min(brect.height, frame.rows - 1);
//                 brect.y = labelSize.height;
//             }
//             else if (brect.y + brect.height >= frame.rows)
//             {
//                 brect.y = std::max(0, frame.rows - brect.height - 1);
//                 brect.height = std::min(brect.height, frame.rows - 1);
//             }
//             DrawFilledRect(frame, cv::Rect(cv::Point(brect.x, brect.y - labelSize.height), cv::Size(labelSize.width, labelSize.height + baseLine)), cv::Scalar(255, 0, 255), 150);
//             cv::putText(frame, label, brect.tl(), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 0));
//         }
//         else
//         {
//             auto velocity = sqrt(sqr(track.m_velocity[0]) + sqr(track.m_velocity[1]));

//             // printf("!!!!m_isStatic\n");
//             // if (track.IsRobust(4,             // Minimal trajectory size
//             // 	0.3f,                         // Minimal ratio raw_trajectory_points / trajectory_lenght
//             // 	cv::Size2f(0.2f, 5.0f)) &&    // Min and max ratio: width / height
//             // 	velocity > 30)                // Velocity more than 30 pixels per second
//             {
//                 track_t mean = 0;
//                 track_t stddev = 0;
//                 TrackingObject::LSParams lsParams;
//                 // if (track.LeastSquares2(20, mean, stddev, lsParams) && mean > stddev)
//                 {
//                     DrawTrack(frame, track, false, framesCounter);
//                 }
//             }
//         }
//     }
// }


void genTrackerSettings(TrackerSettings &settings);
// {
//     cv::Mat tmp = cv::Mat(720, 1280, CV_8UC3);
//     FrameInfo frameInfo(1);
// 	frameInfo.m_frames.resize(frameInfo.m_batchSize);
// 	frameInfo.m_frameInds.resize(frameInfo.m_batchSize);
//     frameInfo.m_frames[0].GetMatBGRWrite() = tmp;
//     cv::UMat umatFrame = frameInfo.m_frames[0].GetUMatBGR();
//     float m_fps = 25;
//     // settings.SetDistance(tracking::DistJaccard);
//     settings.SetDistance(tracking::DistCenters);
//     settings.m_kalmanType = tracking::KalmanLinear;
//     settings.m_filterGoal = tracking::FilterCenter;
//     settings.m_lostTrackType = tracking::TrackCSRT; // Use KCF tracker for collisions resolving. Used if m_filterGoal == tracking::FilterRect
//     settings.m_matchType = tracking::MatchHungrian;
//     settings.m_dt = 0.3f;                           // Delta time for Kalman filter
//     settings.m_accelNoiseMag = 0.2f;                // Accel noise magnitude for Kalman filter
//     settings.m_distThres = 0.8f;                    // Distance threshold between region and object on two frames
//     settings.m_minAreaRadiusPix = umatFrame.rows / 20.f;
//     settings.m_maximumAllowedSkippedFrames = cvRound(2 * m_fps); // Maximum allowed skipped frames
//     settings.m_maxSpeedForStatic = 10;

//     settings.m_useAbandonedDetection = false;
//     if (settings.m_useAbandonedDetection)
//     {
//         settings.m_minStaticTime = 5;
//         settings.m_maxStaticTime = 60;
//         settings.m_maximumAllowedSkippedFrames = cvRound(settings.m_minStaticTime * m_fps); // Maximum allowed skipped frames
//         settings.m_maxTraceLength = 2 * settings.m_maximumAllowedSkippedFrames;        // Maximum trace length
//     }
//     else
//     {
//         settings.m_maximumAllowedSkippedFrames = cvRound(10 * m_fps); // Maximum allowed skipped frames
//         settings.m_maxTraceLength = cvRound(4 * m_fps);              // Maximum trace length
//     }
//     // settings.SetDistance(tracking::DistJaccard);
//     settings.SetDistance(tracking::DistCenters);
// }

void prepareCrosshair(cv::Mat &oriCrosshair);
// {
//     int crosshairW = 150;
// 	int crosshairH = 100;
// 	oriCrosshair = cv::Mat(crosshairH, crosshairW, CV_8UC3);
// 	oriCrosshair.setTo(0);

// 	int w = oriCrosshair.cols;
// 	int h = oriCrosshair.rows;

// 	auto centerPt = cv::Point(w/2, h/2);

// 	auto color = cv::Scalar(0,0,255);
// 	auto thickness = 4;
// 	cv::circle(oriCrosshair, centerPt, 1, color, 2);

// 	cv::Point leftPtStart = cv::Point(0, centerPt.y);
// 	cv::Point leftPtEnd = cv::Point(0 + w/2 - 10, centerPt.y);

// 	cv::Point upPtStart = cv::Point(centerPt.x, 0);
// 	cv::Point upPtEnd = cv::Point(centerPt.x, h/2 - 10);

// 	cv::Point rightPtStart = cv::Point(w/2 + 10, centerPt.y);
// 	cv::Point rightPtEnd = cv::Point(w, centerPt.y);

// 	cv::Point downPtStart = cv::Point(centerPt.x, h/2 + 10);
// 	cv::Point downPtEnd = cv::Point(centerPt.x, h);

// 	//crosshair
// 	cv::line(oriCrosshair, leftPtStart, leftPtEnd, color, thickness);
// 	cv::line(oriCrosshair, upPtStart, upPtEnd, color, thickness);
// 	cv::line(oriCrosshair, rightPtStart, rightPtEnd, color, thickness);
// 	cv::line(oriCrosshair, downPtStart, downPtEnd, color, thickness);


// 	int cornerW = 30;
// 	int cornerH = 35;
// 	int leftupX = 20;
// 	int leftupY = 10;
// 	int leftdownX = leftupX;
// 	int leftdownY = h - leftupY;
// 	int rightupX = w - leftupX;
// 	int rightupY = leftupY;
// 	int rightdownX = rightupX;
// 	int rightdownY = leftdownY;
	
// 	//left up corner
// 	cv::line(oriCrosshair, cv::Point(leftupX,leftupY), cv::Point(leftupX, leftupY + cornerH), color, thickness);
// 	cv::line(oriCrosshair, cv::Point(leftupX,leftupY), cv::Point(leftupX + cornerW,leftupY), color, thickness);

// 	//left down corner
// 	cv::line(oriCrosshair, cv::Point(leftdownX, leftdownY), cv::Point(leftdownX, leftdownY - cornerH), color, thickness);
// 	cv::line(oriCrosshair, cv::Point(leftdownX, leftdownY), cv::Point(leftdownX + cornerW, leftdownY), color, thickness);

// 	//right up corner
// 	cv::line(oriCrosshair, cv::Point(rightupX,rightupY), cv::Point(rightupX - cornerW, rightupY), color, thickness);
// 	cv::line(oriCrosshair, cv::Point(rightupX,rightupY), cv::Point(rightupX, rightupY + cornerH), color, thickness);

// 	//right down corner
// 	cv::line(oriCrosshair, cv::Point(rightdownX,rightdownY), cv::Point(rightdownX - cornerW, rightdownY), color, thickness);
// 	cv::line(oriCrosshair, cv::Point(rightdownX,rightdownY), cv::Point(rightdownX, rightdownY - cornerH), color, thickness);

// }

//input para pt is rect centerpoint
void drawCrosshair(cv::Mat &frame, cv::Point pt, double scale = 1);
// {
//     static cv::Mat oriCrosshair;
//     if(oriCrosshair.empty())
//         prepareCrosshair(oriCrosshair);

// 	cv::Mat mask;// = imgi.clone();
// 	// mask.setTo(1);

// 	cv::Mat resizedTemplate;
// 	cv::resize(oriCrosshair, resizedTemplate, cv::Size(), scale, scale);

// 	// printf("w:%d,h:%d\n", resizedTemplate.cols, resizedTemplate.rows);
// 	// cv::imwrite("111.png", resizedTemplate);
//     // mask = cv::Mat::zeros(resizedTemplate.size(), CV_8UC1);
// 	mask = resizedTemplate.clone();
//     cv::cvtColor(mask, mask, CV_RGB2GRAY);
// 	if(pt.x <= resizedTemplate.cols/2)
// 		pt.x = resizedTemplate.cols/2;
// 	if(pt.y <= resizedTemplate.rows/2)
// 		pt.y = resizedTemplate.rows/2;
// 	if(pt.x >= (frame.cols - resizedTemplate.cols/2))
// 		pt.x = frame.cols - resizedTemplate.cols/2 - 1;
// 	if(pt.y >= (frame.rows - resizedTemplate.rows/2))
// 		pt.y = frame.rows - resizedTemplate.rows/2 - 1;
// 	resizedTemplate.copyTo(frame(cv::Rect(pt.x - resizedTemplate.cols/2, pt.y - resizedTemplate.rows/2, resizedTemplate.cols, resizedTemplate.rows)), mask);
	
// }
/************************mt end*********************************/

#endif