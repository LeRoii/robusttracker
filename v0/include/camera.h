#ifndef _CAMERA_H_
#define _CAMERA_H_

#include <opencv2/opencv.hpp>
#include "v4l2_capture.hpp"

enum class enCamType
{
    UNKNOWN = 0,
    SDI = 1,
    ETH = 0,
    MIPI = 2,
};


class Camera
{
public:

    Camera(const std::string cfgPath, const enCamType camType = enCamType::UNKNOWN);
    virtual ~Camera();
    virtual int Init() = 0;
    virtual void GetFrame(cv::Mat &frame0, cv::Mat &frame1) = 0;
    enCamType GetType();

protected:
    std::string m_cfg;
    enCamType m_type; 
    // std::string m_irStreamAdd;
    // std::string m_visStreamAdd;
    // cv::VideoCapture m_viCap;
    // cv::VideoCapture m_irCap;
};

class CameraEth : public Camera
{
public:
    CameraEth(const std::string cfgPath);
    virtual ~CameraEth();
    virtual int Init();
    virtual void GetFrame(cv::Mat &frame0, cv::Mat &frame1);


private:
    cv::VideoCapture m_viCap;
    cv::VideoCapture m_irCap;
};


class CameraMIPI : public Camera
{
public:
    CameraMIPI(const std::string cfgPath);
    virtual ~CameraMIPI();
    virtual int Init();
    virtual void GetFrame(cv::Mat &frame0, cv::Mat &frame1);


private:
    V4L2Capture *capture;
};

Camera* CreateCamera(const std::string cfgPath);


#endif