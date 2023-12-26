#ifndef _CAMERA_H_
#define _CAMERA_H_

#include <opencv2/opencv.hpp>

enum class enCamType
{
    UNKNOWN = 0,
    SDI = 1,
    ETH = 0,
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

class CameraSDI : public Camera
{
public:
    CameraSDI(const std::string cfgPath);
    virtual ~CameraSDI();
    virtual int Init();
    virtual void GetFrame(cv::Mat &frame0, cv::Mat &frame1);

private:
    bool m_usb;
    cv::VideoCapture m_viCap;

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

class CameraUsb : public Camera
{
public:
    CameraUsb(const std::string cfgPath);
    virtual ~CameraUsb();
    virtual int Init();
    virtual void GetFrame(cv::Mat &frame0, cv::Mat &frame1);

private:
    cv::VideoCapture m_viCap;

};

Camera* CreateCamera(const std::string cfgPath);


#endif