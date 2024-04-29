#ifndef _CAMERA_H_
#define _CAMERA_H_

#include "V4L2_camera.hpp"
#include "USB_camera.hpp"
#include <mutex>
#include <time.h>
#include <pthread.h>
#include <stdbool.h>
#include <string.h>
#include "guidescusb2.h"

enum class enCamType
{
    UNKNOWN = 0,
    SDI = 1,
    ETH = 0,
    MIPI = 2,
    USB = 3,
    GSTUSB = 4,
};

class Camera
{
public:
    Camera(const enCamType camType = enCamType::UNKNOWN);
    virtual ~Camera();
    virtual int Init() = 0;
    virtual void GetFrame(cv::Mat &frame0) = 0;
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
    virtual void GetFrame(cv::Mat &frame0);

private:
    cv::VideoCapture m_viCap;
    cv::VideoCapture m_irCap;
};

class CameraMIPI : public Camera
{
public:
    CameraMIPI(std::string dev, int width, int height);
    virtual ~CameraMIPI();
    virtual int Init();
    virtual void GetFrame(cv::Mat &frame0);

private:
    V4L2Camera *v4l2Camera;
    char *devName;
};

class CameraUSB : public Camera
{
public:
    CameraUSB(std::string dev, int width, int height);
    virtual ~CameraUSB();
    virtual int Init();
    virtual void GetFrame(cv::Mat &frame0);

private:
    USBCamera *usbCamera;
    char *devName;
};

class CameraGSTUSB : public Camera
{
public:
    CameraGSTUSB(int width, int height);
    virtual ~CameraGSTUSB();
    virtual int Init();
    virtual void GetFrame(cv::Mat &frame0);
    static int serailCallBack(int id, guide_usb_serial_data_t *pSerialData);
    static int connectStatusCallBack(int id, guide_usb_device_status_e deviceStatus);
    static int frameCallBack(int id, guide_usb_frame_data_t *pVideoData);

private:
    static cv::Mat frame;
    guide_usb_device_info_t *deviceInfo;
    static std::mutex frameMutex;
};

Camera *CreateCamera(std::string dev, int width, int height, std::string camType);

#endif