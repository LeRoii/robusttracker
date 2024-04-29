#include "camera.h"
#include "common.h"
#include <yaml-cpp/yaml.h>

Camera *CreateCamera(std::string dev, int width, int height, std::string camType)
{

    Camera *camPtr = nullptr;

    // if(camType == "eth")
    // {
    //     camPtr = new CameraEth(cfgPath);
    // }
    // else
    if (camType == "mipi")
    {
        camPtr = new CameraMIPI(dev, width, height);
    }
    else if (camType == "usb")
    {
        camPtr = new CameraUSB(dev, width, height);
    }
    else if (camType == "GSTusb")
    {
        camPtr = new CameraGSTUSB(width, height);
    }

    return camPtr;
}

//*******************************Camera Start*************************

Camera::Camera(const enCamType camType) : m_type(camType)
{
}

Camera::~Camera()
{
}

enCamType Camera::GetType()
{
    return m_type;
}

//*******************************Camera end*************************

//*******************************CameraEth Start*************************

CameraEth::CameraEth(const std::string cfgPath) : Camera(enCamType::ETH)
{
    m_cfg = cfgPath;
}

CameraEth::~CameraEth()
{
}

int CameraEth::Init()
{
    YAML::Node config = YAML::LoadFile(m_cfg);
    std::string irStreamAdd = config["irStreamAdd"].as<std::string>();
    std::string visStreamAdd = config["visStreamAdd"].as<std::string>();

    // m_viCap.open(visStreamAdd);
    // m_irCap.open(irStreamAdd);

    m_viCap.open(visStreamAdd, cv::CAP_GSTREAMER);
    m_irCap.open(irStreamAdd, cv::CAP_GSTREAMER);

    if (!m_viCap.isOpened() || !m_irCap.isOpened())
    {
        printf("eth camera open failed\n");
        return RET_ERR;
    }

    return RET_OK;
}

void CameraEth::GetFrame(cv::Mat &frame0)
{
    m_viCap >> frame0;
}

//*******************************CameraEth end*************************

//*******************************CameraMIPI Start*************************
CameraMIPI::CameraMIPI(std::string dev, int width, int height) : Camera(enCamType::MIPI)
{
    devName = new char[dev.length() + 1];
    std::strcpy(devName, dev.c_str());
    v4l2Camera = new V4L2Camera(devName, width, height, 30);
}

CameraMIPI::~CameraMIPI()
{
    delete v4l2Camera;
    delete[] devName;
}

int CameraMIPI::Init()
{
    if (!v4l2Camera->openDevice())
    {
        return RET_ERR;
    }
    if (!v4l2Camera->queryCapability())
    {
        return RET_ERR;
    }

    if (!v4l2Camera->initDevice())
    {
        return RET_ERR;
    }
    if (!v4l2Camera->mmap_v4l2_buffer())
    {
        return RET_ERR;
    }
    if (!v4l2Camera->startCapture())
    {
        return RET_ERR;
    }
    return RET_OK;
}

void CameraMIPI::GetFrame(cv::Mat &frame0)
{
    frame0 = v4l2Camera->writeVideoFrame();
}

//*******************************CameraMIPI end*************************

//*******************************CameraUSB Start*************************
CameraUSB::CameraUSB(std::string dev, int width, int height) : Camera(enCamType::USB)
{

    devName = new char[dev.length() + 1];
    std::strcpy(devName, dev.c_str());
    usbCamera = new USBCamera(devName, width, height, 30);
}

CameraUSB::~CameraUSB()
{
    delete usbCamera;
    delete[] devName;
}

int CameraUSB::Init()
{
    if (!usbCamera->openDevice())
    {
        return RET_ERR;
    }
    if (!usbCamera->queryCapability())
    {
        return RET_ERR;
    }

    if (!usbCamera->initDevice())
    {
        return RET_ERR;
    }
    if (!usbCamera->mmap_v4l2_buffer())
    {
        return RET_ERR;
    }
    if (!usbCamera->startCapture())
    {
        return RET_ERR;
    }
    return RET_OK;
}

void CameraUSB::GetFrame(cv::Mat &frame0)
{
    frame0 = usbCamera->writeVideoFrame();
}

//*******************************CameraUSB end*************************

//*******************************CameraGSTUSB Start*************************

cv::Mat CameraGSTUSB::frame;
std::mutex CameraGSTUSB::frameMutex;

CameraGSTUSB::CameraGSTUSB(int width, int height) : Camera(enCamType::GSTUSB)
{
    deviceInfo = (guide_usb_device_info_t *)malloc(sizeof(guide_usb_device_info_t));
    deviceInfo->width = width;
    deviceInfo->height = height;
    // deviceInfo->video_mode = YUV_PARAM;
    deviceInfo->video_mode = Y16_PARAM_YUV;
}

CameraGSTUSB::~CameraGSTUSB()
{
    int ret = 0;
    ret = guide_usb_closestream(1);
    printf("close 1 return:%d\n", ret);

    //    ret = guide_usb_closestream(2);
    //    printf("close 2 return:%d\n",ret);

    ret = guide_usb_closecommandcontrol(1);
    printf("closecommandcontrol 1 return:%d\n", ret);

    //    ret = guide_usb_closecommandcontrol(2);
    //    printf("closecommandcontrol 2 return:%d\n",ret);

    ret = guide_usb_exit(1);
    printf("exit 1 return:%d\n", ret);

    //    ret = guide_usb_exit(2);
    //    printf("exit 2 return:%d\n",ret);

    printf("guide_usb_destory_instance return:%d\n", ret);
    free(deviceInfo);
}

int CameraGSTUSB::Init()
{
    int ret = guide_usb_get_devcount();
    printf("devices counts:%d \n", ret);
    ret = guide_usb_initial(1);
    if (ret < 0)
    {
        printf("Initial device 1 fail:%d \n", ret);
        return ret;
    }
    else
    {
        // Endpoint communication is enabled on device 1
        ret = guide_usb_opencommandcontrol(1, (OnSerialDataReceivedCB)serailCallBack);
        printf("Initial device 1 success:%d\n", ret);
    }

    // Device 1 Starts the video streaming thread
    ret = guide_usb_openstream(1, deviceInfo, (OnFrameDataReceivedCB)frameCallBack, (OnDeviceConnectStatusCB)connectStatusCallBack);
    if (ret < 0)
    {
        printf("Open 1 fail:%d\n", ret);
        return ret;
    }
    else
    {
        printf("Open 1 return:%d\n", ret);
    }
}

void CameraGSTUSB::GetFrame(cv::Mat &frame0)
{
    std::unique_lock<std::mutex> lock(frameMutex);
    if (!frame.empty())
        frame0 = frame;
    lock.unlock();
}

int CameraGSTUSB::serailCallBack(int id, guide_usb_serial_data_t *pSerialData)
{

    switch (id)
    {
    case 1:
        printf("ID:%d---->data length:%d \n", id, pSerialData->serial_recv_data_length);
        break;
    case 2:
        printf("ID:%d---->data length:%d \n", id, pSerialData->serial_recv_data_length);

        break;
    case 3:
        // printf("ID:%d---->data length:%d \n",id,pSerialData->serial_recv_data_length);
        break;
    }
}

int CameraGSTUSB::connectStatusCallBack(int id, guide_usb_device_status_e deviceStatus)
{
    switch (id)
    {
    case 1:
        switch (deviceStatus)
        {
        case DEVICE_CONNECT_OK:
            printf("ID:%d VideoStream Capture start...\n", id);
            break;
        case DEVICE_DISCONNECT_OK:
            printf("ID:%d VideoStream Capture end...\n", id);
            break;
        }
        break;
    case 2:
        switch (deviceStatus)
        {
        case DEVICE_CONNECT_OK:
            printf("ID:%d VideoStream Capture start...\n", id);
            break;
        case DEVICE_DISCONNECT_OK:
            printf("ID:%d VideoStream Capture end...\n", id);
            break;
        }
        break;
    }
}

int CameraGSTUSB::frameCallBack(int id, guide_usb_frame_data_t *pVideoData)
{

    // 将YUV数据转换为cv::Mat图像
    cv::Mat yuvFrame(pVideoData->frame_height, pVideoData->frame_width, CV_8UC2, pVideoData->frame_yuv_data);

    // 转换为BGR格式
    cv::Mat bgrFrame;
    cv::cvtColor(yuvFrame, bgrFrame, cv::COLOR_YUV2BGR_UYVY);

    // YUV数据 [pVideoData->frame_yuv_data:机芯送出来的YUV数据] yuv422 uyvy
    // 加锁保护临界区
    std::unique_lock<std::mutex> lock(frameMutex);
    // 存储到全局变量中
    frame = bgrFrame.clone();
    lock.unlock();
}

//*******************************CameraGSTUSB end***************************