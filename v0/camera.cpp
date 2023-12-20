#include "camera.h"
#include "common.h"
#include <yaml-cpp/yaml.h>

Camera* CreateCamera(const std::string cfgPath,std::string camType)
{
    // YAML::Node config = YAML::LoadFile(cfgPath);

    Camera *camPtr = nullptr;

    if(camType == "eth")
    {
        camPtr = new CameraEth(cfgPath);
    }
    else if(camType == "mipi")
    {
        camPtr = new CameraMIPI(cfgPath);
    }
    else if(camType == "usb")
    {
        camPtr = new CameraUSB(cfgPath);
    }

    return camPtr;
    
}

//*******************************Camera Start*************************

Camera::Camera(const std::string cfgPath, const enCamType camType):m_cfg(cfgPath), m_type(camType)
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

CameraEth::CameraEth(const std::string cfgPath):Camera(cfgPath, enCamType::ETH)
{

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

    if(!m_viCap.isOpened() || !m_irCap.isOpened())
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
CameraMIPI::CameraMIPI(const std::string cfgPath):Camera(cfgPath, enCamType::MIPI)
{
    v4l2Camera = new V4L2Camera("/dev/video11", 1920, 1080, 30);
    
}

CameraMIPI::~CameraMIPI()
{
    delete v4l2Camera;

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


    // YAML::Node config = YAML::LoadFile(m_cfg);
    // std::string mipiVisStreamAdd = config["mipiVisStreamAdd"].as<std::string>();


    // m_viCap.open(mipiVisStreamAdd, cv::CAP_GSTREAMER);
    // // m_irCap.open(irStreamAdd, cv::CAP_GSTREAMER);

    // // if(!m_viCap.isOpened() || !m_irCap.isOpened())
    // if(!m_viCap.isOpened())
    // {
    //     printf("mipi camera open failed\n");
    //     return RET_ERR;
    // }

    // return RET_OK;
}

void CameraMIPI::GetFrame(cv::Mat &frame0)
{
    frame0 = v4l2Camera->writeVideoFrame();
}

//*******************************CameraMIPI end*************************

//*******************************CameraUSB Start*************************
CameraUSB::CameraUSB(const std::string cfgPath):Camera(cfgPath, enCamType::USB)
{
    usbCamera = new USBCamera("/dev/video20", 256, 192, 50);
    
}

CameraUSB::~CameraUSB()
{
    delete usbCamera;

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