#include "camera.h"
#include "common.h"
#include <yaml-cpp/yaml.h>
#include "v4l2_capture.hpp"

Camera* CreateCamera(const std::string cfgPath)
{
    YAML::Node config = YAML::LoadFile(cfgPath);
    std::string camType = config["inputVideoType"].as<std::string>();;

    Camera *camPtr = nullptr;

    if(camType == "eth")
    {
        camPtr = new CameraEth(cfgPath);
    }
    else if(camType == "mipi")
    {
        camPtr = new CameraMIPI(cfgPath);
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

void CameraEth::GetFrame(cv::Mat &frame0, cv::Mat &frame1)
{
    m_irCap >> frame1;
    m_viCap >> frame0;
}

//*******************************CameraEth end*************************


//*******************************CameraMIPI Start*************************
CameraMIPI::CameraMIPI(const std::string cfgPath):Camera(cfgPath, enCamType::MIPI)
{
    capture = new V4L2Capture("/dev/video11", 1920, 1080, 30);
    
}

CameraMIPI::~CameraMIPI()
{
    delete capture;

}

int CameraMIPI::Init()
{
    if (!capture->openDevice())
    {
        return RET_ERR;
    }
    if (!capture->queryCapability())
    {
        return RET_ERR;
    }

    if (!capture->initDevice())
    {
        return RET_ERR;
    }
    if (!capture->mmap_v4l2_buffer())
    {
        return RET_ERR;
    }
    if (!capture->startCapture())
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

void CameraMIPI::GetFrame(cv::Mat &frame0, cv::Mat &frame1)
{
    frame0 = capture->writeVideoFrame();
    // m_irCap >> frame1;
    // m_viCap >> frame0;
    frame1 = frame0;
}

//*******************************CameraMIPI end*************************