#include "camera.h"
#include "sdireader.h"
#include "common.h"
#include <yaml-cpp/yaml.h>

Camera* CreateCamera(const std::string cfgPath)
{
    YAML::Node config = YAML::LoadFile(cfgPath);
    std::string camType = config["inputVideoType"].as<std::string>();;

    Camera *camPtr = nullptr;

    if(camType == "sdi")
    {
        camPtr = new CameraSDI(cfgPath);
    }
    else if(camType == "eth")
    {
        camPtr = new CameraEth(cfgPath);
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


//*******************************CameraSDI Start*************************


CameraSDI::CameraSDI(const std::string cfgPath):Camera(cfgPath, enCamType::SDI)
{

}

int CameraSDI::Init()
{
    Sdireader_Init("/etc/jetsoncfg/NXConfig.ini");

    return RET_OK;
}

void CameraSDI::GetFrame(cv::Mat &frame0, cv::Mat &frame1)
{
    Sdireader_GetFrame(frame0, frame1);
}

CameraSDI::~CameraSDI()
{

}


//*******************************CameraSDI end*************************



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
