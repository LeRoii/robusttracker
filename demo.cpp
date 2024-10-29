#include <stdio.h>  
#include <unistd.h>
#include <signal.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <spdlog/fmt/chrono.h>
#include <deque>
#include <numeric>
#include <chrono>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <iterator>
#include <poll.h>
#include <queue>
#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/freetype.hpp>
#include <sstream>
#include <cmath>
#include <arpa/inet.h>
#include <sys/vfs.h>
#include <yaml-cpp/yaml.h>
#include <CL/cl.h>



#ifdef _WIN32  
//Windows  
extern "C"  
{  
#include "libavformat/avformat.h"  
#include "libavutil/mathematics.h"  
#include "libavutil/time.h"  
};  
#else  
//Linux...  
#ifdef __cplusplus  
extern "C"  
{  
#endif  
#include <libavformat/avformat.h>1107
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
#include <libavutil/time.h>
#ifdef __cplusplus  
};  
#endif  
#endif  


#include "common.h"
#include "serialport.h"
#include "camera.h"
#include "painter.h"
#include "realtracker.h"
#include "spdlog/spdlog.h"
#include "spdlog/stopwatch.h"
#include "MppEncoder.h"
#include "VideoPro_Common.h"
#include "VIdeoPro_TimeOut.h"
#include "VideoPro_MsgTable.h"
#include "VideoPro_Enhancement.h"


using namespace std;  
#define __STDC_CONSTANT_MACROS  
#define DEBUG_SERIAL 1


extern ST_A1_CONFIG stA1Cfg;
extern ST_A2_CONFIG stA2Cfg;
extern ST_C1_CONFIG stC1Cfg;
extern ST_C2_CONFIG stC2Cfg;
extern ST_E1_CONFIG stE1Cfg;
extern ST_E2_CONFIG stE2Cfg;
extern ST_S1_CONFIG stS1Cfg;
extern ST_S2_CONFIG stS2Cfg;
extern ST_U_CONFIG stUCfg;
extern ST_A1C1E1_CONFIG stA1C1E1Cfg;
extern ST_A2C2E2_CONFIG stA2C2E2Cfg;
extern ST_A1C1E1S1_CONFIG stA1C1E1S1Cfg;
extern ST_A2C2E2S2_CONFIG stA2C2E2S2Cfg;
extern ST_T1F1B1D1_CONFIG stT1F1B1D1Cfg;
extern ST_T2F2B2D2_CONFIG stT2F2B2D2Cfg;

ST_SYS_STATUS stSysStatus;
unsigned long long lCurTime;            /*****当前时间****/
// 以下变量需要根据您的数据进行初始化
uint8_t *ph264frame; // 指向视频数据的指针
int h264frame_size; // 视频数据的大小

cv::Mat dispFrame;
std::mutex frameMutex;
std::condition_variable frameCond;
bool finished = false;
std::chrono::duration<double, std::milli> elapsed_milliseconds;

std::string globalstreamType;

int count = 0;
int length = 0;

char dst[1920*1080*4];
//char img[1920*1080*4];
//char* img = new char[1280*720*4];
char* img = new char[1920*1080*4];
char *pdst = dst;    

cv::Mat yuvImg;
int64_t duration;
//AVRational time_base;
int fps = 60; // 假设您已经知道帧率是60

AVStream *out_stream;

int frame_index = 0; // 初始化帧索引
int64_t now_time;

int64_t start_time;

int64_t frame_duration;

int64_t pts_time;

whale::vision::MppEncoder mppenc;

AVFormatContext *ifmt_ctx = NULL, *ofmt_ctx = NULL;
AVFormatContext *record_pOutFormatCtx=NULL;  //录像文件
AVPacket pkt;
int ret, videoindex = -1;
//const char *out_filename = "rtsp://192.168.3.5:8554/test1"; // 输出URL
//const char *out_filename = "rtsp://192.168.137.253:8553/stream"; // 输出URL
const char *out_filename = "rtsp://192.168.4.110:8553/stream"; // 输出URL
/**编码分辨率**/
int EncdoerWidth,EncdoerHeight;
/**检测ID**/
int   Detect_Car;
int   Detect_Person;
FILE* fp;

// viewlink通信串口，接收sony相机通信串口，发送sony相机通信串口
Serial serialViewLink, serialRevSony, serialSendSony, serialTCP;
Serial serial137Link; //137通信串口
Serial serialIRLink; //137通信串口

// SerialPort serialTCP;
// 线程同步条件变量
std::condition_variable saveVideoconVar, rtspconVar, OSDconVar, frameVar;
// 线程同步互斥变量
std::mutex m_mtx, rtspMtx, OSDMtx, detectAndTrackMtx, frameMtx, trackMtx;
// 是否存储视频变量
bool isRecording = false;
// 是否存储图片变量
bool isNeedTakePhoto = false;

// 是否对比度增强
bool isNeedContrastEnhance = false;

// 是否线性变换增强
bool isNeedLinearTransformation = false;

// 是否图像增强
bool isNeedImageEnhance = false;
// 是否图像超分辨率
bool isNeedSuperResolution = false;
std::mutex mtx; // 定义一个互斥锁
//超分辨率系数
float  SuperResolutionindex = 1.0;

// 用于推流以及保存视频的全局变量视频帧
cv::Mat saveFrame, rtspFrame, OSDFrame;

// 保存视频的writer
cv::VideoWriter *writer = nullptr;

// 用于推流的writer
cv::VideoWriter rtspWriterr;

// 用于控制伺服在目标中心小范围的时候锁定的变量
bool inObjCenterLock = false;

// TCP客户端连接句柄
int client_sockfd_tcp = -1;

// 存储UDP客户端地址信息
struct sockaddr_in remote_addr;
// UDP客户端连接句柄
int server_sockfd = -1;

realtracker *rtracker;

// 变焦调整波门大小
int zoomGrade = 1;
bool zoomFlag = false;

// UDP或TCP传输标识
bool TCPtransform = true;

bbox_t g_detRet[OBJ_NUMB_MAX_SIZE];
int g_boxes_count = 0;

std::vector<cv::Mat> mat_queue;
std::mutex mat_mutex;
std::condition_variable mat_cond;

VideoPro_Common  VideoProCommon;
cv::Ptr<cv::freetype::FreeType2> ft2;

void serialViewLinkFunc();
/****137项目串口通信协议****/
void serial137Func();
void serialSonyFunc();
//void SaveRecordVideoFunc();
int SaveRecordVideoFunc();

std::atomic<bool> interrupted(false);

std::vector<char> ReadFile(const std::string filename)
{
	std::vector<char> buffer;
    std::ifstream is(filename, std::ios::binary | std::ios::ate);
	if (!is.is_open())
		return buffer; 
    is.unsetf(std::ios::skipws);

    std::streampos size;
    is.seekg(0, std::ios::end);
    size = is.tellg();
    is.seekg(0, std::ios::beg);

    
    buffer.reserve(size);

    buffer.insert(buffer.begin(), std::istream_iterator<char>(is), std::istream_iterator<char>());


    return buffer;
}


// 从内存读取数据的回调函数
int read_packet(void *opaque, uint8_t *buf, int buf_size) {
    if (h264frame_size < buf_size) buf_size = h264frame_size;
    if (buf_size == 0) return AVERROR_EOF; // 检查是否有数据读取
    memcpy(buf, ph264frame, buf_size); // 复制内存内容
    ph264frame += buf_size; // 移动指针
    h264frame_size -= buf_size; // 减少剩余大小
    return buf_size; // 返回读取的字节数
}

void signalHandler(int signum)
{
    // 处理中断信号
    std::cout << "Signal " << signum << " caught, setting the interrupted flag to true." << std::endl;
    interrupted.store(true);
}



static void cvtIrImg(cv::Mat &img, EN_IRIMG_MODE mode)
{

    if (mode == EN_IRIMG_MODE::BLACKHOT)
    {
        cv::cvtColor(img, img, cv::COLOR_RGB2GRAY);
        img = 255 - img;
        cv::cvtColor(img, img, cv::COLOR_GRAY2RGB);
    }
    else if (mode == EN_IRIMG_MODE::PSEUDOCOLOR)
    {
        cv::applyColorMap(img, img, cv::COLORMAP_HOT);
    }
}

static std::string CreateDirAndReturnCurrTimeStr(std::string folderName)
{
    std::string cmd = "mkdir " + folderName;
    int ret = system(cmd.c_str());
    printf("create %s dir result is %s\n", folderName.c_str(), (ret == 0) ? "success" : "failed");
    std::time_t curr = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss << std::put_time(std::localtime(&curr), "%Y-%m-%d-%H-%M-%S");
    std::string currTime(ss.str());
    return currTime;
}

// 跟踪脱靶量向上位机反馈
static void TrackerMissDistanceResultFeedbackToDown(uint8_t *buf)
{
    uint8_t sendBuf[15] = {0};
    int sendBufLen = 15;
    sendBuf[0] = 0x55;
    sendBuf[1] = 0xAA;
    sendBuf[2] = 0xDC;
    sendBuf[3] = 0X0C;
    sendBuf[4] = 0x66;

    stSysStatus.trackMissDistance[0] = (buf[0] << 8) ^ buf[1];
    stSysStatus.trackMissDistance[1] = (buf[2] << 8) ^ buf[3];

    memcpy(sendBuf + 5, buf, 9);

    // sendBuf[14] = viewlink_protocal_checksum(sendBuf);
    // serialViewLink.serial_send(sendBuf, sendBufLen);
}

// 用于打印字节流为十六进制格式
void printHex(uint8_t *buffer, size_t length)
{
    for (size_t i = 0; i < length; i++)
    {
        printf("%02x ", buffer[i]);
    }
    printf("\n");
}

// // TCP服务端线程程序，收到TCP消息解析后转发至小板串口
// void TCP2serialFunc()
// {
//     int server_sockfd_tcp = -1;
//     sockaddr_in my_addr;
//     sockaddr_in remote_addr;
//     socklen_t sin_size;
//     memset(&my_addr, 0, sizeof(my_addr));

//     my_addr.sin_family = AF_INET;
//     my_addr.sin_addr.s_addr = INADDR_ANY;
//     my_addr.sin_port = htons(2000);

//     server_sockfd_tcp = socket(PF_INET, SOCK_STREAM, 0);
//     if (server_sockfd_tcp < 0)
//     {
//         std::cerr << "socket error" << std::endl;
//         return;
//     }

//     // 设置SO_REUSEADDR选项
//     int yes = 1;
//     if (setsockopt(server_sockfd_tcp, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) == -1)
//     {
//         std::cerr << "setsockopt SO_REUSEADDR error" << std::endl;
//         close(server_sockfd_tcp);
//         return;
//     }

//     if (bind(server_sockfd_tcp, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) < 0)
//     {
//         std::cerr << "TCP bind error" << std::endl;
//         close(server_sockfd_tcp);
//         return;
//     }

//     if (listen(server_sockfd_tcp, 5) < 0)
//     {
//         std::cerr << "listen error" << std::endl;
//         close(server_sockfd_tcp);
//         return;
//     }

//     std::cout << "Server is listening on port 2000..." << std::endl;

//     while (!interrupted.load())
//     {
//         sin_size = sizeof(struct sockaddr_in);
//         client_sockfd_tcp = accept(server_sockfd_tcp, (struct sockaddr *)&remote_addr, &sin_size);
//         if (client_sockfd_tcp < 0)
//         {
//             std::cerr << "accept error" << std::endl;
//             if (interrupted.load())
//             {
//                 break; // 退出循环前检查是否应该中断
//             }
//             continue; // 继续下一次循环以接受新连接
//         }

//         std::cout << "Accepted client: " << inet_ntoa(remote_addr.sin_addr) << std::endl;

//         while (!interrupted.load())
//         {
//             uint8_t recv_buf[BUFSIZ];
//             memset(recv_buf, 0, BUFSIZ);
//             ssize_t retLen = recv(client_sockfd_tcp, recv_buf, BUFSIZ, 0);
//             if (retLen > 0)
//             {
//                 TCPtransform = true;
//                 // receiveBuffer.insert(receiveBuffer.end(), recv_buf_temp, recv_buf_temp + retLen);
//                 serialTCP.serial_send(recv_buf + 3, retLen - 4);
//                 // printf("==============>");
//                 // printHex(recv_buf, retLen);
//             }
//             else if (retLen == 0)
//             {
//                 std::cout << "Client disconnected." << std::endl;
//                 break; // 客户端断开连接
//             }
//             else
//             {
//                 std::cerr << "recv error" << std::endl;
//                 if (interrupted.load())
//                 {
//                     break; // 退出循环前检查是否应该中断
//                 }
//                 continue; // 继续下一次循环以尝试再次接收数据
//             }
//         }

//         if (client_sockfd_tcp != -1)
//         {
//             close(client_sockfd_tcp); // 关闭客户端套接字
//             client_sockfd_tcp = -1;   // 将客户端套接字设置为无效
//         }
//         std::cout << "Waiting for new connection..." << std::endl;
//     }

//     if (server_sockfd_tcp != -1)
//     {
//         close(server_sockfd_tcp); // 关闭服务器套接字
//     }
// }
int camFocusPreviousValue = 0;
uint8_t rgb_focus_change[9] = {0x81, 0x01, 0x04, 0x47, 0x00, 0x00, 0x00, 0x00,0xFF};
uint8_t camFocus[30][4] = {
        {0x00, 0x00, 0x00, 0x00},
        {0x01, 0x06, 0x0A, 0x01},
        {0x02, 0x00, 0x06, 0x03},
        {0x02, 0x06, 0x02, 0x08},
        {0x02, 0x0A, 0x01, 0x0D},
        {0x02, 0x0D, 0x01, 0x03},
        {0x02, 0x0F, 0x06, 0x0D},
        {0x03, 0x01, 0x06, 0x01},
        {0x03, 0x03, 0x00, 0x0D},
        {0x03, 0x04, 0x08, 0x06},
        {0x03, 0x05, 0x0D, 0x07},
        {0x03, 0x07, 0x00, 0x09},
        {0x03, 0x08, 0x02, 0x00},
        {0x03, 0x09, 0x02, 0x00},
        {0x03, 0x0A, 0x00, 0x0A},
        {0x03, 0x0A, 0x0D, 0x0D},
        {0x03, 0x0B, 0x09, 0x0C},
        {0x03, 0x0C, 0x04, 0x06},
        {0x03, 0x0C, 0x0D, 0x0C},
        {0x03, 0x0D, 0x06, 0x00},
        {0x03, 0x0D, 0x0D, 0x04},
        {0x03, 0x0E, 0x03, 0x09},
        {0x03, 0x0E, 0x09, 0x00},
        {0x03, 0x0E, 0x0D, 0x0C},
        {0x03, 0x0F, 0x01, 0x0E},
        {0x03, 0x0F, 0x05, 0x07},
        {0x03, 0x0F, 0x08, 0x0A},
        {0x03, 0x0F, 0x08, 0x06},
        {0x03, 0x0F, 0x0D, 0x0C},
        {0x04, 0x00, 0x00, 0x00}};

// TCP服务端线程程序，收到TCP消息解析后转发至小板串口
void TCP2serialFunc()
{
    int server_sockfd_tcp = -1;
    sockaddr_in my_addr;
    sockaddr_in remote_addr;
    socklen_t sin_size;
    memset(&my_addr, 0, sizeof(my_addr));

    my_addr.sin_family = AF_INET;
    my_addr.sin_addr.s_addr = INADDR_ANY;
    my_addr.sin_port = htons(2000);

    server_sockfd_tcp = socket(PF_INET, SOCK_STREAM, 0);
    if (server_sockfd_tcp < 0)
    {
        std::cerr << "socket error" << std::endl;
        return;
    }

    // 设置SO_REUSEADDR选项
    int yes = 1;
    if (setsockopt(server_sockfd_tcp, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) == -1)
    {
        std::cerr << "setsockopt SO_REUSEADDR error" << std::endl;
        close(server_sockfd_tcp);
        return;
    }

    if (bind(server_sockfd_tcp, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) < 0)
    {
        std::cerr << "TCP bind error" << std::endl;
        close(server_sockfd_tcp);
        return;
    }

    if (listen(server_sockfd_tcp, 5) < 0)
    {
        std::cerr << "listen error" << std::endl;
        close(server_sockfd_tcp);
        return;
    }

    std::cout << "Server is listening on port 2000..." << std::endl;

    std::vector<uint8_t> receiveBuffer;
    const std::vector<uint8_t> frameStart = {0x55, 0xAA, 0xDC};
    // const std::vector<uint8_t> frameStart_XJ = {0xAA, 0x55, 0xDC};
    uint8_t buffRcvData[1024] = {0};
    
    while (!interrupted.load())
    {
        sin_size = sizeof(struct sockaddr_in);
        client_sockfd_tcp = accept(server_sockfd_tcp, (struct sockaddr *)&remote_addr, &sin_size);
        if (client_sockfd_tcp < 0)
        {
            std::cerr << "accept error" << std::endl;
            if (interrupted.load())
            {
                break; // 退出循环前检查是否应该中断
            }
            continue; // 继续下一次循环以接受新连接
        }

        std::cout << "Accepted client: " << inet_ntoa(remote_addr.sin_addr) << std::endl;

        while (!interrupted.load())
        {
            uint8_t recv_buf[BUFSIZ];
            memset(recv_buf, 0, BUFSIZ);
            ssize_t retLen = recv(client_sockfd_tcp, recv_buf, BUFSIZ, 0);
            if (retLen > 0)
            {
            std::cout << "=======>serialUp received " << std::dec << retLen << "bytes" << std::endl;
            receiveBuffer.insert(receiveBuffer.end(), recv_buf, recv_buf + retLen);
            // 处理粘包的情况
            {
                // 循环直到缓冲区数据不够一帧的最小长度
                while (receiveBuffer.size() > frameStart.size())
                {
                    // 查找帧起始标志
                    auto frameStartIt = std::search(receiveBuffer.begin(), receiveBuffer.end(), frameStart.begin(), frameStart.end());
                    // std::cout << "frameStartIt != receiveBuffer.end(): "<<(frameStartIt != receiveBuffer.end())<<std::endl;
                    // for (int i = 0; i < receiveBuffer.size(); i++)
                    // {
                    //     printf("[%02X]", receiveBuffer[i]);
                    // }
                    // printf("\n");
                    if (frameStartIt != receiveBuffer.end())
                    {
                        // 检查是否有足够的数据读取长度字节
                        auto headerEndIt = frameStartIt + frameStart.size();
                        std::cout <<"distance="<<std::distance(headerEndIt, receiveBuffer.end()) <<std::endl;
                        if (std::distance(headerEndIt, receiveBuffer.end()) >= 1)
                        {
                            // 读取长度字节，假设长度字节紧随帧起始后
                            size_t frameLength = *(headerEndIt) & 0x3F; // 取字节的低6位作为长度
                            std::cout<< "长度："  <<frameLength<<std::endl;
                            // 检查是否有足够的数据包含整个帧
                            if (std::distance(headerEndIt, receiveBuffer.end()) >= frameLength)
                            {
                                // 提取帧，包括帧头和数据
                                std::vector<uint8_t> frame(frameStartIt, headerEndIt + frameLength);
                                // 处理帧

                                EN_DATA_FRAME_TYPE frameType = GetFrameType(frame, frameLength + 3);
                                std::cout <<"frameType="<< frameType <<std::endl;
                                // printf("frame type:%d\n", frameType);
                                if (frameType == HeartBeat15)
                                {
                                    printf("heart beat 15 from up serial\n\n");
                                    receiveBuffer.erase(receiveBuffer.begin(), headerEndIt + frameLength);
                                    continue;
                                }
                                else if (frameType == CtrlSdCmd)
                                {
                                    // printf("CtrlSdCmd from up serial\n\n");
                                    receiveBuffer.erase(receiveBuffer.begin(), headerEndIt + frameLength);
                                    continue;
                                }
                                else if (frameType == IPInq)
                                {
                                    // printf("IPInq from up serial\n\n");
                                    receiveBuffer.erase(receiveBuffer.begin(), headerEndIt + frameLength);
                                    continue;
                                }
                                else if (frameType == HandShake)
                                {
                                    printf("handshake from up serial\n\n");
                                    receiveBuffer.erase(receiveBuffer.begin(), headerEndIt + frameLength);
                                    continue;
                                }
                                else if (frameType == FrameS2)
                                {
                                    // printf("TGCC Ctrl S2 from up serial\n\n");
                                    receiveBuffer.erase(receiveBuffer.begin(), headerEndIt + frameLength);
                                    continue;
                                }
                                // else if (frameType == FrameE2)
                                // {
                                //     printf("TGCC Ctrl E2 from up serial\n\n");
                                //     receiveBuffer.erase(receiveBuffer.begin(), headerEndIt + frameLength);
                                //     continue;
                                // }
                                else if (frameType == Status42)
                                {
                                    // printf("TGCC Ctrl 42 from up serial\n\n");
                                    receiveBuffer.erase(receiveBuffer.begin(), headerEndIt + frameLength);
                                    continue;
                                }
                                else if (frameType == HeartBeat14)
                                {
                                    printf("heart beat 14 from up serial\n\n");
                                    receiveBuffer.erase(receiveBuffer.begin(), headerEndIt + frameLength);
                                    continue;
                                }
                                for (int i = 0; i < frame.size(); i++)
                                {
                                    printf("[%02X]", frame[i]);
                                }
                               printf("\n");
                                uint8_t *usefulFrame = frame.data();
                               if (frameType == FocusC1) {
                                    rgb_focus_change[4] = 0;
                                    rgb_focus_change[5] = 0;
                                    rgb_focus_change[6] = 0;
                                    rgb_focus_change[7] = 0;

                                    if (usefulFrame[6] == 0x80 && camFocusPreviousValue < 30) {
                                        camFocusPreviousValue++;
                                    } else if (usefulFrame[6] == 0xC0 && camFocusPreviousValue >= 1) {
                                        camFocusPreviousValue--;
                                    }
                                    printf("focus camFocusPreviousValue=%d\n", camFocusPreviousValue);
                                    rgb_focus_change[4] = camFocus[camFocusPreviousValue][0];
                                    rgb_focus_change[5] = camFocus[camFocusPreviousValue][1];
                                    rgb_focus_change[6] = camFocus[camFocusPreviousValue][2];
                                    rgb_focus_change[7] = camFocus[camFocusPreviousValue][3];
                                    printf("rgb_focus_change :[%02X][%02X][%02X][%02X]\n", rgb_focus_change[4], rgb_focus_change[5], rgb_focus_change[6], rgb_focus_change[7]);
                                    serialSendSony.serial_send(rgb_focus_change, 9);
                                }
                                VL_ParseSerialData(usefulFrame);

                                // 移除处理过的数据
                                receiveBuffer.erase(receiveBuffer.begin(), headerEndIt + frameLength);
                            }
                            else
                            {
                                // 不完整的帧，退出循环等待更多数据
                                break;
                            }
                        }
                        else
                        {
                            // 不完整的头，退出循环等待更多数据
                            break;
                        }
                    }
                    else
                    {
                        // 未找到帧起始标志，清空缓冲区
                        receiveBuffer.clear();
                        break;
                    }
                }
            }

        }
            else if (retLen == 0)
            {
                std::cout << "Client disconnected." << std::endl;
                break; // 客户端断开连接
            }
            else
            {
                std::cerr << "recv error" << std::endl;
                if (interrupted.load())
                {
                    break; // 退出循环前检查是否应该中断
                }
                continue; // 继续下一次循环以尝试再次接收数据
            }
        }

        if (client_sockfd_tcp != -1)
        {
            close(client_sockfd_tcp); // 关闭客户端套接字
            client_sockfd_tcp = -1;   // 将客户端套接字设置为无效
        }
        std::cout << "Waiting for new connection..." << std::endl;
    }

    if (server_sockfd_tcp != -1)
    {
        close(server_sockfd_tcp); // 关闭服务器套接字
    }
}


// 从小板接收的串口数据封装成TCP消息，向上位机发送
void serial2TCPFunc()
{

    uint8_t buffRcvData[1024] = {0};
    uint8_t buffRcvData_[1024] = {0};
    int retLen = 0;
    std::vector<uint8_t> receiveBuffer;
    const std::vector<uint8_t> frameStart = {0x55, 0xAA, 0xDC};
    buffRcvData[0] = 0xEB;
    buffRcvData[1] = 0x90;

    while (!interrupted.load())
    {
        if (TCPtransform)
        {
            retLen = serialTCP.serial_receive(buffRcvData + 3);

            if (retLen > 0)
            {
                // printf("<===================");
                // printHex(buffRcvData + 3, retLen);
                // std::cout << std::endl;
                buffRcvData[2] = retLen & 0xFF;
                buffRcvData[retLen + 3] = viewlink_protocal_tcp_checksum(buffRcvData + 2);
                int len = send(client_sockfd_tcp, buffRcvData, 4 + retLen, 0);
            }
        }
    }
}
uint16_t UDPSendPort;
// UDP接收上位机消息
void UDP2serialFunc()
{

    struct sockaddr_in my_addr;

    socklen_t sin_size;
    uint8_t recv_buf[BUFSIZ];

    // 初始化地址信息
    memset(&my_addr, 0, sizeof(my_addr));
    my_addr.sin_family = AF_INET;
    my_addr.sin_addr.s_addr = INADDR_ANY;
    my_addr.sin_port = htons(UDPSendPort);

    // 创建UDP socket
    if ((server_sockfd = socket(PF_INET, SOCK_DGRAM, 0)) < 0)
    {
        std::cout << "socket error" << std::endl;
        return;
    }

    // 绑定socket到指定的端口和任意地址
    if (bind(server_sockfd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) < 0)
    {
        std::cout << " UDP bind error" << std::endl;
        return;
    }

    std::cout << "Server is listening on port 2000 for UDP traffic..." << std::endl;

    while (!interrupted.load())
    {
        sin_size = sizeof(struct sockaddr_in);
        // 接受来自客户端的数据
        ssize_t len = recvfrom(server_sockfd, recv_buf, BUFSIZ, 0, (struct sockaddr *)&remote_addr, &sin_size);
        if (len > 0)
        {
            TCPtransform = false;
            // std::cout << "Received packet from " << inet_ntoa(remote_addr.sin_addr) << std::endl;
            serialTCP.serial_send(recv_buf + 3, len - 4);
        }
    }

    close(server_sockfd); // 关闭socket
}

// 从小板接收的串口数据封装成UDP消息，向上位机发送
void serial2UDPFunc()
{
    uint8_t buffRcvData[1024] = {0};
    int retLen = 0;
    memset(buffRcvData, 0, sizeof(buffRcvData));
    buffRcvData[0] = 0xeb;
    buffRcvData[1] = 0x90;
    std::vector<uint8_t> receiveBuffer;
    const std::vector<uint8_t> frameStart = {0x55, 0xAA, 0xDC};

    while (!interrupted.load())
    {
        if (TCPtransform == false)
        {
            retLen = serialTCP.serial_receive(buffRcvData + 3);

            if (retLen > 0)
            {
                buffRcvData[2] = retLen & 0xFF; // 假设长度可以用一个字节表示
                buffRcvData[retLen + 3] = viewlink_protocal_tcp_checksum(buffRcvData + 3);

                // 发送数据到客户端
                socklen_t addrlen = sizeof(remote_addr); // 客户端地址长度
                sendto(server_sockfd, buffRcvData, retLen + 4, 0, (struct sockaddr *)&remote_addr, addrlen);
            }
        }
    }
}

// 绘制OSD线程以及推流
void osdAndSendRTSPStreamFunc()
{
    // 循环等待主线程的通知
    cv::Mat frame, dispFrame;
    while (!interrupted.load())
    {

        // 在界面上绘制OSD
        {
            std::unique_lock<std::mutex> lock(OSDMtx);
            OSDconVar.wait(lock);
            frame = OSDFrame.clone();
        }
        // 在界面上绘制OSD
        // stSysStatus.osdSet1Ctrl.enOSDShow = true;
        if (!stSysStatus.osdCtrl.osdSwitch)
        {
            if (!stSysStatus.osdCtrl.attitudeAngleSwitch)
            {
                // 绘制吊舱当前方位角度滚轴
                PaintRollAngleAxis(frame, stSysStatus.rollAngle);

                // 绘制吊舱当前俯仰角度滚轴
                PaintPitchAngleAxis(frame, stSysStatus.pitchAngle);
            }
            if (!stSysStatus.osdCtrl.crosshairSwitch)
            {
                // 绘制中心十字`
                PaintCrossPattern(frame, stSysStatus.rollAngle, stSysStatus.pitchAngle);
            }
            if (!stSysStatus.osdCtrl.GPSSwitch)
            {
                // 绘制经纬度、海拔高度等坐标参数
                PaintCoordinate(frame);
            }

            // 绘制界面上其他参数
            PaintViewPara(frame);
            if (!stSysStatus.osdCtrl.MissToTargetSwitch)
            {
                // 绘制脱靶量
                PaintTrackerMissDistance(frame);
            }
        }

        cv::resize(frame, dispFrame, cv::Size(1280, 720), cv::INTER_NEAREST);
        if (isRecording)
        {
            std::unique_lock<std::mutex> lock(m_mtx);
            saveFrame = dispFrame.clone();
        }

        if (isNeedTakePhoto)
        {
            std::string currTimeStr = CreateDirAndReturnCurrTimeStr("photos");
            std::string savePicFileName = "photos/" + currTimeStr + ".png";
            cv::imwrite(savePicFileName, dispFrame);
            isNeedTakePhoto = false;
        }

        if (!dispFrame.empty() && rtspWriterr.isOpened())
        {
            rtspWriterr << dispFrame;
        }
    }
}

// 多线程导致的图像延时，三线程会视频延时会增加两帧的延时
std::queue<cv::Mat> frameQueue;
uint8_t trackerStatus[9];
// std::vector<bbox_t> g_detRet;

cv::Rect g_trackRect;

void detectAndTrackFunc()
{
    // 循环等待主线程的通知
    // cv::Mat frontFrame, backFrame;
    cv::Mat frontFrame = cv::Mat::zeros(1920, 1080, CV_8UC3);
    cv::Mat backFrame = cv::Mat::zeros(1920, 1080, CV_8UC3);
    cv::Rect trackRect;

    int center_x, center_y;
    bool detOn;
    // 产生的检测框vector
    bbox_t detRet_[OBJ_NUMB_MAX_SIZE];
    memset(detRet_, 0x00, sizeof(*detRet_));
    int boxes_count = 0;
    while (!interrupted.load())
    {
        detOn = true;
        {

            std::unique_lock<std::mutex> lock(frameMtx);
            frameVar.wait(lock);
            frontFrame = frameQueue.front().clone();
            backFrame = frameQueue.back().clone();
        }

        if (stSysStatus.trackOn)
        {
            if (!stSysStatus.trackerInited)
            {
                stSysStatus.detOn = false;
                spdlog::debug("start tracking, init Rect:");
                stSysStatus.trackerGateSize = stSysStatus.trackerGateSize * (sqrt(sqrt(zoomGrade)));
                zoomGrade = 1;
                rtracker->setGateSize(stSysStatus.trackerGateSize);
                rtracker->reset();
                rtracker->init(stSysStatus.trackAssignPoint, frontFrame, backFrame);
                stSysStatus.trackerInited = true;
            }
            else
            {
                rtracker->update(backFrame, frontFrame, trackerStatus, center_x, center_y, trackRect);
                {
                    std::unique_lock<std::mutex> lock(trackMtx);
                    g_trackRect = trackRect;
                }

                spdlog::debug("tracker status:{}", trackerStatus[4]);

                TrackerMissDistanceResultFeedbackToDown(trackerStatus);
                if (zoomFlag)
                {
                    stSysStatus.trackAssignPoint.x = center_x;
                    stSysStatus.trackAssignPoint.y = center_y;
                    stSysStatus.trackerInited = false;
                    zoomFlag = false;
                }
                if (stSysStatus.detOn)
                {
                    stSysStatus.trackOn = false;
                }
            }
        }
        else
        {

            rtracker->runDetectorOut(backFrame, detRet_, boxes_count);

            std::unique_lock<std::mutex> lock(detectAndTrackMtx);
            g_boxes_count = boxes_count;
            memcpy(g_detRet, detRet_, sizeof(bbox_t) * OBJ_NUMB_MAX_SIZE);
        }
    }
}

// 预定义一个包含12种醒目颜色的列表
const std::vector<cv::Scalar> predefinedColors = {
    cv::Scalar(255, 0, 0),   // 红色
    cv::Scalar(0, 255, 0),   // 绿色
    cv::Scalar(0, 0, 255),   // 蓝色
    cv::Scalar(255, 255, 0), // 黄色
    cv::Scalar(255, 0, 255), // 粉色
    cv::Scalar(0, 255, 255), // 青色
    cv::Scalar(255, 127, 0), // 橙色
    cv::Scalar(127, 0, 255), // 紫色
    cv::Scalar(0, 127, 255), // 天蓝色
    cv::Scalar(127, 255, 0), // 酸橙色
    cv::Scalar(255, 0, 127), // 玫瑰红
    cv::Scalar(0, 255, 127), // 春绿色
    // ... 或许还可以添加更多的颜色，如果类别有增加
};


#define INTPUT_Width              640
#define INTPUT_Height             512
#define OUTPUT_Width              6400
#define OUTPUT_Height             5120

#define MAX_SOURCE_SIZE (0x100000)

// 帧数累加变量
uint64_t nFramesl = 0;

// 折线运算函数
cv::Mat applyPiecewiseLinearTransform(const cv::Mat& img_detail, double lowerThreshold, double upperThreshold,double k1,double k2,double k3) {
    cv::Mat result = img_detail.clone();
    for (int y = 0; y < img_detail.rows; y++) {
        for (int x = 0; x < img_detail.cols; x++) {
            float value = img_detail.at<float>(y, x);
            if (value > -lowerThreshold && value < lowerThreshold) {
                result.at<float>(y, x) = value * k1; // Example: reduce the intensity for lower values
            } 
            else if (value >= lowerThreshold && value <= upperThreshold) {
                result.at<float>(y, x) = lowerThreshold*k1+(value- lowerThreshold)* k2; // Example: enhance the intensity for higher values
            }
            else if (value > upperThreshold) {
                result.at<float>(y, x) = lowerThreshold*k1+(upperThreshold- lowerThreshold)* k2 + (value-upperThreshold) * k3; // Example: enhance the intensity for higher values
            }
             else if (value <= -lowerThreshold && value >= -upperThreshold) {
                result.at<float>(y, x) = -lowerThreshold*k1+(value+lowerThreshold)* k2; // Example: enhance the intensity for higher values
            }
            else if (value < -upperThreshold) {
                result.at<float>(y, x) = -lowerThreshold*k1+(-upperThreshold+ lowerThreshold)* k2 + (value+upperThreshold) * k3; // Example: enhance the intensity for higher values
            }            
        }
    }
    
    return result;
}

// cv::Mat applyPiecewiseLinearTransform(const cv::Mat& img_detail, double lowerThreshold, double upperThreshold, double k1, double k2, double k3) {
//     cv::Mat result = img_detail.clone();
//     for (int y = 0; y < img_detail.rows; y++) {
//         for (int x = 0; x < img_detail.cols; x++) {
//             for (int c = 0; c < 3; c++) {
//                 unsigned char value = img_detail.at<cv::Vec3b>(y, x)[c];
//                 if (value > -lowerThreshold && value < lowerThreshold) {
//                     result.at<cv::Vec3b>(y, x)[c] = static_cast<uchar>(value * k1);
//                 }
//                 else if (value >= lowerThreshold && value <= upperThreshold) {
//                     result.at<cv::Vec3b>(y, x)[c] = static_cast<uchar>(lowerThreshold * k1 + (value - lowerThreshold) * k2);
//                 }
//                 else if (value > upperThreshold) {
//                     result.at<cv::Vec3b>(y, x)[c] = static_cast<uchar>(lowerThreshold * k1 + (upperThreshold - lowerThreshold) * k2 + (value - upperThreshold) * k3);
//                 }
//                 else if (value <= -lowerThreshold && value >= -upperThreshold) {
//                     result.at<cv::Vec3b>(y, x)[c] = static_cast<uchar>(-lowerThreshold * k1 + (value + lowerThreshold) * k2);
//                 }
//                 else if (value < -upperThreshold) {
//                     result.at<cv::Vec3b>(y, x)[c] = static_cast<uchar>(-lowerThreshold * k1 + (-upperThreshold + lowerThreshold) * k2 + (value + upperThreshold) * k3);
//                 }
//             }
//         }
//     }
//     return result;
// }


cv::Mat LinearDarken(const cv::Mat& img_todark, double Threshold,double k)
{
 cv::Mat dst = img_todark.clone();

    for (int y = 0; y < img_todark.rows; y++)
     {
        for (int x = 0; x < img_todark.cols; x++)
        {
            float value = img_todark.at<float>(y, x);
            if (value < Threshold) 
            {
                    dst.at<float>(y, x) = value * k; // Example: reduce the intensity for lower values
            } 
            else 
            {
            dst.at<float>(y, x) = (255.0-Threshold*k)/(255.0-Threshold)*value + (255.0-((255.0-Threshold*k)/(255.0-Threshold)*255.0)); // Example: enhance the intensity for higher values
            }
        }
    }
 
  return dst;
}

int main()
{
    //std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // spdlog调试
    spdlog::set_level(spdlog::level::debug);
    spdlog::stopwatch sw;
    signal(SIGINT, signalHandler);

    //  fps统计变量
    std::deque<double> fpsCalculater;

    // 串口初始化
   // serialViewLink.set_serial(1); // "/dev/ttyTHS1"
    serial137Link.set_serial(1);   //初始化串口节点"/dev/ttyTHS1" 
    serialRevSony.set_serial(2);  // "/dev/ttyS0"
    serialSendSony.set_serial(3); // "/dev/ttyS6"
    serialTCP.set_serial(4);      // "/dev/ttyUSB0"
    serialIRLink.set_serial(5);      // "/dev/ttyACM0"


    uint8_t vidata[9]={0x81,0x01,0x04,0x47,0x02,0x00, 0x04, 0x00, 0xFF};
    serialSendSony.serial_send(vidata, 9);
    // serialTCP.set_serial("/dev/ttyUSB0", B115200, 8, 'N', 0); // "/dev/ttyUSB0"

    // 读取配置文件加载模型路径、设备名称等配置
    std::string cfgpath = "/home/rpdzkj/1/h265encode_test/exe/config.yaml";
    YAML::Node config = YAML::LoadFile(cfgpath);
    std::string visi_dev = config["visi_dev"].as<std::string>();
    std::string ir_dev = config["ir_dev"].as<std::string>();
    std::string streamType = config["streamType"].as<std::string>();
    globalstreamType = streamType;
    std::cout << streamType <<std::endl;
    std::cout << globalstreamType <<std::endl;
    // printf(globalstreamType);
    // printf(streamType);
    UDPSendPort = config["UDPSendPort"].as<uint16_t>();

    std::string  recognitionClass=config["recognitionClass"].as<std::string>();
   
    if (recognitionClass == "Both")
    { 

              std::cout <<  " 检测:Both " <<std::endl;
       Detect_Car =1;
       Detect_Person=1;
    }
    else if(recognitionClass == "Car")
    {
        std::cout <<  " 检测:Car " <<std::endl;
       Detect_Car =1;
       Detect_Person=0;
    }
    else if(recognitionClass == "Person")
    {
     std::cout <<  " 检测:Person " <<std::endl;
       Detect_Car =0;
       Detect_Person=1;
    }
    
    std::string EncdoerType=config["resolution"].as<std::string>();
    if (EncdoerType == "1080P")
    { 
        EncdoerWidth=1920 ;
        EncdoerHeight=1080;
    }
    else
    {
        EncdoerWidth=1280 ;
        EncdoerHeight=720;
    }
    int Encdoerprofile=100;
    std::string videoCompressionQuality=config["videoCompressionQuality"].as<std::string>();
    if (videoCompressionQuality == "low")
    { 
       Encdoerprofile=66;
    }
    else if(videoCompressionQuality == "medium")
    {
        Encdoerprofile=77;
    }
    else
    {
        Encdoerprofile=100;
    }

      ft2 = cv::freetype::createFreeType2();
      ft2->loadFontData("/home/rpdzkj/2/simsun.ttc", 0);

      int EncoderBitrate= config["RTSPEncoderBitrate"].as<uint16_t>();
    
       mppenc.MppEncdoerInit(EncdoerWidth, EncdoerHeight,EncoderBitrate,Encdoerprofile, 30);

        av_register_all();
        avformat_network_init();

        // 创建输入格式上下文
        ifmt_ctx = avformat_alloc_context();
        //unsigned char *avio_ctx_buffer = (unsigned char *)av_malloc(4096);
        unsigned char *avio_ctx_buffer = (unsigned char *)av_malloc(1920*1080);
        //AVIOContext *avio_ctx = avio_alloc_context(avio_ctx_buffer, 4096, 0, NULL, &read_packet, NULL, NULL);
        AVIOContext *avio_ctx = avio_alloc_context(avio_ctx_buffer, (1920*1080), 0, NULL, &read_packet, NULL, NULL);
        ifmt_ctx->pb = avio_ctx;
        
        // 手动设置输入流信息
        AVStream *in_stream = avformat_new_stream(ifmt_ctx, NULL);
        AVCodecParameters *in_codecpar = in_stream->codecpar;
        in_codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
        if (globalstreamType == "h264")
        {
            in_codecpar->codec_id = AV_CODEC_ID_H264;
        }
        else if (globalstreamType == "h265")
        {
           in_codecpar->codec_id = AV_CODEC_ID_H265;
        }
    
        //in_codecpar->codec_id = AV_CODEC_ID_H264;
        //in_codecpar->codec_id = AV_CODEC_ID_H265;

        in_codecpar->format = AV_PIX_FMT_YUV420P;
        //in_codecpar->format = AV_PIX_FMT_NV12;
        //in_codecpar->width = 1920;  // 示例宽度，根据实际情况调整
        //in_codecpar->height = 1080; // 示例高度，根据实际情况调整
        in_codecpar->width = 1280;  // 示例宽度，根据实际情况调整
        in_codecpar->height = 720; // 示例高度，根据实际情况调整
        // 根据需要设置其他参数

        // 设置输出格式上下文
        avformat_alloc_output_context2(&ofmt_ctx, NULL, "rtsp", out_filename);
        if (!ofmt_ctx) {
            printf("Could not create output context\n");
            ret = AVERROR_UNKNOWN;
            //goto end;
        }

        // 创建输出流并复制输入流参数
        //AVStream *out_stream = avformat_new_stream(ofmt_ctx, NULL);
        out_stream = avformat_new_stream(ofmt_ctx, NULL);
        if (!out_stream) {
            printf("Failed allocating output stream\n");
            ret = AVERROR_UNKNOWN;
            //goto end;
        }
        ofmt_ctx->streams[0]->time_base=(AVRational){ 1, 60 };
        ret = avcodec_parameters_copy(out_stream->codecpar, in_codecpar);
        if (ret < 0) {
            printf("Failed to copy codec parameters\n");
            //goto end;
        }
        out_stream->codecpar->codec_tag = 0;
        // 打开输出URL
        if (!(ofmt_ctx->oformat->flags & AVFMT_NOFILE)) {
            ret = avio_open(&ofmt_ctx->pb, out_filename, AVIO_FLAG_WRITE);
            if (ret < 0) {
                printf("Could not open output URL '%s'\n", out_filename);
                //goto end;
            }
        }
        
        if (!ofmt_ctx) {
          fprintf(stderr, "Could not create output context\n");
          printf("Could not create output context\n");
          return -1;  // 处理错误
        }
        
    ret = avformat_write_header(ofmt_ctx, NULL);
    for (unsigned i = 0; i < ofmt_ctx->nb_streams; i++) {
        if (!ofmt_ctx->streams[i]->codecpar) {
            fprintf(stderr, "Stream parameters are not set properly\n");
            printf("Could not create output context\n");
            return -1;
        }
    }
    if (ret < 0) {
        printf("Error occurred when opening output URL\n");
        //goto end;
    }

   // 读取内核源代码文件
    FILE *kernelFile;
    char *kernelSource;
    size_t kernelSize;

    kernelFile = fopen("/home/rpdzkj/2/kernel_.cl", "r");
    if (!kernelFile) {
        std::cout << "无法打开内核文件！" << std::endl;
        return -1;
    }

    kernelSource = (char *)malloc(MAX_SOURCE_SIZE);
    kernelSize = fread(kernelSource, 1, MAX_SOURCE_SIZE, kernelFile);
    fclose(kernelFile);

    // 初始化OpenCL设备和上下文
    cl_platform_id platformId;
    cl_device_id deviceId;
    cl_uint numDevices, numPlatforms;
    cl_int ret;

    ret = clGetPlatformIDs(1, &platformId, &numPlatforms);

    ret = clGetDeviceIDs(platformId, CL_DEVICE_TYPE_GPU, 1, &deviceId, &numDevices);
    cl_context context = clCreateContext(NULL, 1, &deviceId, NULL, NULL, &ret);

    // 创建命令队列
    cl_command_queue commandQueue = clCreateCommandQueue(context, deviceId, 0, &ret);
    // 创建图像对象并加载图像数据
    size_t inputWidth = INTPUT_Width;
    size_t inputHeight = INTPUT_Height;
    size_t outputWidth =OUTPUT_Width;
    size_t outputHeight = OUTPUT_Height;
    size_t image_row_pitch = 0;
    size_t image_slice_pitch = 0;

    cl_image_format format;

    format.image_channel_order = CL_LUMINANCE;
    format.image_channel_data_type = CL_UNORM_INT8;

    cl_image_desc imageDesc;
    imageDesc.image_type = CL_MEM_OBJECT_IMAGE2D;
    imageDesc.image_width = inputWidth;
    imageDesc.image_height = inputHeight;
    imageDesc.image_row_pitch = 0;
    imageDesc.image_slice_pitch = 0;
    imageDesc.num_mip_levels = 0;
    imageDesc.num_samples = 0;
    imageDesc.buffer = NULL; 
    int kernel_size = 3;
    float gauss_sigma = 5.0;
    float gauss_alpha = 2.0; //细节增强系数。用于控制对图像进行细节增强的程度。当这个值较大时，会对图像的细节部分进行更强烈的增强。
    float gausskernel[(2*3+1)*(2*3+1)]={0};

    for (int i = 0; i <= (2*kernel_size+1); i++) 
    { 
        for (int j = 0; j <= (2*kernel_size+1); j++) 
        { 
            float weight = exp(-((i-kernel_size)*(i-kernel_size) + (j-kernel_size)*(j-kernel_size))/ (2.0 * gauss_sigma * gauss_sigma));
            gausskernel[i*(2*kernel_size+1)+j] = weight;
        }
    }
    cl_mem inputBuffer = clCreateImage2D(context, CL_MEM_READ_ONLY | CL_MEM_ALLOC_HOST_PTR, &format, inputWidth, inputHeight, 0, NULL, (cl_int*)&ret);
    cl_mem outputBuffer = clCreateImage2D(context, CL_MEM_WRITE_ONLY, &format, outputWidth, outputHeight, 0, NULL, &ret);
    cl_mem innerBuffer = clCreateImage2D(context, CL_MEM_READ_WRITE, &format, inputWidth, inputHeight, 0, NULL, &ret);
    // 创建内核程序并设置参数
    cl_mem gauss_kernel = clCreateBuffer(context,CL_MEM_READ_ONLY| CL_MEM_COPY_HOST_PTR, sizeof(float)*(2*3+1),&gausskernel,(cl_int*)&ret);
    // 创建内核程序并设置参数
    cl_program program = clCreateProgramWithSource(context, 1, (const char **)&kernelSource, (const size_t *)&kernelSize,(cl_int*) &ret);
    ret = clBuildProgram(program, 1, &deviceId, NULL, NULL, NULL);
    cl_kernel kernel1 = clCreateKernel(program, "image_process", &ret);

    ret = clSetKernelArg(kernel1, 0, sizeof(cl_mem), (void *)&inputBuffer);
    ret = clSetKernelArg(kernel1, 1, sizeof(cl_mem), (void *)&innerBuffer);
    ret = clSetKernelArg(kernel1, 2, sizeof(cl_mem), (void *)&gauss_kernel);
    ret = clSetKernelArg(kernel1, 3, sizeof(int), &kernel_size);
    ret = clSetKernelArg(kernel1, 4, sizeof(float), &gauss_sigma);
    ret = clSetKernelArg(kernel1, 5, sizeof(float), &gauss_alpha);


    /**细节增强 线性变换****/
    float  lowerThreshold = 8;
    float  upperThreshold = 36;
    float  k1 =0;
    float  k2 =1.0;
    float  k3 =0.2;
    ret = clSetKernelArg(kernel1, 6, sizeof(float), &lowerThreshold);
    ret = clSetKernelArg(kernel1, 7, sizeof(float), &upperThreshold);
    ret = clSetKernelArg(kernel1, 8, sizeof(float), &k1);
    ret = clSetKernelArg(kernel1, 9, sizeof(float), &k2);
    ret = clSetKernelArg(kernel1, 10, sizeof(float), &k3);


    float Ratio=1.0;
    cl_kernel kernel2 = clCreateKernel(program, "interpolate_image", &ret);
    ret = clSetKernelArg(kernel2, 0, sizeof(cl_mem), (void *)&innerBuffer);
    ret = clSetKernelArg(kernel2, 1, sizeof(cl_mem), (void *)&outputBuffer);
    ret = clSetKernelArg(kernel2, 2, sizeof(cl_uint), &Ratio);
    // 执行内核函数
    size_t globalSize_i[2] = { inputWidth, inputHeight };
    size_t globalSize[2] = { outputWidth, outputHeight };
    size_t localSize[2] = { 1, 1 };
    // 从设备中读取输出图像
    cv::Mat outputImage(outputHeight, outputWidth, CV_8U);
    cv::Mat inneroutputImage(inputHeight, inputWidth, CV_8U);

    size_t origin[3] = { 0, 0, 0 };
    size_t region[3] = { outputWidth, outputHeight, 1 };
    size_t region1[3] = { inputWidth, inputHeight, 1 };
    size_t region_i[3] = { inputWidth, inputHeight, 1 };
    

        //unsigned char *data = (unsigned char *)clEnqueueMapImage(commandQueue, inputBuffer ,CL_TRUE,CL_MAP_READ,origin,region_i, &image_row_pitch, &image_slice_pitch,0,NULL,NULL,&ret);
        unsigned char *outputdata = (unsigned char *)clEnqueueMapImage(commandQueue,  outputBuffer, CL_TRUE, CL_MAP_READ, origin, region, &image_row_pitch, &image_slice_pitch, 0, NULL, NULL, &ret);
        //  unsigned char *data = (unsigned char *)clEnqueueMapImage(commandQueue, inputBuffer ,CL_TRUE,CL_MAP_READ,origin,region_i, &image_row_pitch, &image_slice_pitch,0,NULL,NULL,&ret);
        unsigned char *outputdata1 = (unsigned char *)clEnqueueMapImage(commandQueue,  innerBuffer, CL_TRUE, CL_MAP_READ, origin, region1, &image_row_pitch, &image_slice_pitch, 0, NULL, NULL, &ret);
        //std::cout<<ret<<std::endl;
    
       
    // 读取视频帧
    // cv::Mat frame; 
    cv::Mat FrameOutSend; 
    int size;
    cv::Mat grayFrame;
      cv::Mat graydFrame;

    rtracker = new realtracker("/home/rpdzkj/1/h265encode_test/exe/trackercfg.yaml");

    bbox_t detRet[OBJ_NUMB_MAX_SIZE];
    memset(detRet, 0x00, sizeof(*detRet));
    memset(g_detRet, 0x00, sizeof(*g_detRet));
    int boxes_count = 0;
    
    // 开启viewlink串口线程，开启sony串口线程
    std::thread serialThViewLink = std::thread(serialViewLinkFunc);
    
    std::thread serial137 = std::thread(serial137Func);
    serial137.detach();

    std::thread serialThSony = std::thread(serialSonyFunc);
    serialThSony.detach();
    serialThViewLink.detach();
 

    std::thread TCP2serialFuncTh = std::thread(TCP2serialFunc);
    TCP2serialFuncTh.detach();

    std::thread serial2TCPFuncTh = std::thread(serial2TCPFunc);
    serial2TCPFuncTh.detach();
    
    // 开启推流线程
    // std::thread osdAndSendRTSPStreamTh(osdAndSendRTSPStreamFunc);
    // osdAndSendRTSPStreamTh.detach();

    // // 开启保存视频的线程
    std::thread SaveRecordVideoTh(SaveRecordVideoFunc);
    SaveRecordVideoTh.detach();
    
    // // 开启检测跟踪视频的线程
    // std::thread detectAndTrackTh(detectAndTrackFunc);
    // detectAndTrackTh.detach();

    // // UDP接收上位机消息至串口线程
    std::thread UDP2serialFuncTh(UDP2serialFunc);
    UDP2serialFuncTh.detach();

    // UDP发送串口消息至上位机线程 
    std::thread serial2UDPFuncTh(serial2UDPFunc);
    serial2UDPFuncTh.detach();

    /*
        图像显示以及检测跟踪相关变量
    */
    // 需要目标检测的图像
    cv::Mat frame;

    // 帧数累加变量
    uint64_t nFrames = 0;

    // 初始化跟踪、检测以及显示模式
    stSysStatus.enDispMode = Vision;
    stSysStatus.trackOn = false;
    stSysStatus.detOn = false;
   // stSysStatus.detOn = true;
    stSysStatus.osdCtrl.dateSwitch = true;
    // stSysStatus.osdCtrl.EOwitch = true;

    // 画中画中小图像的起始点
    int pipPosX, pipPosY;

    // 可见光和红外图像的实例
    //Camera *visCam = CreateCamera(visi_dev, 1920, 1080, std::string("mipi"));
    Camera *visCam = CreateCamera("/dev/video0", 1920, 1080, std::string("mipi"));
    Camera *irCam = CreateCamera("/dev/video11", 640, 512, std::string("usb"));
    // Camera *irCam = CreateCamera(ir_dev, 640, 512, std::string("usb"));

    // 初始化红外图像
    cv::Mat oriIrImg = cv::Mat::zeros(640, 512, CV_8UC3);

    // 可见光图像
    cv::Mat rgbImg;

    int center_x, center_y;

    // 屏蔽上一次检测留在检测线程里的两帧检测框设立的检测帧数
    uint64_t DetFrameCount = 0;

    if (visCam != nullptr)
    {
        visCam->Init();
        printf("vis camera init success\n");
    }
    else
    {
        printf("vis camera init failed\n");
        return 0;
    }
    if (irCam != nullptr)
    {
        irCam->Init();
        printf("ir camera init success\n");
    }
    else
    {
        printf("ir camera init failed\n");
        return 0;
    }
    
    visCam->GetFrame(rgbImg);
    irCam->GetFrame(oriIrImg);
    
    if ( rgbImg.empty() ||
      oriIrImg.empty())
    {
        printf("input img empty, quit\n");
        return 0;
    }

    int irImgW = rgbImg.cols;
    int irImgH = rgbImg.rows;

    cv::Mat irImg = cv::Mat::zeros(1080, 1920, CV_8U);

    int oriImgW = 1350;
    int oriImgH = 1080;

    int viImgW = rgbImg.cols;
    int viImgH = rgbImg.rows;

    pipPosX = (rgbImg.cols - oriImgW) / 2;
    pipPosY = (rgbImg.rows - oriImgH) / 2;

    rtracker->setFrameScale((double)viImgW / 1920);
    // 跟踪状态

    memset(trackerStatus, 0, 9);

    // 等待上次录制完成，需要进行录制标志
    bool isNeedRecording = false;

    uint64_t servoCommandSendCount = 0;
    stSysStatus.osdSet1Ctrl.enOSDShow = true;
    struct timeval time;
    long tmpTime, lopTime;
    gettimeofday(&time, nullptr);
    tmpTime, lopTime = time.tv_sec * 1000 + time.tv_usec / 1000;

    auto write_interval = std::chrono::milliseconds(1000 / 30); // 30 fps
    auto last_write_time = std::chrono::steady_clock::now() - write_interval;
    int framequeue_size = 3;
    int framequeue_index = 0;

    cv::Rect trackRect;
    // 计算原始和均衡后的对比度
    double originalContrast;
    double equalizedContrast ;

    double w_lowerThreshold_ = 70;
    double w_upperThreshold_ = 230;
    double w_k1_ = 0.5;
    double w_k2_ = 1;
    double w_k3_ = (255 - w_lowerThreshold_*w_k1_-(w_upperThreshold_-w_upperThreshold_)*w_k2_)/(255-w_upperThreshold_);

    double  Threshold=70; 
    double  k =0.5;  

    cv::Mat ContrastEnhanceMat;
    // 创建CLAHE对象
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(1.0, cv::Size(16, 16));  //   块大小  
   

    while (!interrupted.load())
    {
        sw.reset();
        visCam->GetFrame(rgbImg);
        irCam->GetFrame(oriIrImg);

        if (rgbImg.empty()
         || oriIrImg.empty())
        {
            printf("input img empty, quit\n");
            continue;
        }
        cv::cvtColor(oriIrImg,FrameOutSend,cv::COLOR_BGR2GRAY);

        // cv::Rect crop_region(0, 0, 256, 192); // x, y, width, height
        // // 裁剪图片
        // oriIrImgGRAY = oriIrImgGRAY(crop_region);
        // FrameOutSend=oriIrImgGRAY.clone();
        // // // 设定裁剪区域的坐标和大小
        // cv::Rect crop_region(0, 0, 256, 192); // x, y, width, height
        // // 裁剪图片
        // FrameOutSend = FrameOutSend(crop_region);
        
        if(isNeedLinearTransformation) 
        {
            cv::Mat LinearTransformationMat;
            FrameOutSend.convertTo(FrameOutSend, CV_32FC1);
            LinearTransformationMat = applyPiecewiseLinearTransform(FrameOutSend,w_lowerThreshold_,w_upperThreshold_,w_k1_,w_k2_,w_k3_);   
            //LinearTransformationMat=LinearDarken(FrameOutSend,  Threshold,k);  
            FrameOutSend = LinearTransformationMat.clone();
            FrameOutSend.convertTo(FrameOutSend, CV_8UC1);
        }


        if( isNeedContrastEnhance ) 
        {
            clahe->apply(FrameOutSend, ContrastEnhanceMat);
            FrameOutSend=ContrastEnhanceMat.clone();
        }
            
        if(isNeedImageEnhance)
        {
            //  std::cout<<"图像增强" <<std::endl;            
            // // 设定裁剪区域的坐标和大小
            // cv::Rect crop_region(0, 0, 256, 192); // x, y, width, height
            // // 裁剪图片
            // grayFrame = frame(crop_region);
            // std::cout << "图像大小: " << grayFrame.size() << std::endl; // 打印图像大小
            cv::cvtColor(FrameOutSend,grayFrame,cv::COLOR_GRAY2BGR);
            cv::cvtColor(grayFrame,graydFrame,cv::COLOR_BGR2GRAY);

            /****opencl pro******/
            ret = clEnqueueWriteImage(commandQueue, inputBuffer, CL_TRUE, origin, region_i, 0, 0, graydFrame.data, 0, NULL, NULL); 
            // std::cout<<"111111111111111  :    "<<ret<<std::endl;
            size_t localSize_Enhance[2] = { 4, 4 };
            ret = clEnqueueNDRangeKernel(commandQueue, kernel1, 2, NULL, globalSize_i, localSize_Enhance, 0, NULL, NULL);
            // std::cout<<"222222222222  :    "<<ret<<std::endl;
            clFinish(commandQueue);
            ret = clEnqueueReadImage(commandQueue, innerBuffer, CL_TRUE, origin, region1, 0, 0, inneroutputImage.data, 0, NULL, NULL);
            // std::cout<<"333333333333  :    "<<ret<<std::endl;
            // std::cout << "11111111111111  尺寸 "<<inneroutputImage.size() << std::endl; // 打印图像大小
            FrameOutSend=inneroutputImage.clone();
        }
        if(isNeedSuperResolution)
        {
            mtx.lock(); // 加锁
            Ratio=SuperResolutionindex;
            ret = clSetKernelArg(kernel2, 2, sizeof(float), (float *)&Ratio);
            int  W_SuperResolution= int(640*SuperResolutionindex)-(int(640*SuperResolutionindex)%4);
            int  H_SuperResolution= int(512*SuperResolutionindex)-(int(512*SuperResolutionindex)%4);
            
            // int  W_SuperResolution= int(256*SuperResolutionindex);
            // int  H_SuperResolution= int(192*SuperResolutionindex);
            


            cv::Mat outputImage(H_SuperResolution, W_SuperResolution, CV_8U);
            size_t globalSize[2] = {W_SuperResolution, H_SuperResolution };

            size_t region_EN[3]={ 640, 512, 1 };
            ret = clEnqueueWriteImage(commandQueue, innerBuffer, CL_TRUE, origin, region_EN, 0, 0, FrameOutSend.data, 0, NULL, NULL);
            // if (int(640*SuperResolutionindex) % 4 == 0 &&  int(512*SuperResolutionindex)%4 == 0 ) 
            // {
                size_t localSize_SuperResolution[2] = { 4, 4 };
            // } 
            // else
            // {
            //     size_t localSize_SuperResolution[2] = { 1, 1 };
            // }
       
            ret = clEnqueueNDRangeKernel(commandQueue, kernel2, 2, NULL, globalSize, localSize_SuperResolution, 0, NULL, NULL);
            clFinish(commandQueue);
            size_t region[3] ={ W_SuperResolution, H_SuperResolution, 1 };     
        //  memcpy(outputImage.data, outputdata, outputWidth*outputHeight * outputImage.elemSize());
            ret = clEnqueueReadImage(commandQueue, outputBuffer, CL_TRUE, origin, region, 0, 0, outputImage.data, 0, NULL, NULL);
            FrameOutSend=outputImage.clone();
            mtx.unlock(); // 解锁
        }


        if(FrameOutSend.cols>1920 )
        {
            cv::Rect crop_region((FrameOutSend.cols-1920)/2, 0, 1920, FrameOutSend.rows); // x, y, width, height
            // 裁剪图片
            FrameOutSend = FrameOutSend(crop_region);

        }
        if(FrameOutSend.rows>1080 )
        {
            cv::Rect crop_region(0, (FrameOutSend.rows-1080)/2, FrameOutSend.cols, 1080); // x, y, width, height
            // 裁剪图片
            FrameOutSend = FrameOutSend(crop_region);

        }

        // cv::imwrite("./tes_Frame.jpg", FrameOutSend);
        // // // 按 'q' 退出循环
        // // if (cv::waitKey(40) == 'q') {
        // //     break;
        // // }
        // std::cout << "111122图像大小: " << FrameOutSend.size() << std::endl; // 打印图像大小
        // cvtIrImg(oriIrImg, stSysStatus.enIrImgMode);
        // stSysStatus.enDispMode = Ir;

        switch (stSysStatus.enDispMode)
        {
            case Vision: // 0x01
                frame = rgbImg;
                rtracker->setIrFrame(false);
                break;
            case Ir: // 0x02
                irImg.setTo(0);
                // std::cout << "图像宽度: " << FrameOutSend.size() << std::endl;
                FrameOutSend.copyTo(irImg(cv::Rect(1920/2-FrameOutSend.cols/2, 1080/2-FrameOutSend.rows/2, FrameOutSend.cols, FrameOutSend.rows)));
                // cv::imwrite("1_eEnhance.jpg",irImg);
                cv::cvtColor(irImg,frame,cv::COLOR_GRAY2BGR);
                trackerStatus[4] |= 0x01; // 0000 0001
                rtracker->setIrFrame(true);
                break;
            case VisIrPip: // 0x03
                cv::resize(oriIrImg, oriIrImg, cv::Size(480, 360));
                oriIrImg.copyTo(rgbImg(cv::Rect(viImgW - 480, 0, 480, 360)));
                frame = rgbImg;
                rtracker->setIrFrame(false);
                break;
            case IrVisPip: // 0x04
                cv::resize(rgbImg, rgbImg, cv::Size(480, 360));
                cv::resize(oriIrImg, oriIrImg, cv::Size(1350, 1080));
                irImg.setTo(0);
                oriIrImg.copyTo(irImg(cv::Rect(pipPosX, pipPosY, oriIrImg.cols, oriIrImg.rows)));

                rgbImg.copyTo(irImg(cv::Rect(irImgW - 480, 0, 480, 360)));
                frame = irImg;
                // frame = rgbImg;
                rtracker->setIrFrame(true);
                break;
            default:
                frame = rgbImg;
                break;
        }

        frameQueue.push(frame.clone());

        frameVar.notify_one();
        if (frameQueue.size() <= framequeue_size)
        {
            continue;
        }
        
        if (stSysStatus.enScreenOpMode == EN_SCREEN_OP_MODE::SCREEN_SHOOT)
        {
            stSysStatus.enScreenOpMode = EN_SCREEN_OP_MODE::SCREEN_NONE;
            isNeedTakePhoto = true;
        }
        else if (stSysStatus.enScreenOpMode == EN_SCREEN_OP_MODE::RECORDING_START && !isRecording)
        {
            stSysStatus.enScreenOpMode = EN_SCREEN_OP_MODE::SCREEN_NONE;

            isRecording = true;
            printf("start recording video\n");
            //std::string currTimeStr = CreateDirAndReturnCurrTimeStr("/home/rpdzkj/wjm/videos_recorded");
            //std::string saveVideoFileName = "/home/rpdzkj/wjm/videos_recorded/" + currTimeStr + ".mp4";
            if (fp == nullptr)
            {
                //writer = new cv::VideoWriter();
                if (globalstreamType == "h264")
                {
                    fp = fopen("output.h264", "wb+");
                }
                else if (globalstreamType == "h265")
                {
                    fp = fopen("output.h265", "wb+");
                }
            }
            
        }
        
        else if (stSysStatus.enScreenOpMode == EN_SCREEN_OP_MODE::RECORDING_END && isRecording)
        {
            stSysStatus.enScreenOpMode = EN_SCREEN_OP_MODE::SCREEN_NONE;
            printf("recording end\n");
            isRecording = false;
        }

        if (stSysStatus.trackOn)
        {
            if (framequeue_index < framequeue_size)
            {
                framequeue_index++;
                rtracker->runDetectorOut(frameQueue.back(), detRet, boxes_count);
            }
            else
            {
                if (!stSysStatus.trackerInited)
                {
                    // stSysStatus.detOn = false;
                    spdlog::debug("start tracking, init Rect:");
                    stSysStatus.trackerGateSize = stSysStatus.trackerGateSize * (sqrt(sqrt(zoomGrade)));
                    zoomGrade = 1;
                    // rtracker->setGateSize(stSysStatus.trackerGateSize);
                    rtracker->reset();
                    rtracker->init(stSysStatus.trackAssignPoint, frameQueue.front(), frameQueue.back());
                    stSysStatus.trackerInited = true;
                }
                else
                {
                    rtracker->update(frameQueue.back(), frameQueue.front(), trackerStatus, center_x, center_y, trackRect);
                    {
                        if(rtracker->trackerLost())
                        {
                            drawLostRect(frameQueue.front(), trackRect);
                        }
                        else
                            drawRect(frameQueue.front(), trackRect);
                            // cv::rectangle(frameQueue.front(), trackRect, cv::Scalar(255, 255, 255), 3, 8);
                    }

                    spdlog::debug("tracker status:{}", trackerStatus[4]);

                    TrackerMissDistanceResultFeedbackToDown(trackerStatus);
                    if (zoomFlag)
                    {
                        stSysStatus.trackAssignPoint.x = center_x;
                        stSysStatus.trackAssignPoint.y = center_y;
                        stSysStatus.trackerInited = false;
                        zoomFlag = false;
                    }
                    // if (stSysStatus.detOn)
                    // {
                    //     stSysStatus.trackOn = false;
                    // }

                    if(trackerStatus[4] == 0)
                    {
                        stSysStatus.trackOn = false;
                        stSysStatus.trackerInited = false;
                    }
                }
            }
        }
        else if (stSysStatus.detOn)
        {
            //  printf("stSysStatus.detOn\n");
            rtracker->runDetectorOut(frameQueue.back(), detRet, boxes_count);
            if (framequeue_index < framequeue_size)
            {
                framequeue_index++;
                boxes_count = 0;
            }
            for (int i = 0; i < boxes_count; ++i)
            {
                // 获取当前的 bbox_t 对象
                bbox_t &box = detRet[i];
                  //0:人  3:车
                // if(box.obj_id==0)
                //     continue;
                // if( (box.obj_id==0 && Detect_Person==1 ) || (box.obj_id==3 && Detect_Car==1 ) )
                // {
                    // 假设 predefinedColors 是已定义的颜色数组，box.obj_id 是 bbox_t 结构中的成员
                    size_t colorIndex = box.obj_id % predefinedColors.size();
                    cv::Scalar color = predefinedColors[colorIndex];
                
                    // 画矩形框
                    // cv::rectangle(frameQueue.front(), cv::Point(box.x, box.y), cv::Point(box.x + box.w, box.y + box.h), color, 2, 8);
                    drawRect(frameQueue.front(), cv::Rect(cv::Point(box.x, box.y), cv::Point(box.x + box.w, box.y + box.h)), color);

                    // 在框的左上角添加 obj_id 文本
                    // std::string id_text = std::to_string(box.obj_id);
                    // cv::putText(frameQueue.front(), id_text, cv::Point(box.x, box.y), cv::FONT_HERSHEY_SIMPLEX, 1, color, 2);
              // }
            }
        }
        else
        {
            framequeue_index = 0;
        }
        

        // spdlog::debug("before osd Elapsed {}", std::chrono::duration_cast<std::chrono::milliseconds>(sw.elapsed()));
        // sw.reset();
        if (!stSysStatus.osdCtrl.osdSwitch)
        {
            if (!stSysStatus.osdCtrl.attitudeAngleSwitch)
            {
                // 绘制吊舱当前方位角度滚轴
                PaintRollAngleAxis(frameQueue.front(), stSysStatus.rollAngle);

                // 绘制吊舱当前俯仰角度滚轴
                PaintPitchAngleAxis(frameQueue.front(), stSysStatus.pitchAngle);
            }
            if (!stSysStatus.osdCtrl.crosshairSwitch)
            {
                // 绘制中心十字`
                PaintCrossPattern(frameQueue.front(), stSysStatus.rollAngle, stSysStatus.pitchAngle);
            }
            if (!stSysStatus.osdCtrl.GPSSwitch)
            {
                // 绘制经纬度、海拔高度等坐标参数
                PaintCoordinate(frameQueue.front());
            }
    
            // 绘制界面上其他参数
            PaintViewPara(frameQueue.front());
            PaintWorkStatus(frameQueue.front(),ft2);  
            PaintAngle(frameQueue.front());
            //姿态角度信息
            Paint_Arhs(frameQueue.front());
            
            if (!stSysStatus.osdCtrl.MissToTargetSwitch)
            {
                // 绘制脱靶量
                PaintTrackerMissDistance(frameQueue.front());
            }
        }

        // spdlog::debug("after osd Elapsed {}", std::chrono::duration_cast<std::chrono::milliseconds>(sw.elapsed()));
        // sw.reset();

        {
            std::lock_guard<std::mutex> lock(frameMutex);
            cv::resize(frameQueue.front(), dispFrame, cv::Size(EncdoerWidth, EncdoerHeight), cv::INTER_NEAREST);
        }

        // spdlog::debug("after rs Elapsed {}", std::chrono::duration_cast<std::chrono::milliseconds>(sw.elapsed()));
        frameCond.notify_one();  // Notify the worker thread
        frameQueue.pop();
        nFrames++;

        if (nFrames % 60 == 0)
        {
            gettimeofday(&time, nullptr);
            tmpTime = time.tv_sec * 1000 + time.tv_usec / 1000;
            printf("!!<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<60帧平均帧率:\t%f帧\n", 60000.0 / (float)(tmpTime - lopTime));
            lopTime = tmpTime;
        }
    }


    clReleaseMemObject(inputBuffer);
    clReleaseMemObject(innerBuffer);
    clReleaseMemObject(gauss_kernel);

    clReleaseProgram(program);
    clReleaseKernel(kernel1);
    clReleaseCommandQueue(commandQueue);
    clReleaseContext(context);
    free(kernelSource);


    return 0;
}

void serialViewLinkFunc()
{
    uint8_t output[1024] = {0};
    int outLen = 0;
    uint8_t buffRcvData[1024] = {0};
    int retLen = 0;

    std::vector<uint8_t> receiveBuffer;
    const std::vector<uint8_t> frameStart = {0x55, 0xAA, 0xDC};
    while (!interrupted.load())
    {
        retLen = serialViewLink.serial_receive(buffRcvData);
        if (retLen > 0)
        {
            // std::cout << "=======>serialUp received " << std::dec << retLen << "bytes" << std::endl;
            receiveBuffer.insert(receiveBuffer.end(), buffRcvData, buffRcvData + retLen);
            // 处理粘包的情况
            {
                // 循环直到缓冲区数据不够一帧的最小长度
                while (receiveBuffer.size() > frameStart.size())
                {
                    // 查找帧起始标志
                    auto frameStartIt = std::search(receiveBuffer.begin(), receiveBuffer.end(), frameStart.begin(), frameStart.end());
                    if (frameStartIt != receiveBuffer.end())
                    {
                        // 检查是否有足够的数据读取长度字节
                        auto headerEndIt = frameStartIt + frameStart.size();
                        if (std::distance(headerEndIt, receiveBuffer.end()) >= 1)
                        {
                            // 读取长度字节，假设长度字节紧随帧起始后
                            size_t frameLength = *(headerEndIt) & 0x3F; // 取字节的低6位作为长度
                            // std::cout<<frameLength<<std::endl;
                            // 检查是否有足够的数据包含整个帧
                            if (std::distance(headerEndIt, receiveBuffer.end()) >= frameLength)
                            {
                                // 提取帧，包括帧头和数据
                                std::vector<uint8_t> frame(frameStartIt, headerEndIt + frameLength);
                                // 处理帧

                                EN_DATA_FRAME_TYPE frameType = GetFrameType(frame, frameLength + 3);
                                // printf("frame type:%d\n", frameType);
                                if (frameType == HeartBeat15)
                                {
                                    printf("heart beat 15 from up serial\n\n");
                                    receiveBuffer.erase(receiveBuffer.begin(), headerEndIt + frameLength);
                                    continue;
                                }
                                else if (frameType == CtrlSdCmd)
                                {
                                    // printf("CtrlSdCmd from up serial\n\n");
                                    receiveBuffer.erase(receiveBuffer.begin(), headerEndIt + frameLength);
                                    continue;
                                }
                                else if (frameType == IPInq)
                                {
                                    // printf("IPInq from up serial\n\n");
                                    receiveBuffer.erase(receiveBuffer.begin(), headerEndIt + frameLength);
                                    continue;
                                }
                                else if (frameType == HandShake)
                                {
                                    printf("handshake from up serial\n\n");
                                    receiveBuffer.erase(receiveBuffer.begin(), headerEndIt + frameLength);
                                    continue;
                                }
                                else if (frameType == FrameS2)
                                {
                                    // printf("TGCC Ctrl S2 from up serial\n\n");
                                    receiveBuffer.erase(receiveBuffer.begin(), headerEndIt + frameLength);
                                    continue;
                                }
                                // else if (frameType == FrameE2)
                                // {
                                //     printf("TGCC Ctrl E2 from up serial\n\n");
                                //     receiveBuffer.erase(receiveBuffer.begin(), headerEndIt + frameLength);
                                //     continue;
                                // }
                                else if (frameType == Status42)
                                {
                                    // printf("TGCC Ctrl 42 from up serial\n\n");
                                    receiveBuffer.erase(receiveBuffer.begin(), headerEndIt + frameLength);
                                    continue;
                                }
                                else if (frameType == HeartBeat14)
                                {
                                    printf("heart beat 14 from up serial\n\n");
                                    receiveBuffer.erase(receiveBuffer.begin(), headerEndIt + frameLength);
                                    continue;
                                }
                                for (int i = 0; i < frame.size(); i++)
                                {
                                    printf("[%02X]", frame[i]);
                                }
                                printf("\n");
                                uint8_t *usefulFrame = frame.data();
                                VL_ParseSerialData(usefulFrame);

                                // 移除处理过的数据
                                receiveBuffer.erase(receiveBuffer.begin(), headerEndIt + frameLength);
                            }
                            else
                            {
                                // 不完整的帧，退出循环等待更多数据
                                break;
                            }
                        }
                        else
                        {
                            // 不完整的头，退出循环等待更多数据
                            break;
                        }
                    }
                    else
                    {
                        // 未找到帧起始标志，清空缓冲区
                        receiveBuffer.clear();
                        break;
                    }
                }
            }
        }
    }
}

/****137项目串口通信协议****/
void serial137Func()
{
     int lSts;
     uint8_t buffRcvData[128] = {0};
     int retLen = 0;
     int i=0;

    // std::vector<uint8_t> receiveBuffer;
    // const std::vector<uint8_t> frameStart = {0xCC};

    struct pollfd stPollFd[1];
    stPollFd[0].fd = serial137Link.fdSerial;
    stPollFd[0].events = POLLIN;
    if(stPollFd[0].fd < 0)
    {
    	// WriteLog(LOG_ERR,"[%s]open error", DEV_NAME1);
        printf(" serial137Link.fdSerial  open error\n");
    	// goto OUT;
    }
    while (!interrupted.load())
    {
       // VideoProCommon.memberFunctionA();
        // lCurTime=VidePro_ComGetTime();
        lCurTime=VideoProCommon.VidePro_ComGetTime();
        TimeOut();
        lSts = poll((struct pollfd *)&stPollFd, 1, 1);
		if (0 == lSts)
		{
		    continue;
		}
		else if (lSts < 0)
		{
			if (EINTR == errno)
			{ /* 被中断打断 */
			    continue;
			}

			///goto OUT;
		}
		
        if (stPollFd[0].revents & POLLIN)
        {
            memset(buffRcvData, 0, 128);
            retLen = read(stPollFd[0].fd, buffRcvData, 128);
            if (retLen <0)
            {
                // usleep(50);
                // WriteLog(LOG_ERR,"receive msg is failed!\n");
            }

            printf("接受长度%d \n",retLen);
            for(int i=0;i<retLen;i++)
                printf("接受字符串：%02x\r\n", buffRcvData[i]);
             for(int i=0;i< retLen-8;i++) 
             {
                if(buffRcvData[i]==0xCC && buffRcvData[i+8]==project137_serial_checksum(buffRcvData+i+1,7))
                {
                //    printf("进入处理%02x \n",buffRcvData[i+1]);
                    project137_ParseSerialData(buffRcvData+i+1);
                }
             }

        }
    }

   close(stPollFd[0].fd);

}


// 序列化和反序列化函数
uint64_t serializeBytes(unsigned char *bytes)
{

    uint64_t value = 0;
    // std::cout<<std::hex<<bytes[0]<<bytes[1]<<bytes[2]<<bytes[3]<<std::endl;
    memcpy(&value, bytes, sizeof(uint32_t));
    return value;
}

void deserializeBytes(uint64_t value, unsigned char *bytes)
{
    memcpy(bytes, &value, sizeof(uint32_t));
}

uint32_t swapEndian(uint32_t value)
{
    uint32_t result = ((value & 0xFF000000) >> 24) |
                      ((value & 0x00FF0000) >> 8) |
                      ((value & 0x0000FF00) << 8) |
                      ((value & 0x000000FF) << 24);
    return result;
}

void serialSonyFunc()
{

    uint8_t buffRcvData[1024] = {0};
    int retLen = 0;
    std::vector<uint8_t> receiveBuffer;
    uint8_t zoom[10][4] = {
        {0x00, 0x00, 0x00, 0x00},
        {0x01, 0x06, 0x0C, 0x05},
        {0x02, 0x00, 0x08, 0x05},
        {0x02, 0x06, 0x04, 0x05},
        {0x02, 0x0A, 0x04, 0x05},
        {0x02, 0x0D, 0x04, 0x00},
        {0x02, 0x0F, 0x08, 0x00},
        {0x03, 0x01, 0x08, 0x00},
        {0x03, 0x03, 0x04, 0x00},
        {0x03, 0x04, 0x0C, 0x00}};
    while (!interrupted.load())
    {
        memset(buffRcvData, 0, sizeof(buffRcvData));
        retLen = serialRevSony.serial_receive(buffRcvData);
        bool found = false;

        if (retLen > 0)
        {
            for (int i = 0; i < retLen; i++)
            {
                printf("[%02X]", buffRcvData[i]);
            }
            // buffRcvData[retLen] = '\r';
            printf("\n");
            serialSendSony.serial_send(buffRcvData, retLen);

            for (int i = 0; i < 10; ++i)
            { // 遍历 zoom 数组的每一行
                if (zoom[i][0] == buffRcvData[4] &&
                    zoom[i][1] == buffRcvData[5] &&
                    zoom[i][2] == buffRcvData[6] &&
                    zoom[i][3] == buffRcvData[7])
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    zoomFlag = true;
                    std::cout << "Matching index in zoom array: " << i << std::endl;
                    zoomGrade = i + 1;
                    break; // 匹配后退出循环
                }
            }
        }
    }
}

/****************************************************************************************
 * 函 数 名 ： record_FFmpegcolsefile
 * 功    能 ： 关闭ffmpeg创建的文件句柄
 * 输入参数 ： 无
 * 输出参数 ： 无
 * 返 回 值 ： 0: 成功；-1: 失败
 ***************************************************************************************/
void record_FFmpegcolsefile()
{
    int lRet=0;
    if(record_pOutFormatCtx == NULL)
    {
        printf("record_pOutFormatCtx ==NULL");
        goto OUT;
    }
    if (record_pOutFormatCtx != NULL)
    {
		lRet=av_write_trailer(record_pOutFormatCtx);
        if( lRet < 0 )
        {
           printf("av_write_trailer error");
        }
    }
	if (record_pOutFormatCtx && !(record_pOutFormatCtx->oformat->flags & AVFMT_NOFILE))
    {
		lRet=avio_close(record_pOutFormatCtx->pb);// for new : avio_close(m_pOc->pb);
		if( lRet < 0 )
        {
           printf("avio_close error");
        }
    }
	if (record_pOutFormatCtx != NULL)
    {
		avformat_free_context(record_pOutFormatCtx);
		record_pOutFormatCtx = NULL;
    }
    // utl_system("sync");
	// usleep(100);
	// fflush_drop_cache();
    // system("sync");
    // system("sync");
OUT:    
   return ;
}


/****************************************************************************************
 * 函 数 名 ： record_FFmpegRecordfileInit
 * 功    能 ： 初始化录像句柄
 * 输入参数 ： 无
 * 输出参数 ： 无
 * 返 回 值 ： 0: 成功；-1: 失败
 ***************************************************************************************/
void record_FFmpegRecordfileInit(char *pData)
{
   	struct timeval starttime;
	time_t lTimeT;
    struct tm *tmDate;

	memset(&tmDate, 0, sizeof(tmDate));
    time(&lTimeT);
	tmDate = localtime(&lTimeT);
  
	gettimeofday(&starttime, NULL);

    char strFilePath[128]={0};
    sprintf(strFilePath, "/home/rpdzkj/%.4d%.2d%.2d_%.2d%.2d%.2d%s", tmDate->tm_year + 1900, tmDate->tm_mon + 1, tmDate->tm_mday, tmDate->tm_hour, tmDate->tm_min, tmDate->tm_sec,  ".mp4");

     AVOutputFormat* fmt;
     AVStream *outStream;
    /********************** walker ：创建文件节点 *****************/
    avformat_alloc_output_context2(&record_pOutFormatCtx, NULL, NULL, strFilePath);
    if (!record_pOutFormatCtx) 
    {
        printf( "Could not create output context -------error \r\n");
        goto end;
    }

    outStream = avformat_new_stream(record_pOutFormatCtx, NULL);
     if (globalstreamType == "h264")
    {
        outStream->codecpar->codec_id   = AV_CODEC_ID_H264;
    }
    else if (globalstreamType == "h265")
    {
            outStream->codecpar->codec_id   = AV_CODEC_ID_H265;
        // 	if (strncmp(YV_GLOBAL(lrecordType),".avi",4)==0)  //等于avi时
        //   	{
        //         //仅仅avi H265时需要写入此参数
        //         outStream->codecpar->codec_tag = MKTAG('H', '2', '6', '5');
        //     }
    }
    else
    {
        goto end;
    }
	record_pOutFormatCtx->streams[0]->time_base=(AVRational){ 1, 60 };
    outStream->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    outStream->codecpar->width      = 1920;
    outStream->codecpar->height     = 1080;
	outStream->codecpar->bit_rate=4000;
    //outStream->avg_frame_rate.den=1;
    //outStream->avg_frame_rate.num=25;  //帧率
    outStream->id                   = record_pOutFormatCtx->nb_streams - 1;

    if (globalstreamType == "h264")
    {	
        outStream->codecpar->extradata  = (unsigned char *)malloc(53);
        memcpy(outStream->codecpar->extradata, pData, 53);
        outStream->codecpar->extradata_size = 53;
     }
     else
     {
        outStream->codecpar->extradata  = (unsigned char *)malloc(89);
        memcpy(outStream->codecpar->extradata, pData, 89);
        outStream->codecpar->extradata_size = 89; 
     }
     fmt = record_pOutFormatCtx->oformat;
     if (!(fmt->flags & AVFMT_NOFILE))
     {
		
         ret = avio_open(&record_pOutFormatCtx->pb, strFilePath, AVIO_FLAG_WRITE);
         if (ret < 0) {
            printf( "Could not open output file '%s'", strFilePath);
             goto end;
         }
     }

     ret = avformat_write_header(record_pOutFormatCtx, NULL);
     if (ret < 0)
     {
        printf( "写入头失败 avformat_write_header");
     }
end:
 return ;
}


bool IsFirstFrame( char* p)
{
    if (globalstreamType == "h265")
    {
        return p[0] == 0 && p[1] == 0 && p[2] == 0 && p[3] == 1 && p[4] == 0x40;//VPS
    }
    else if (globalstreamType == "h264")
    {
        return p[0] == 0 && p[1] == 0 && p[2] == 0 && p[3] == 1 && p[4] == 0x67;//SPS
    }
    return false;
}
static  long  double ffmpegTimeStamp=0;  /*ffmpeg写入时标*/
/****************************************************************************************
 * 函 数 名 ： record_FFmpegWriteDate
 * 功    能 ： ffmpeg接口写视频文件
 * 输入参数 ： 无
 * 输出参数 ： 无
 * 返 回 值 ： 0: 成功；-1: 失败
 ***************************************************************************************/
int record_FFmpegWriteDate(char *pData,int dwDataLen)
{
	int ret;
     //取帧塞入进行封装
    AVPacket pkt1, *pkt = &pkt1;//为pkt创建内存不包含data的内存
    if( record_pOutFormatCtx==NULL)
    {
        printf("FFmpegWriteDate record_pOutFormatCtx NULL");
        goto OUT;
    } 

    av_new_packet(pkt, dwDataLen);//为pkt->data创建内存
    pkt->flags |= IsFirstFrame(pData) ? AV_PKT_FLAG_KEY : 0;   
    pkt->stream_index = 0;
    memcpy(pkt->data, pData, dwDataLen);
    pkt->size = dwDataLen;
    ffmpegTimeStamp+= ( record_pOutFormatCtx->streams[0]->time_base.den*1.0)/60;  //帧率
    pkt->pts=ffmpegTimeStamp;
    pkt->dts = pkt->pts;
    ret= av_interleaved_write_frame( record_pOutFormatCtx, pkt);
    if (ret < 0)
    {
       printf("av_interleaved_write_frame   error \n");
        goto OUT;
    }
    av_packet_unref(pkt);
 OUT:        
    //引用计数-1释放data内存
    return true;
}
int SaveRecordVideoFunc()
{
        std::unique_lock<std::mutex> lock;
        
         AVRational time_base;
                 char recordflag=0;
     
    lock = std::unique_lock<std::mutex>(frameMutex);  // 现在锁定互斥体
    while (!interrupted.load())
    {


        /**********walker:    保存照片***************/
        if (isNeedTakePhoto)
        {
            std::string currTimeStr = CreateDirAndReturnCurrTimeStr("photos");
            std::string savePicFileName = "/home/rpdzkj/" + currTimeStr + ".png";
           //std::string savePicFileName = "/home/rpdzkj/test.png";
            cv::imwrite(savePicFileName, frameQueue.front());
            isNeedTakePhoto = false;
        }

        frameCond.wait(lock, []{ return !dispFrame.empty(); });
        // Process the frame
        cv::cvtColor(dispFrame, yuvImg, cv::COLOR_BGR2YUV_I420);

        dispFrame.release();  // Clear the frame after processing
        lock.unlock();
        // continue;
        img = reinterpret_cast<char*>(yuvImg.data);

       // mppenc.encode(img, 1382400, pdst, &length);     //1382400=1280*720*1.5
        mppenc.encode(img, EncdoerWidth*EncdoerHeight*1.5, pdst, &length);     //1382400=1280*720*1.5
        if(isRecording )
        {   
            if(recordflag==0 )
            {  
                record_FFmpegRecordfileInit(dst);
                recordflag=1;
            }
            /****ffmpeg接口写视频文件****/
            record_FFmpegWriteDate(dst,length);
        }
        else
        {
           if(record_pOutFormatCtx != NULL)
            record_FFmpegcolsefile();
            recordflag=0;
        }

        // if (isRecording)
        // {
        //     if (fp != nullptr )
        //     {
        //         {
        //             fwrite(dst, length,1,  fp);
        //         }
        //     }
        // }

        // // if (!isRecording && writer != nullptr)
        // if (!isRecording && fp != nullptr)
        // {
        //     fclose(fp);
        //     fp = nullptr;

        // }

        ph264frame = static_cast<uint8_t*>(static_cast<void*>(pdst));

        h264frame_size=length;

        av_init_packet(&pkt);
        pkt.data = ph264frame; // ph264frame
        pkt.size = h264frame_size; // pdst_size是您的H264帧数据的大小

        // 设置流索引
        pkt.stream_index = out_stream->index;

        // 设置PTS和DTS
        // 注意: 实际值应根据您的具体需求来设置

        time_base = out_stream->time_base;
       // printf(" walker: ------------  %d   %d --- \n",time_base.den,time_base.num);       
 
        frame_duration = AV_TIME_BASE / fps;
        pts_time = frame_index * frame_duration;
        pkt.pts = av_rescale_q(pts_time, AVRational{1, AV_TIME_BASE}, time_base);
        pkt.dts = pkt.pts;
        pkt.duration = av_rescale_q(frame_duration, AVRational{1, AV_TIME_BASE}, time_base);
        // 写入帧
        if (av_interleaved_write_frame(ofmt_ctx, &pkt) < 0) {
            fprintf(stderr, "Error muxing packet\n");
            // 错误处理
        }
        
        // 清理
        av_packet_unref(&pkt);

        frame_index++; // 增加帧索引
        //printf("Frame %d sent\n", frame_index);
        lock.lock();
        
    }
    //   record_FFmpegcolsefile();
    return 0;

}
