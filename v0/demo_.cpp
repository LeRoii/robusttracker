#include "multitracker.h"
#include <unistd.h>
#include <signal.h>
// #include "jetsonEncoder.h"
#include "serial.h"
#include "common.h"
#include <sstream>
#include <cmath>
#include "camera.h"
#include "painter.h"
#include <arpa/inet.h>
#include <sys/vfs.h>
#include <yaml-cpp/yaml.h>
#include "realtracker.h"
#include "spdlog/spdlog.h"
#include "spdlog/stopwatch.h"
#include <deque>
#include <numeric>
#include <queue>

static std::vector<std::vector<uint8_t>> CmdNeedProcess =
    {
        {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x81, 0x00, 0x00, 0x00, 0xAC}, // EO(热像白热)
        {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x83, 0x00, 0x00, 0x00, 0xAE}, // EO+IR(白热)
        {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x84, 0x00, 0x00, 0x00, 0xA9}, // IR(白热)+EO
        {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x82, 0x00, 0x00, 0x00, 0xAF}, // IR(白热)

        {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xC1, 0x00, 0x00, 0x00, 0xEC}, // EO(热像黑热)
        {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xC3, 0x00, 0x00, 0x00, 0xEE}, // EO+IR(黑热)
        {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xC4, 0x00, 0x00, 0x00, 0xE9}, // IR(黑热)+EO
        {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xC2, 0x00, 0x00, 0x00, 0xEF}, // IR(黑热)

        {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x81, 0x00, 0x00, 0x00, 0x7C}, // EO(热像红热)
        {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x83, 0x00, 0x00, 0x00, 0x7E}, // EO+IR(红热)
        {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x84, 0x00, 0x00, 0x00, 0x79}, // IR(红热)+EO
        {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x82, 0x00, 0x00, 0x00, 0x7F}, // IR(红热)

        {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0xD0, 0x00, 0x00, 0x00, 0xF8}, // 热像放大间隔1倍
        {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x10, 0x00, 0x00, 0x00, 0x39}, // 热像缩小间隔1倍

        {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0xD0, 0x00, 0x00, 0x00, 0xFA}, // 拍照
        {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x10, 0x00, 0x00, 0x00, 0x3B}, // 录像开始
        {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x50, 0x00, 0x00, 0x00, 0x7B}, // 录像结束

        {0x55, 0xAA, 0xDC, 0x05, 0x01, 0x05, 0x00, 0x01}, // 打开OSD
        {0x55, 0xAA, 0xDC, 0x05, 0x01, 0x05, 0x01, 0x00}, // 关闭OSD

        {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x24}, // 开始跟踪
        {0x55, 0xAA, 0xDC, 0x06, 0x1E, 0x00, 0x01, 0x00, 0x19},                                                                   // 结束跟踪

        {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x01, 0x2A}, // 打开识别
        {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x2B}, // 关闭识别

};

#define DEBUG_SERIAL 0

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

bool quit = false;

static void signal_handle(int signum)
{
    quit = true;
    // _xdma_reader_ch0.xdma_close();
}

bool IsInCmdNeedProcess(uint8_t *buf, int len)
{
    if (buf[4] == 0x5D)
        return true;

    switch (buf[4])
    {
    case 0x30:
    case 0x01:
    case 0x1E:
        for (auto &cmd : CmdNeedProcess)
        {
            int find = true;
            for (int i = 0; i < len; ++i)
            {
                if (buf[i] != cmd[i])
                {
                    find = false;
                    break;
                }
            }
            if (find)
                return true;
        }

    default:
        return false;
        break;
    }
}

bool IsTransparentToPod(uint8_t *buf, int len)
{
    // if (buf == nullptr) {
    //     return false;
    // }

    // if (buf[4] == 0x5D) {
    //     return false;
    // }
    // return true;

    if (len == 8 || len == 9 || len == 20)
    {
        return !IsInCmdNeedProcess(buf, len);
    }

    return true;
}

Serial serialUp, serialDown;

void OnceSendFromDownToUp(uint8_t *buf)
{
    uint8_t sendBuf[1024] = {0};
    int sendBufLen = 7;

    sendBuf[0] = 0x55;
    sendBuf[1] = 0xAA;
    sendBuf[2] = 0xDC;

    if (buf[4] == 0x5D)
    {
        struct statfs diskInfo;
        statfs("/", &diskInfo);
        unsigned long long totalBlocks = diskInfo.f_bsize;
        unsigned long long totalSize = totalBlocks * diskInfo.f_blocks;
        uint32_t mbTotalsize = totalSize >> 20;
        unsigned long long freeDisk = diskInfo.f_bavail * totalBlocks;
        uint32_t mbFreedisk = freeDisk >> 20;
        sendBufLen = 10;

        sendBuf[3] = 0x5 + 0x3;
        sendBuf[4] = 0xD5;

        if (buf[5] == 0x8B)
        {
            sendBuf[5] = buf[6] - 1;
            if (buf[6] == 0x2)
            {
                if (mbTotalsize > 0 && mbFreedisk > 100)
                {
                    sendBuf[6] = 0x1;
                }
                else if (mbTotalsize == 0 || mbFreedisk == 0)
                {
                    sendBuf[6] = 0x80;
                }
                else
                {
                    sendBuf[6] = 0x20;
                }
            }
            else if (buf[6] == 0x3)
            {
                mbTotalsize = ntohl(mbTotalsize);
                memcpy(&sendBuf[6], &mbTotalsize, 4);
            }
            else if (buf[6] == 0x4)
            {
                mbFreedisk = ntohl(mbFreedisk);
                memcpy(&sendBuf[6], &mbFreedisk, 4);
            }
            else if (buf[6] == 0x5)
            {
                uint32_t photoGraphNum = mbFreedisk / 1.5;
                photoGraphNum = ntohl(photoGraphNum);
                memcpy(&sendBuf[6], &photoGraphNum, 4); // 按照每张照片1MB计算，剩余拍照张数
            }
            else if (buf[6] == 0x6)
            {
                uint32_t recordVideoTime = mbFreedisk / 1.5;
                recordVideoTime = ntohl(recordVideoTime);
                memcpy(&sendBuf[6], &recordVideoTime, 4); // 按照每秒视频1.5MB计算，剩余录像时间，单位sec
            }
        }

        uint8_t checksum = viewlink_protocal_checksum(sendBuf);
        sendBuf[sendBufLen - 1] = checksum;
        sendBufLen = serialUp.serial_send(sendBuf, sendBufLen);
        printf("down send to up:%d\n", sendBufLen);
    }
}

void SerialTransUp2Down()
{
    cpu_set_t mask;
    int cpuid = 4;

    CPU_ZERO(&mask);
    CPU_SET(cpuid, &mask);
    if (pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask) < 0)
    {
        std::cout << "set thread affinity failed" << std::endl;
    }

    uint8_t output[1024] = {0};
    int outLen = 0;
    uint8_t buffRcvData_servo[1024] = {0};
    while (1)
    {
        int retLen = serialUp.serial_recieve(buffRcvData_servo);
        if (retLen > 0)
        {
            std::cout << "=======>serialUp received " << std::dec << retLen << "bytes" << std::endl;
#if DEBUG_SERIAL
            for (int i = 0; i < retLen; i++)
            {
                printf("[%02X]", buffRcvData_servo[i]);
            }
            printf("\n");
#endif
            {
                retLen = serialDown.serial_send(buffRcvData_servo, retLen);
                printf("up send to down:%d\n", retLen);
                // continue;
            }
            // if (IsTransparentToPod(buffRcvData_servo)) {
            //     retLen = serialDown.serial_send(buffRcvData_servo, retLen);
            // }

            // printf("up send to down:%d\n", retLen);
            int aa = 5;
            while (aa--)
            {
                int rr = serialUp.ProcessSerialData(buffRcvData_servo, retLen, output, outLen);

                if (rr == RET_ERR)
                {
                    printf("RET_ERR\n");
                    break;
                }
                retLen = 0;

                EN_DATA_FRAME_TYPE frameType = GetFrameType(output, outLen);
                printf("frame type:%d\n", frameType);
                if (frameType == HeartBeat15)
                {
                    printf("heart beat 15 from up serial\n\n");
                    continue;
                }
                else if (frameType == HandShake)
                {
                    printf("handshake from up serial\n\n");
                    continue;
                }
                else if (frameType == FrameS2)
                {
                    printf("TGCC Ctrl S2 from up serial\n\n");
                    continue;
                }
                else if (frameType == HeartBeat14)
                {
                    printf("heart beat 14 from up serial\n\n");
                    continue;
                }
#if DEBUG_SERIAL
                printf("Up2Down output:\n");
                for (int i = 0; i < outLen; i++)
                {
                    printf("[%02X]", output[i]);
                }
                printf("\n");
#endif
                VL_ParseSerialData(output);
                OnceSendFromDownToUp(output);

                printf("status->enDispMode:%d, detOn:%d, trackOn:%d, trackerGateSize:%d\n",
                       stSysStatus.enDispMode, stSysStatus.detOn, stSysStatus.trackOn, stSysStatus.trackerGateSize);
                printf("\n\n");

                // continue;

                // printf("output buf\n");
                // for(int i=0; i< outLen ;i++)
                // {
                //     printf("[%02X]", output[i]);
                // }
                // printf("\n");

                // check frame header
                //  if(!CheckFrameHeader(buffRcvData_servo, retLen))
                //  {
                //      for(int i=0; i< retLen ;i++)
                //      {
                //          printf("[%02X]", buffRcvData_servo[i]);
                //      }
                //      printf("\n");
                //      printf("frame header error, drop data\n\n");
                //      memset(buffRcvData_servo,0,1024);
                //      continue;
                //  }

                // EN_DATA_FRAME_TYPE frameType = GetFrameType(output, outLen);
                // printf("frame type:%d\n", frameType);
                // if(frameType == HeartBeat15)
                // {
                //     printf("heart beat 15 from up serial\n\n");
                //     continue;
                // }
                // else if(frameType == HandShake)
                // {
                //     printf("handshake from up serial\n\n");
                //     continue;
                // }
                // else if(frameType == FrameS2)
                // {
                //     printf("TGCC Ctrl S2 from up serial\n\n");
                //     continue;
                // }
                // else if(frameType == HeartBeat14)
                // {
                //     printf("heart beat 14 from up serial\n\n");
                //     continue;
                // }

                // for(int i=0; i< outLen ;i++)
                // {
                //     printf("[%02X]", output[i]);
                // }
                // printf("\n");

                // for(int i=0; i< retLen ;i++)
                // {
                //     printf("[%02X]", buffRcvData_servo[i]);
                // }
                // printf("\n");

                // checksum
                //  uint8_t checksum = viewlink_protocal_checksum(buffRcvData_servo);
                //  if(checksum != buffRcvData_servo[retLen - 1])
                //  {
                //      printf("frame checksum error, drop data\n\n");
                //      memset(buffRcvData_servo,0,1024);
                //      continue;
                //  }

                // VL_ParseSerialData(output);
                // if (!IsTransparentToPod(buffRcvData_servo)) {
                //     OnceSendFromDownToUp(output);
                // }

                // printf("status->enDispMode:%d, detOn:%d, trackOn:%d, trackerGateSize:%d\n",\
            //  stSysStatus.enDispMode, stSysStatus.detOn, stSysStatus.trackOn, stSysStatus.trackerGateSize);
                // printf("\n\n");
                if (serialDown.bufLen == 0)
                {
                    printf("parse buf over\n");
                    break;
                }
            }
        }
    }
}

void SerialTransDown2Up()
{
    cpu_set_t mask;
    int cpuid = 4;

    CPU_ZERO(&mask);
    CPU_SET(cpuid, &mask);
    if (pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask) < 0)
    {
        std::cout << "set thread affinity failed" << std::endl;
    }

    uint8_t buffRcvData_servo[1024] = {0};
    uint8_t output[1024] = {0};
    int outLen = 0;
    while (1)
    {
        int retLen = serialDown.serial_recieve(buffRcvData_servo);
        if (retLen > 0)
        {
            std::cout << "<===========serial down received " << std::dec << retLen << "bytes" << std::endl;
#if DEBUG_SERIAL
            for (int i = 0; i < retLen; i++)
            {
                printf("[%02X]", buffRcvData_servo[i]);
            }
            printf("\n");
#endif
            retLen = serialUp.serial_send(buffRcvData_servo, retLen);
            printf("down send to up:%d\n", retLen);
            int aa = 5;
            while (aa--)
            {
                int rr = serialDown.ProcessSerialData(buffRcvData_servo, retLen, output, outLen);
                if (rr == RET_ERR)
                {
                    printf("RET_ERR\n");
                    break;
                }
                retLen = 0;
#if DEBUG_SERIAL
                printf("output buf\n");
                for (int i = 0; i < outLen; i++)
                {
                    printf("[%02X]", output[i]);
                }
                printf("\n");
#endif

                // continue;

                // if(!CheckFrameHeader(buffRcvData_servo, retLen))
                // {
                //     for(int i=0; i< retLen ;i++)
                //     {
                //         printf("[%02X]", buffRcvData_servo[i]);
                //     }
                //     printf("\n");
                //     printf("frame header error, drop data\n\n");
                //     memset(buffRcvData_servo,0,1024);
                //     continue;
                // }

                VL_ParseSerialData(output);

                EN_DATA_FRAME_TYPE frameType = GetFrameType(output, outLen);

                if (frameType == Status41 || frameType == Status40)
                {
                    printf("Status from down serial frameType:%d\n\n", frameType);
                    continue;
                }

                printf("!!!non Status frametype\n");
                for (int i = 0; i < outLen; i++)
                {
                    printf("[%02X]", output[i]);
                }
                printf("\n");

                // for(int i=0; i< retLen ;i++)
                // {
                //     printf("[%02X]", buffRcvData_servo[i]);
                // }
                // printf("\n");

                printf("\n\n");
                if (serialDown.bufLen == 0)
                {
                    printf("parse buf over\n");
                    break;
                }
            }
        }
    }
}

int sockfd;
int clientfd;
int clientSocketfd;
bool tcpAcc = false;

std::mutex m_mtxTcp;
std::condition_variable conVarTcp;

void TCPTransUp2Down()
{
    printf("TCPTransUp2Down\n");
    // 接收上位机消息，并作为客户端向吊舱转发消息
    sockfd = socket(AF_INET, SOCK_STREAM, 0); // 1、创建接收上位机消息套接字

    std::string cfgpath = "/etc/jetsoncfg/pl/config.yaml";
    YAML::Node config = YAML::LoadFile(cfgpath);
    const std::string upToNxIp = config["upToNxIp"].as<std::string>();
    uint16_t upToNxPort = config["upToNxPort"].as<uint16_t>();
    printf("upToNxIp:%s upToNxPort:%d\n", upToNxIp.c_str(), upToNxPort);
    struct sockaddr_in serveraddr;
    serveraddr.sin_addr.s_addr = inet_addr(upToNxIp.c_str());
    serveraddr.sin_port = htons(upToNxPort);
    serveraddr.sin_family = AF_INET;
    bind(sockfd, (struct sockaddr *)&serveraddr, sizeof(serveraddr)); // 2、绑定套接字

    listen(sockfd, 10); // 3、监听套接字
    char buf[1024];

    while (!quit)
    {
        if (!tcpAcc)
        {
            printf("TCPTransUp2Down wait accept\n");
            clientfd = accept(sockfd, NULL, NULL); // 4、接受客户端连接，返回值就是与客户端通信的套接字
            int flags = fcntl(clientfd, F_GETFL, 0);
            fcntl(clientfd, F_SETFL, flags & ~O_NONBLOCK); // 设置成阻塞模式；
            tcpAcc = true;
            conVarTcp.notify_all();
            printf("TCPTransUp2Down accepted, tcpAcc true\n");
        }
        else
        {
            memset(buf, 0, 1024);
            int num = recv(clientfd, buf, 1024, 0); // 返回值就是接收的大小
            if (num > 0)
            {
                printf("TCPTransUp2Down size is %d \n TCPTransUp2Down buf:", num);
                for (int i = 0; i < num; i++)
                {
                    printf("[%02X]", buf[i]);
                }
                printf("\n");

                int ret = send(clientSocketfd, buf, num, 0);
                printf("TCPTransUp2Down send to down, ret=%d\n", ret);
            }
            else
            {
                tcpAcc = false;
            }
        }
    }
    close(clientfd);
    close(sockfd);
    close(clientSocketfd);
}

void TCPTransDown2Up()
{
    printf("TCPTransDown2Up\n");
    char buf[1024];

    while (!quit)
    {
        if (!tcpAcc)
        {
            std::unique_lock<std::mutex> lock(m_mtxTcp);
            printf("TCPTransDown2Up wait accept\n");
            conVarTcp.wait(lock);
            printf("TCPTransDown2Up accepted\n");
        }
        else
        {
            memset(buf, 0, 1024);
            int num = recv(clientSocketfd, buf, 1024, 0); // 返回值就是接收的大小
            if (num > 0)
            {
                printf("TCPTransDown2Up size is %d \n TCPTransDown2Up buf:", num);
                for (int i = 0; i < num; i++)
                {
                    printf("[%02X]", buf[i]);
                }
                printf("\n");

                int ret = send(clientfd, buf, num, 0);
                printf("TCPTransDown2Up send to up ret=%d\n", ret);
            }
        }
    }

    close(clientfd);
    close(sockfd);
    close(clientSocketfd);
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

// AI识别结果向上位机反馈
static void DetectorResultFeedbackToUp(vector<TrackingObject> &dets)
{
    ST_F3_CONFIG f3Cfg = {0};
    f3Cfg.targetSum = dets.size();
    f3Cfg.totalPacketNum = f3Cfg.targetSum / 4; // 每个帧包最多只能放4个目标，超过4个目标需要分包向上位机发送
    if ((f3Cfg.targetSum % 4) > 0)
    {
        f3Cfg.totalPacketNum += 1;
    }

    for (int i = 0; i < f3Cfg.totalPacketNum; i++)
    {
        f3Cfg.currPacketId = i;
        int currPacketTargetNum = 4;
        if (f3Cfg.totalPacketNum - i == 1)
        {
            currPacketTargetNum = f3Cfg.targetSum % 4;
        }
        uint8_t sendBuf[1024] = {0};
        int sendBufLen = 5 + 8 + currPacketTargetNum * 13 + 1; // 帧头5字节,F3目标总体信息占8字节,每个目标具体信息占13字节,最后1个字节为checksum

        sendBuf[0] = 0x55;
        sendBuf[1] = 0xAA;
        sendBuf[2] = 0xDC;
        sendBuf[3] = ((i + 1) << 6) ^ (2 + currPacketTargetNum * 13 + 1);
        sendBuf[4] = 0xF3;
        sendBuf[5] = f3Cfg.targetSum;
        sendBuf[6] = f3Cfg.totalPacketNum;
        sendBuf[7] = f3Cfg.currPacketId;
        sendBuf[8] = 0;
        sendBuf[9] = 0;
        sendBuf[10] = 0;
        sendBuf[11] = 0;
        sendBuf[12] = 0;
        int pos = 13;
        for (int j = 0; j < currPacketTargetNum; j++)
        {
            f3Cfg.targetInfo[i + j].targetType = (dets[i + j].m_type == 1) ? 0 : 1; // 目标类型与向上位机传递的目标类型相反，且目前只有0，1 人、车两种
            memcpy(&f3Cfg.targetInfo[i + j].targetId, &dets[i + j].m_ID, 2);
            // f3Cfg.targetInfo[i + j].targetId = (uint16_t)dets[i + j].m_ID;
            f3Cfg.targetInfo[i + j].targetAzimuthCoordinate = dets[i + j].m_rrect.boundingRect().x;
            f3Cfg.targetInfo[i + j].targetPitchCoordinate = dets[i + j].m_rrect.boundingRect().y;
            f3Cfg.targetInfo[i + j].targetLength = dets[i + j].m_rrect.boundingRect().width;
            f3Cfg.targetInfo[i + j].targetWidth = dets[i + j].m_rrect.boundingRect().height;
            f3Cfg.targetInfo[i + j].targetDetectionConfidence = dets[i + j].m_confidence * 10000;

            f3Cfg.targetInfo[i + j].targetId = ntohs(f3Cfg.targetInfo[i + j].targetId);
            f3Cfg.targetInfo[i + j].targetAzimuthCoordinate = ntohs(f3Cfg.targetInfo[i + j].targetAzimuthCoordinate);
            f3Cfg.targetInfo[i + j].targetPitchCoordinate = ntohs(f3Cfg.targetInfo[i + j].targetPitchCoordinate);
            f3Cfg.targetInfo[i + j].targetLength = ntohs(f3Cfg.targetInfo[i + j].targetLength);
            f3Cfg.targetInfo[i + j].targetWidth = ntohs(f3Cfg.targetInfo[i + j].targetWidth);
            f3Cfg.targetInfo[i + j].targetDetectionConfidence = ntohs(f3Cfg.targetInfo[i + j].targetDetectionConfidence);

            sendBuf[pos++] = f3Cfg.targetInfo[i + j].targetType;
            memcpy(&sendBuf[pos], &f3Cfg.targetInfo[i + j].targetId, 2);
            pos += 2;
            memcpy(&sendBuf[pos], &f3Cfg.targetInfo[i + j].targetAzimuthCoordinate, 2);
            pos += 2;
            memcpy(&sendBuf[pos], &f3Cfg.targetInfo[i + j].targetPitchCoordinate, 2);
            pos += 2;
            memcpy(&sendBuf[pos], &f3Cfg.targetInfo[i + j].targetLength, 2);
            pos += 2;
            memcpy(&sendBuf[pos], &f3Cfg.targetInfo[i + j].targetWidth, 2);
            pos += 2;
            memcpy(&sendBuf[pos], &f3Cfg.targetInfo[i + j].targetDetectionConfidence, 2);
            pos += 2;
        }
        sendBuf[pos] = viewlink_protocal_checksum(sendBuf);
        sendBufLen = serialUp.serial_send(sendBuf, sendBufLen);
        printf("detector down send to up:%d\n", sendBufLen);
#if DEBUG_SERIAL
        for (auto x : dets)
        {
            printf("[%d] ", x.m_type);
            printf("[%zu] ", x.m_ID.ID2Module(1000));
            printf("[%d] ", x.m_rrect.boundingRect().x);
            printf("[%d] ", x.m_rrect.boundingRect().y);
            printf("[%d] ", x.m_rrect.boundingRect().width);
            printf("[%d] ", x.m_rrect.boundingRect().height);
            printf("[%f] ", x.m_confidence);
        }
        printf("\npos=%d\n", pos);
        for (int l = 0; l <= pos; l++)
        {
            printf("[%02x] ", sendBuf[l]);
        }
        printf("\n");
#endif
    }
    printf("DetectorResultFeedbackToUp end\n");
}

bool tempFlag = true;
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
    // memset(&sendBuf[5], 0, 11);
    memcpy(sendBuf + 5, buf, 9);
    // uint16_t azimuthPixel = pt.x;
    // uint16_t horiPixel = pt.y;
    // azimuthPixel = ntohs(azimuthPixel);
    // horiPixel = ntohs(horiPixel);
    // memcpy(&sendBuf[16], &azimuthPixel, 2);
    // memcpy(&sendBuf[18], &horiPixel, 2);

    stSysStatus.trackMissDistance[0] = (buf[0] << 8) ^ buf[1];
    stSysStatus.trackMissDistance[1] = (buf[2] << 8) ^ buf[3];
    // if (stSysStatus.trackMissDistance[0] == 0 && stSysStatus.trackMissDistance[1] == 0) {
    //     printf("Tracker MissDistance is All Zero\n");
    //     if (tempFlag) {
    //         sendBuf[6] = 0x01;
    //         sendBuf[8] = 0x01;
    //         tempFlag = false;
    //     } else {
    //         sendBuf[5] = 0xff;
    //         sendBuf[6] = 0xff;
    //         sendBuf[7] = 0xff;
    //         sendBuf[8] = 0xff;
    //         tempFlag = true;
    //     }

    //     sendBuf[9] |= 0x02;
    // }

    sendBuf[14] = viewlink_protocal_checksum(sendBuf);
    // printf("trackMissDistance x:%d, y:%d\n", stSysStatus.trackMissDistance[0], stSysStatus.trackMissDistance[1]);
    sendBufLen = serialDown.serial_send(sendBuf, sendBufLen);
    // printf("Tracker MissDistance Result FeedbackTodown, %d\n", sendBufLen);

    // for(int i=0;i<15;++i)
    // {
    //     printf("[%02X] ", sendBuf[i]);
    // }
    // printf("\n");
}

// 上位机拉流时首先板子向吊舱发送查询OSD设置信息，使自制OSD可以直接呈现到界面，否则刚拉的流不会有自制OSD呈现
static void QueryOSDSettingSendToDown()
{
    uint8_t sendBuf[1024] = {0};
    int sendBufLen = 8;
    sendBuf[0] = 0x55;
    sendBuf[1] = 0xAA;
    sendBuf[2] = 0xDC;
    sendBuf[3] = 0X5;
    sendBuf[4] = 0x01;
    sendBuf[5] = 0x85;
    sendBuf[6] = 0;
    sendBuf[7] = viewlink_protocal_checksum(sendBuf);

#if DEBUG_SERIAL
    printf("\nQueryOSDSettingSendToDown:\n");
    for (int i = 0; i < 8; i++)
    {
        printf("[%02X] ", sendBuf[i]);
    }
    printf("\n");
#endif
    sendBufLen = serialDown.serial_send(sendBuf, sendBufLen);
    printf("QueryOSDSettingSendToDown sendBufLen=%d\n", sendBufLen);
}

// 上位机拉流时首先板子向吊舱发送查询OSD设置信息，使自制OSD可以直接呈现到界面，否则刚拉的流不会有自制OSD呈现
static void QueryDeviceModelSendToDown()
{
    uint8_t sendBuf[1024] = {0};
    int sendBufLen = 8;
    sendBuf[0] = 0x55;
    sendBuf[1] = 0xAA;
    sendBuf[2] = 0xDC;
    sendBuf[3] = 0X05;
    sendBuf[4] = 0x01;
    sendBuf[5] = 0xE4;
    sendBuf[6] = 0;
    sendBuf[7] = viewlink_protocal_checksum(sendBuf);

#if DEBUG_SERIAL
    printf("\n QueryDeviceModelSendToDown:\n");
    for (int i = 0; i < 8; i++)
    {
        printf("[%02X] ", sendBuf[i]);
    }
    printf("\n");
#endif
    sendBufLen = serialDown.serial_send(sendBuf, sendBufLen);
    printf("QueryDeviceModelSendToDown sendBufLen=%d\n", sendBufLen);
}

bool isRecording = false;
;
cv::VideoWriter *writer = nullptr;

std::mutex m_mtx;
std::condition_variable conVar;
std::queue<cv::Mat> m_queue;

cv::Mat rcFrame;
void SaveRecordVideo()
{
    while (!quit)
    {
        std::unique_lock<std::mutex> lock(m_mtx);
        conVar.wait(lock);

        while (!m_queue.empty())
        {
            cv::Mat mat = m_queue.front().clone();
            if (writer != nullptr)
            {
                writer->write(mat);
                m_queue.pop();
            }
        }

        if (!isRecording)
        {
            writer->release();
            writer = nullptr;
        }
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

int main()
{
    spdlog::set_level(spdlog::level::debug);
    spdlog::stopwatch sw;

    // cv::Mat im = cv::imread("/home/nx/data/123.PNG");
    // cv::resize(im, im, cv::Size(1920,1080), cv::INTER_NEAREST);
    // cv::Mat dst;

    // int q = 0;
    // while(q++<100)
    // {
    //     sw.reset();
    //     cv::resize(im, dst, cv::Size(1280,720), cv::INTER_NEAREST);
    //     spdlog::debug("before cal aveg Elapsed {}", sw);
    // }

    // return 0;

    // unsigned int in = std::thread::hardware_concurrency();
    // std::cout << in << std::endl;
    // std::cout << cv::getBuildInformation() << std::endl;
    // return 0;

    // cpu_set_t mask;
    // /* 初始化set集，将set设置为空*/
    // CPU_ZERO(&mask);
    // /* 依次将0、1号cpu加入到集合*/
    // CPU_SET(5, &mask);
    // CPU_SET(0, &mask);
    // /*将当前进程绑定到cpu */
    // if (sched_setaffinity(0, sizeof(mask), &mask) == -1) {
    //     printf("Set CPU affinity failue, ERROR:%s\n", strerror(errno));
    //     return -1;
    // }

    cv::VideoWriter rtspWriterr;
    // rtspWriterr.open("appsrc ! videoconvert ! video/x-raw,format=I420 ! x264enc speed-preset=ultrafast bitrate=2048 key-int-max=" + std::to_string(15 * 2) + " ! video/x-h264,profile=baseline ! rtspclientsink location=rtsp://localhost:8553/stream", cv::CAP_GSTREAMER, 0, 15, cv::Size(1280, 720), true);
    // rtspWriterr.open("appsrc ! videoconvert ! video/x-raw,format=I420 ! omxh264enc bitrate=8000000 control-rate=2 ! video/x-h264,profile=main ! rtspclientsink location=rtsp://localhost:8553/stream", cv::CAP_GSTREAMER, 0, 30, cv::Size(1280, 720), true);
    // rtspWriterr.open("appsrc ! videoconvert ! video/x-raw,format=I420 ! omxh264enc bitrate=8000000 control-rate=2 ! video/x-h264,profile=baseline ! rtspclientsink location=rtsp://localhost:8553/stream", cv::CAP_GSTREAMER, 0, 30, cv::Size(1280, 720), true);
    rtspWriterr.open("appsrc ! videoconvert ! video/x-raw,format=I420 ! omxh264enc bitrate=4000000 control-rate=2 ! video/x-h264 ! rtspclientsink location=rtsp://localhost:8553/stream", cv::CAP_GSTREAMER, 0, 30, cv::Size(1280, 720), true);
    // rtspWriterr.open("appsrc ! nvvidconv ! video/x-raw(memory:NVMM),format=I420 ! nvv4l2h264enc bitrate=6000000 ! video/x-h264,profile=baseline ! rtspclientsink location=rtsp://localhost:8553/stream", cv::CAP_GSTREAMER, 0, 30, cv::Size(1280, 720), true);

    if (!rtspWriterr.isOpened())
    {
        printf("is not opened\n");
        return 0;
    }

    std::deque<double> fpsCalculater;

    struct sigaction sig_action;
    sig_action.sa_handler = signal_handle;
    sigemptyset(&sig_action.sa_mask);
    sig_action.sa_flags = 0;
    sigaction(SIGINT, &sig_action, NULL);

    //*******************************serial init*************************
    serialUp.set_serial(1);   //"/dev/ttyTHS1"
    serialDown.set_serial(2); //"/dev/ttyUSB0"

    std::thread serialThUp2Down = std::thread(SerialTransUp2Down);
    serialThUp2Down.detach();
    std::thread serialThDown2Up = std::thread(SerialTransDown2Up);
    serialThDown2Up.detach();
    //*******************************serial end*************************

    std::string cfgpath = "/etc/jetsoncfg/pl/config.yaml";
    YAML::Node config = YAML::LoadFile(cfgpath);

    //*******************************tcp socket*************************
    /* 与上位机tcp客户端连接，作为服务器端接收上位机发的tcp消息 */
    /* 作为客户端与吊舱服务器端建立连接，首先创建客户端套接字并绑定 */
    clientSocketfd = socket(AF_INET, SOCK_STREAM, 0);

    const std::string downToNXIp = config["downToNXIp"].as<std::string>();
    uint16_t downToNXPort = config["downToNXPort"].as<uint16_t>();
    printf("downToNXIp:%s downToNXPort:%d\n", downToNXIp.c_str(), downToNXPort);
    struct sockaddr_in clientaddr;
    clientaddr.sin_family = AF_INET;
    clientaddr.sin_port = htons(downToNXPort); // 目标端口和IP

    inet_pton(AF_INET, downToNXIp.c_str(), &clientaddr.sin_addr.s_addr);
    bind(clientSocketfd, (struct sockaddr *)&clientaddr, sizeof(clientaddr));

    // 与吊舱服务器端建立连接
    struct sockaddr_in serveraddrPod;

    const std::string podIp = config["podIp"].as<std::string>();
    uint16_t podPort = config["podPort"].as<uint16_t>();
    printf("podIp:%s podPort:%d\n", podIp.c_str(), podPort);
    serveraddrPod.sin_family = AF_INET;
    serveraddrPod.sin_port = htons(podPort); // 为当前进程添加的端口号为2000

    inet_pton(AF_INET, podIp.c_str(), &serveraddrPod.sin_addr.s_addr);

    int connectTime = 0;
    // while(1) {
    //     if(connect(clientSocketfd, (struct sockaddr *)&serveraddrPod, sizeof(serveraddrPod)) < 0) {
    //         printf("TCPTrans connect error\n");
    //         connectTime++;
    //         usleep(5000000); // 5s
    //         if (connectTime > 4) {
    //             printf("TCPTrans connect timeout\n");
    //             break;
    //         }
    //     } else {
    //         printf("TCPTrans connect success\n");
    //         break;
    //     }
    // }

    // std::thread tcpUp2DownTh = std::thread(TCPTransUp2Down);
    // tcpUp2DownTh.detach();
    // std::thread tcpDown2UpTh = std::thread(TCPTransDown2Up);
    // tcpDown2UpTh.detach();
    //*******************************tcp end*************************
    std::string engine = config["engine"].as<std::string>();
    realtracker *rtracker = new realtracker(engine);

    cv::Mat frame;
    int nFrames = 0;

    cv::Mat dispFrame, trackFrame, detFrame;
    cv::Mat oriIrImg, viImg, irImg;
    std::vector<TrackingObject> detRet;
    // std::vector<bbox_t> detRet;

    stSysStatus.enDispMode = Vision;
    stSysStatus.trackOn = false;
    stSysStatus.detOn = false;

    int pipPosX, pipPosY;

    Camera *cam = CreateCamera(cfgpath);

    if (cam != nullptr)
    {
        cam->Init();
        // QueryDeviceModelSendToDown();
        // QueryOSDSettingSendToDown();
        printf("camera init success\n");
    }
    else
    {
        printf("camera init failed\n");
        return 0;
    }

    cam->GetFrame(viImg, oriIrImg);
    if (oriIrImg.empty() || viImg.empty())
    {
        printf("input img empty, quit\n");
        return 0;
    }

#if DEBUG_SERIAL
    rtracker->runDetector(viImg, detRet);
#else
    rtracker->runDetectorNoDraw(viImg, detRet);
#endif

    int irImgW = viImg.cols;
    int irImgH = viImg.rows;

    irImg = cv::Mat(irImgH, irImgW, CV_8UC3);
    irImg.setTo(0);

    int oriImgW = oriIrImg.cols;
    int oriImgH = oriIrImg.rows;

    int viImgW = viImg.cols;
    int viImgH = viImg.rows;

    pipPosX = (viImg.cols - oriIrImg.cols) / 2;
    pipPosY = (viImg.rows - oriIrImg.rows) / 2;

    rtracker->setFrameScale((double)viImgW / 1920);

    uint8_t trackerStatus[9];
    memset(trackerStatus, 0, 9);

    std::thread saveVideTh(&SaveRecordVideo);
    saveVideTh.detach();

    // 等待上次录制完成，需要进行录制标志
    bool isNeedRecording = false;

    while (!quit)
    {
        // usleep(1000000);
        // continue;
        spdlog::debug("=====nframe:{}======", nFrames);
        cam->GetFrame(viImg, oriIrImg);

        if (oriIrImg.empty() || viImg.empty())
        {
            printf("input img empty, quit\n");
        }

        sw.reset();

        cvtIrImg(oriIrImg, stSysStatus.enIrImgMode);

        // printf("oriIrImg w:%d, oriIrImg h:%d\n", oriIrImg.cols, oriIrImg.rows);
        // printf("viImg w:%d, viImg h:%d\n", viImg.cols, viImg.rows);

        irImg.setTo(0);
        oriIrImg.copyTo(irImg(cv::Rect(pipPosX, pipPosY, oriIrImg.cols, oriIrImg.rows)));

        // printf("irImg w:%d, irImg h:%d\n", irImg.cols, irImg.rows);
        // printf("viImg w:%d, viImg h:%d\n", viImg.cols, viImg.rows);

        switch (stSysStatus.enDispMode)
        {
        case Vision: // 0x01
            frame = viImg;
            rtracker->setIrFrame(false);
            break;
        case Ir: // 0x02
            frame = irImg;
            trackerStatus[4] |= 0x01; // 0000 0001
            rtracker->setIrFrame(true);
            break;
        case VisIrPip: // 0x03
            cv::resize(oriIrImg, oriIrImg, cv::Size(480, 360));
            oriIrImg.copyTo(viImg(cv::Rect(viImgW - 480, 0, 480, 360)));
            frame = viImg;
            rtracker->setIrFrame(false);
            break;
        case IrVisPip: // 0x04
            cv::resize(viImg, viImg, cv::Size(480, 360));
            viImg.copyTo(irImg(cv::Rect(irImgW - 480, 0, 480, 360)));
            frame = irImg;
            // frame = viImg;
            rtracker->setIrFrame(true);
            break;
        default:
            frame = viImg;
            break;
        }

        rcFrame = frame.clone();

        // rtspWriterr << frame;
        // continue;

        // frame = cv::imread("/home/nx/data/123.PNG");
        // stSysStatus.detOn = true;

        // spdlog::debug("after cap img Elapsed {}", sw);

        // trackFrame = frame.clone();
        // detFrame = frame.clone();
        // dispFrame = frame.clone();

        bool isNeedTakePhoto = false;
        if (stSysStatus.trackOn)
        {
            if (!stSysStatus.trackerInited)
            {
                spdlog::debug("start tracking, init Rect:");
                rtracker->reset();
                rtracker->setGateSize(stSysStatus.trackerGateSize);
                rtracker->init(stSysStatus.trackAssignPoint, frame);
                stSysStatus.trackerInited = true;
            }
            else
            {
                rtracker->update(frame, detRet, trackerStatus);
                spdlog::debug("tracker status:{}", trackerStatus[4]);
                TrackerMissDistanceResultFeedbackToDown(trackerStatus);
            }
        }
        else if (stSysStatus.detOn)
        {

            rtracker->runDetector(frame, detRet);

            // if(stSysStatus.detRetOutput)
            //     DetectorResultFeedbackToUp(detRet);
        }

        // 在界面上绘制OSD
        stSysStatus.osdSet1Ctrl.enOSDShow = true;
        if (stSysStatus.osdSet1Ctrl.enOSDShow)
        {
            stSysStatus.osdSet1Ctrl.enAttitudeAngleShow = true;
            if (stSysStatus.osdSet1Ctrl.enAttitudeAngleShow)
            {
                // 绘制吊舱当前方位角度滚轴
                PaintRollAngleAxis(frame, stSysStatus.rollAngle);

                // 绘制吊舱当前俯仰角度滚轴
                PaintPitchAngleAxis(frame, stSysStatus.pitchAngle);
            }
            stSysStatus.osdSet1Ctrl.enCrossShow = true;
            if (stSysStatus.osdSet1Ctrl.enCrossShow)
            {
                // 绘制中心十字
                PaintCrossPattern(frame, stSysStatus.rollAngle, stSysStatus.pitchAngle);
            }

            // 绘制经纬度、海拔高度等坐标参数
            PaintCoordinate(frame);

            // 绘制界面上其他参数
            PaintViewPara(frame);

            // 绘制脱靶量
            PaintTrackerMissDistance(frame);
        }

        if (stSysStatus.enScreenOpMode == EN_SCREEN_OP_MODE::SCREEN_SHOOT)
        {
            stSysStatus.enScreenOpMode = EN_SCREEN_OP_MODE::SCREEN_NONE;
            isNeedTakePhoto = true;
        }
        else if ((stSysStatus.enScreenOpMode == EN_SCREEN_OP_MODE::RECORDING_START && !isRecording) || isNeedRecording)
        {
            stSysStatus.enScreenOpMode = EN_SCREEN_OP_MODE::SCREEN_NONE;
            if (!m_queue.empty())
            {
                printf("wait recording video\n");
                isNeedRecording = true;
            }
            else
            {
                isNeedRecording = false;
                printf("start recording video\n");
                std::string currTimeStr = CreateDirAndReturnCurrTimeStr("/home/nx/videos_recorded");
                std::string saveVideoFileName = "/home/nx/videos_recorded/" + currTimeStr + ".mp4";
                if (writer == nullptr)
                {
                    writer = new cv::VideoWriter();
                }
                writer->open(saveVideoFileName, cv::VideoWriter::fourcc('M', 'P', '4', 'V'),
                             25,
                             cv::Size(1280, 720),
                             true);
                if (writer->isOpened())
                {
                    printf("writer open success\n");
                    isRecording = true;
                }
                else
                {
                    printf("writer open failed\n");
                }
            }
        }
        else if (stSysStatus.enScreenOpMode == EN_SCREEN_OP_MODE::RECORDING_END && isRecording)
        {
            stSysStatus.enScreenOpMode = EN_SCREEN_OP_MODE::SCREEN_NONE;
            printf("recording end\n");
            isRecording = false;
            conVar.notify_all();
        }

        // spdlog::debug("before resize Elapsed {}", sw);

        cv::resize(frame, dispFrame, cv::Size(1280, 720), cv::INTER_NEAREST);
        // cv::resize(frame, dispFrame, cv::Size(1920,1080), cv::INTER_NEAREST);

        if (isRecording)
        {
            cv::resize(rcFrame, rcFrame, cv::Size(1280, 720), cv::INTER_NEAREST);
            // *writer << rcFrame;
            // *writer << dispFrame;
            m_queue.push(rcFrame);
            conVar.notify_all();
        }

        if (isNeedTakePhoto)
        {
            std::string currTimeStr = CreateDirAndReturnCurrTimeStr("photos");
            std::string savePicFileName = "photos/" + currTimeStr + ".png";
            cv::imwrite(savePicFileName, frame);
            isNeedTakePhoto = false;
        }

        nFrames++;
        // spdlog::debug("before rtsp Elapsed {}", sw);
        // encoder->process(dispFrame);
        rtspWriterr << dispFrame;

        // cv::imshow("det", detret);
        // cv::imwrite("1.png", frame);
        // cv::imshow("show", dispFrame);
        // cv::waitKey(30);
        // usleep(25000);

        // spdlog::debug("before cal aveg Elapsed {}", sw);

        // std::chrono::duration<double> dd =  sw.elapsed();

        fpsCalculater.emplace_back(sw.elapsed().count());
        if (fpsCalculater.size() > 30)
            fpsCalculater.pop_front();
        double meanValue = accumulate(begin(fpsCalculater), end(fpsCalculater), 0.0) / fpsCalculater.size(); // 求均值

        spdlog::debug("one frame Elapsed {}", meanValue);
    }
    writer->release();
    return 0;
}