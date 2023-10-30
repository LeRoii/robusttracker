#include "multitracker.h"
#include <unistd.h>
#include <signal.h>
#include "jetsonEncoder.h"
#include "serial.h"
#include "common.h"
#include <sstream>
#include <cmath>
#include "camera.h"
#include "painter.h"
#include<arpa/inet.h>
#include <unistd.h>
#include <sys/vfs.h>
#include <yaml-cpp/yaml.h>
#include "realtracker.h"
#include "spdlog/spdlog.h"
#include "spdlog/stopwatch.h"
#include <deque>
#include <numeric>

static std::vector<std::vector<uint8_t>> CmdNeedProcess = 
{
    {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x81, 0x00, 0x00, 0x00, 0xAC},//EO(热像白热)
    {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x83, 0x00, 0x00, 0x00, 0xAE},//EO+IR(白热)
    {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x84, 0x00, 0x00, 0x00, 0xA9},//IR(白热)+EO
    {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x82, 0x00, 0x00, 0x00, 0xAF},//IR(白热)

    {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xC1, 0x00, 0x00, 0x00, 0xEC},//EO(热像黑热)
    {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xC3, 0x00, 0x00, 0x00, 0xEE},//EO+IR(黑热)
    {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xC4, 0x00, 0x00, 0x00, 0xE9},//IR(黑热)+EO
    {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xC2, 0x00, 0x00, 0x00, 0xEF},//IR(黑热)

    {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x81, 0x00, 0x00, 0x00, 0x7C},//EO(热像红热)
    {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x83, 0x00, 0x00, 0x00, 0x7E},//EO+IR(红热)
    {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x84, 0x00, 0x00, 0x00, 0x79},//IR(红热)+EO
    {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x82, 0x00, 0x00, 0x00, 0x7F},//IR(红热)

    {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0xD0, 0x00, 0x00, 0x00, 0xF8},//热像放大间隔1倍
    {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x10, 0x00, 0x00, 0x00, 0x39},//热像缩小间隔1倍

    {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0xD0, 0x00, 0x00, 0x00, 0xFA},//拍照
    {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x10, 0x00, 0x00, 0x00, 0x3B},//录像开始
    {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x50, 0x00, 0x00, 0x00, 0x7B},//录像结束

    {0x55, 0xAA, 0xDC, 0x05, 0x01, 0x05, 0x00, 0x01},//打开OSD
    {0x55, 0xAA, 0xDC, 0x05, 0x01, 0x05, 0x01, 0x00},//关闭OSD

    {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x24},//开始跟踪
    {0x55, 0xAA, 0xDC, 0x06, 0x1E, 0x00, 0x01, 0x00, 0x19}, //结束跟踪

    {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x01, 0x2A},//打开识别
    {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x2B},//关闭识别

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
    if(buf[4] == 0x5D)
        return true;

    switch(buf[4])
    {
        case 0x30:
        case 0x01:
        case 0x1E:
            for(auto &cmd:CmdNeedProcess)
            {
                int find = true;
                for(int i=0;i<len;++i)
                {
                    if(buf[i] != cmd[i])
                    {
                        find = false;
                        break;
                    }
                }
                if(find)
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

    if(len == 8 || len == 9 || len == 20)
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

    if (buf[4] == 0x5D) {
        struct statfs diskInfo;
        statfs("/", &diskInfo);
        unsigned long long totalBlocks = diskInfo.f_bsize;  
        unsigned long long totalSize = totalBlocks * diskInfo.f_blocks;  
        size_t mbTotalsize = totalSize>>20;  
        unsigned long long freeDisk = diskInfo.f_bavail*totalBlocks;  
        size_t mbFreedisk = freeDisk>>20;  
        printf ("/  total=%ldMB, free=%ldMB\n", mbTotalsize, mbFreedisk);

        sendBufLen = 10;
        
        sendBuf[3] = 0x5 + 0x3;
        sendBuf[4] = 0xD5;

        if (buf[5] == 0x8B) {
            sendBuf[5] = buf[6] - 1;
            if (buf[6] == 0x2) {
                if (mbTotalsize > 0 && mbFreedisk > 100) {
                    sendBuf[6] = 0x1;
                } else if (mbTotalsize == 0 || mbFreedisk == 0) {
                    sendBuf[6] = 0x80;
                } else {
                    sendBuf[6] = 0x20;
                }
            } else if (buf[6] == 0x3) {
                sendBuf[6] = mbTotalsize;
            }  else if (buf[6] == 0x4) {
                sendBuf[6] = mbFreedisk;
            }
            // todo: 拍照张数和剩余录像时间待计算截图与录像视频格式再补充
        }
    }
    uint8_t checksum = viewlink_protocal_checksum(sendBuf);
    sendBuf[sendBufLen - 1] = checksum;
    sendBufLen = serialUp.serial_send(sendBuf, sendBufLen);
    printf("down send to up:%d\n", sendBufLen);
}

void SerialTransUp2Down()
{
    uint8_t output[1024] = {0};
    int outLen = 0;
    uint8_t buffRcvData_servo[1024] = {0};
    while(1)
    {
        int retLen = serialUp.serial_recieve(buffRcvData_servo);
        if(retLen > 0)
        {
            std::cout<<"=======>serialUp received "<<std::dec<<retLen<<"bytes"<<std::endl;
#if DEBUG_SERIAL
            for(int i=0; i< retLen ;i++)
            {
                printf("[%02X]", buffRcvData_servo[i]);
            }
            printf("\n");
#endif
            // if (IsTransparentToPod(buffRcvData_servo)) {
            //     retLen = serialDown.serial_send(buffRcvData_servo, retLen);
            // }
            
            // printf("up send to down:%d\n", retLen);

            int rr = serialUp.ProcessSerialData(buffRcvData_servo, retLen, output, outLen);
            if(rr == RET_ERR)
            {
                printf("RET_ERR\n");
                continue;
            }

            EN_DATA_FRAME_TYPE frameType = GetFrameType(output, outLen);
            printf("frame type:%d\n", frameType);
            if(frameType == HeartBeat15)
            {
                printf("heart beat 15 from up serial\n\n");
            }
            else if(frameType == HandShake)
            {
                printf("handshake from up serial\n\n");
            }
            else if(frameType == FrameS2)
            {
                printf("TGCC Ctrl S2 from up serial\n\n");
            }
            else if(frameType == HeartBeat14)
            {
                printf("heart beat 14 from up serial\n\n");
            }

            if(IsTransparentToPod(output, outLen))
            {
                retLen = serialDown.serial_send(output, outLen);
                printf("up send to down:%d\n", retLen);
                continue;
            }

            for(int i=0; i< outLen ;i++)
            {
                printf("[%02X]", output[i]);
            }
            printf("\n");

            VL_ParseSerialData(output);
            OnceSendFromDownToUp(output);

            printf("status->enDispMode:%d, detOn:%d, trackOn:%d, trackerGateSize:%d\n",\
             stSysStatus.enDispMode, stSysStatus.detOn, stSysStatus.trackOn, stSysStatus.trackerGateSize);
            printf("\n\n");

            continue;

            // printf("output buf\n");
            // for(int i=0; i< outLen ;i++)
            // {
            // 	printf("[%02X]", output[i]);
            // }
            // printf("\n");

            //check frame header
            // if(!CheckFrameHeader(buffRcvData_servo, retLen))
            // {
            // 	for(int i=0; i< retLen ;i++)
            // 	{
            // 		printf("[%02X]", buffRcvData_servo[i]);
            // 	}
            // 	printf("\n");
            // 	printf("frame header error, drop data\n\n");
            // 	memset(buffRcvData_servo,0,1024);
            // 	continue;
            // }

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

            //checksum
            // uint8_t checksum = viewlink_protocal_checksum(buffRcvData_servo);
            // if(checksum != buffRcvData_servo[retLen - 1])
            // {
            //     printf("frame checksum error, drop data\n\n");
            //     memset(buffRcvData_servo,0,1024);
            //     continue;
            // }

            // VL_ParseSerialData(output);
            // if (!IsTransparentToPod(buffRcvData_servo)) {
            //     OnceSendFromDownToUp(output);
            // }

            // printf("status->enDispMode:%d, detOn:%d, trackOn:%d, trackerGateSize:%d\n",\
            //  stSysStatus.enDispMode, stSysStatus.detOn, stSysStatus.trackOn, stSysStatus.trackerGateSize);
            // printf("\n\n");
        }
    }
}

void SerialTransDown2Up()
{
    uint8_t buffRcvData_servo[1024] = {0};
    uint8_t output[1024] = {0};
    int outLen = 0;
    while(1)
    {    
        int retLen = serialDown.serial_recieve(buffRcvData_servo);
        if(retLen > 0)
        {
            std::cout<<"<===========serial down received "<<std::dec<<retLen<<"bytes"<<std::endl;
#if DEBUG_SERIAL
            for(int i=0; i< retLen ;i++)
            {
                printf("[%02X]", buffRcvData_servo[i]);
            }
            printf("\n");
#endif

            retLen = serialUp.serial_send(buffRcvData_servo, retLen);
            printf("down send to up:%d\n", retLen);

            int rr = serialDown.ProcessSerialData(buffRcvData_servo, retLen, output, outLen);

            if(rr == RET_ERR)
            {
                printf("RET_ERR\n");
                continue;
            }

#if DEBUG_SERIAL
            printf("output buf\n");
            for(int i=0; i< outLen ;i++)
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
            
            if(frameType == Status41 || frameType == Status40)
            {
                printf("Status from down serial frameType:%d\n\n", frameType);
                continue;
            }

            printf("!!!non Status frametype\n");
            for(int i=0; i< outLen ;i++)
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
        }
        
    }
}

static void cvtIrImg(cv::Mat &img, EN_IRIMG_MODE mode)
{
    
    if(mode == EN_IRIMG_MODE::BLACKHOT)
    {
        cv::cvtColor(img, img, cv::COLOR_RGB2GRAY);
        img = 255 - img;
        cv::cvtColor(img, img, cv::COLOR_GRAY2RGB);
    }
    else if(mode == EN_IRIMG_MODE::PSEUDOCOLOR)
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
    if ((f3Cfg.targetSum % 4) > 0) {
        f3Cfg.totalPacketNum += 1;
    }

    for (int i = 0; i < f3Cfg.totalPacketNum; i++) {
        f3Cfg.currPacketId = i;
        int currPacketTargetNum = 4;
        if (f3Cfg.totalPacketNum - i == 1) {
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
        for (int j = 0; j < currPacketTargetNum; j++) {
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
        for (auto x : dets) {
            printf("[%d] ", x.m_type);
            printf("[%zu] ", x.m_ID.ID2Module(1000));
            printf("[%d] ", x.m_rrect.boundingRect().x);
            printf("[%d] ", x.m_rrect.boundingRect().y);
            printf("[%d] ", x.m_rrect.boundingRect().width);
            printf("[%d] ", x.m_rrect.boundingRect().height);
            printf("[%f] ", x.m_confidence);
        }
        printf("\npos=%d\n", pos);
        for (int l = 0; l <= pos; l++) {
            printf("[%02x] ", sendBuf[l]);
        }
        printf("\n");
#endif
    }
    printf("DetectorResultFeedbackToUp end\n");
}

bool isRecording = false;;
cv::VideoWriter writer;

cv::Mat tempFrame;

void SaveRecordVideo()
{
    printf("================================SaveRecordVideo======================\n");
    writer.write(tempFrame);
    tempFrame.release();
}

static std::string CreateDirAndReturnCurrTimeStr(std::string folderName)
{
    std::string cmd = "mkdir " + folderName;
    int ret = system(cmd.c_str());
    printf("create %s result is %s\n", folderName.c_str(), (ret == 0) ? "success" : "failed");
    std::time_t curr = std::chrono::system_clock::to_time_t (std::chrono::system_clock::now());
    std::stringstream ss;
    ss << std::put_time(std::localtime(&curr), "%Y-%m-%d-%H-%M-%S");
    std::string currTime(ss.str());
    return currTime;
}

int main()
{
    spdlog::set_level(spdlog::level::debug);
    spdlog::stopwatch sw;
    std::deque<double> fpsCalculater;

    struct sigaction sig_action;
    sig_action.sa_handler = signal_handle;
    sigemptyset(&sig_action.sa_mask);
    sig_action.sa_flags = 0;
    sigaction(SIGINT, &sig_action, NULL);

//*******************************serial init*************************
    
    serialUp.set_serial(1);    //"/dev/ttyTHS1"
    serialDown.set_serial(2);    //"/dev/ttyUSB0"

    std::thread serialThUp2Down = std::thread(SerialTransUp2Down);
    serialThUp2Down.detach();
    std::thread serialThDown2Up = std::thread(SerialTransDown2Up);
    serialThDown2Up.detach();
//*******************************serial end*************************

    jetsonEncoder *encoder = new jetsonEncoder(8554);

    YAML::Node config = YAML::LoadFile("../config.yaml");
	std::string engine = config["engine"].as<std::string>();
    realtracker *rtracker = new realtracker(engine);

    cv::Mat frame;
    int nFrames = 0;

    cv::Mat dispFrame, trackFrame, detFrame;
	cv::Mat oriIrImg, viImg, irImg;
    std::vector<TrackingObject> detRet;

    int irImgW = 1920;
    int irImgH = 1080;

    irImg = cv::Mat(irImgH, irImgW, CV_8UC3);
    irImg.setTo(0);

    stSysStatus.enDispMode = Vision;
    stSysStatus.trackOn = false;
    stSysStatus.detOn = false;

    int pipPosX, pipPosY;

    Camera *cam = CreateCamera("../config.yaml");

    if(cam != nullptr) {
        cam->Init();
    } else {
        printf("camera inti failed\n");
        return 0;
    }

    cam->GetFrame(viImg, oriIrImg);
    if(oriIrImg.empty() || viImg.empty())
    {
        printf("input img empty, quit\n");
        return 0;
    }

    rtracker->runDetector(viImg, detRet);

    int oriImgW = oriIrImg.cols;
    int oriImgH = oriIrImg.rows;

    int viImgW = viImg.cols;
    int viImgH = viImg.rows;

    pipPosX = (viImg.cols - oriIrImg.cols)/2;
    pipPosY = (viImg.rows - oriIrImg.rows)/2;

    while(!quit)
    {
        // usleep(1000000);
        // continue;
        spdlog::debug("=====nframe:{}======", nFrames);
        cam->GetFrame(viImg, oriIrImg);

        if(oriIrImg.empty() || viImg.empty())
        {
            printf("input img empty, quit\n");
        }

        sw.reset();

        cvtIrImg(oriIrImg, stSysStatus.enIrImgMode);

        // printf("oriIrImg w:%d, oriIrImg h:%d\n", oriIrImg.cols, oriIrImg.rows);
        // printf("viImg w:%d, viImg h:%d\n", viImg.cols, viImg.rows);

        // irImg.setTo(0);
        // oriIrImg.copyTo(irImg(cv::Rect(pipPosX, pipPosY, oriIrImg.cols, oriIrImg.rows)));

        // printf("irImg w:%d, irImg h:%d\n", irImg.cols, irImg.rows);
        // printf("viImg w:%d, viImg h:%d\n", viImg.cols, viImg.rows);

		switch(stSysStatus.enDispMode)
		{
			case Vision:    //0x01
				frame = viImg;
				break;
			case Ir:        //0x02
				frame = irImg;
				break;
			case VisIrPip:  //0x03
				cv::resize(oriIrImg, oriIrImg, cv::Size(480, 360));
				oriIrImg.copyTo(viImg(cv::Rect(viImgW-480, 0, 480, 360)));
				frame = viImg;
				break;
			case IrVisPip:  //0x04
				cv::resize(viImg, viImg, cv::Size(480, 360));
				viImg.copyTo(irImg(cv::Rect(irImgW-480, 0, 480, 360)));
				frame = irImg;
				break;
			default:
				frame = viImg;
				break;
		}

        // frame = cv::imread("/home/nx/data/123.PNG");
        // stSysStatus.detOn = true;

        // spdlog::debug("after cap img Elapsed {}", sw);

        // trackFrame = frame.clone();
        // detFrame = frame.clone();
        // dispFrame = frame.clone();

        // 在界面上绘制OSD
        //float currRollAngle = -78.5; // 需要修改为吊舱返回的当前横滚角度
        PaintRollAngleAxis(frame, stSysStatus.rollAngle);

        //float currPitchAngle = 0; // 需要修改为吊舱返回的当前俯仰角度
        PaintPitchAngleAxis(frame, stSysStatus.pitchAngle);

        // 绘制中心十字
        PaintCrossPattern(frame, stSysStatus.rollAngle, stSysStatus.pitchAngle);

        // 绘制经纬度、海拔高度等坐标参数
        PaintCoordinate(frame);

        // 绘制界面上其他参数
        PaintViewPara(frame);

        if (stSysStatus.trackOn) {
            cv::Rect initRect = cv::Rect{(frame.cols-stSysStatus.trackerGateSize)/2, (frame.rows-stSysStatus.trackerGateSize)/2, stSysStatus.trackerGateSize, stSysStatus.trackerGateSize};
            if (!stSysStatus.trackerInited) {
                spdlog::debug("start tracking, init Rect:");
                std::cout<<initRect<<std::endl;
                rtracker->reset();
                rtracker->init(initRect, frame );
        		stSysStatus.trackerInited = true;
            } else {
                cv::Point pt;
                rtracker->update(trackFrame, detRet, pt);
                // cv::imshow("trackRet", trackFrame);
            }
        } else if (stSysStatus.detOn) {
            rtracker->runDetector(frame, detRet);
            if(stSysStatus.detRetOutput)
                DetectorResultFeedbackToUp(detRet);
        } else if (stSysStatus.enScreenOpMode == EN_SCREEN_OP_MODE::SCREEN_SHOOT) {
            stSysStatus.enScreenOpMode = EN_SCREEN_OP_MODE::SCREEN_NONE;
            std::string currTimeStr = CreateDirAndReturnCurrTimeStr("photos");
            std::string savePicFileName = "photos/" + currTimeStr + ".png";
            //cv::imwrite(savePicFileName, frame);
        } else if (stSysStatus.enScreenOpMode == EN_SCREEN_OP_MODE::RECORDING_START && !isRecording) {
            stSysStatus.enScreenOpMode = EN_SCREEN_OP_MODE::SCREEN_NONE;
            std::string currTimeStr = CreateDirAndReturnCurrTimeStr("videos_recorded");
            std::string saveVideoFileName = "videos_recorded/" + currTimeStr + ".mp4";
            writer.open(saveVideoFileName, cv::VideoWriter::fourcc('M', 'P', '4', 'V'),
                20,
                cv::Size(1280,720),
                true);
            if (writer.isOpened()) {
                printf("writer open success\n");
                isRecording = true;
            } else {
                printf("writer open failed\n");
            }
        } else if (stSysStatus.enScreenOpMode == EN_SCREEN_OP_MODE::RECORDING_END && isRecording) {
            stSysStatus.enScreenOpMode = EN_SCREEN_OP_MODE::SCREEN_NONE;
            printf("writer end\n");
            isRecording = false;
        }

        cv::resize(frame, dispFrame, cv::Size(1280,720));

        if (isRecording) {
            tempFrame = dispFrame.clone();
            std::thread saveVideoThread(SaveRecordVideo);
            saveVideoThread.detach();
        } else {
            writer.release();
        }

        nFrames++;

        //cv::imshow("show", dispFrame);
        encoder->process(dispFrame);

        // cv::imshow("det", detret);
        // cv::imwrite("1.png", frame);
        //cv::waitKey(30);
        // usleep(30000);

        // spdlog::debug("before cal aveg Elapsed {}", sw);

        // std::chrono::duration<double> dd =  sw.elapsed();

        fpsCalculater.emplace_back(sw.elapsed().count());
        if(fpsCalculater.size() > 30)
            fpsCalculater.pop_front();
        double meanValue = accumulate(begin(fpsCalculater), end(fpsCalculater), 0.0) / fpsCalculater.size();                   // 求均值

        spdlog::debug("one frame Elapsed {}", meanValue);


    }
    writer.release();
    return 0;
}