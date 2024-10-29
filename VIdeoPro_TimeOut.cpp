
/****************************************************************************************
 * 文 件 名 : Video_TimeOut.cpp
 * 项目名称 : 205s
 * 模 块 名 :
 * 功    能 :
 * 操作系统 : LINUX
 * 修改记录 : 无
 * 版    本 : Rev 0.1.0
 *- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 * 设    计 : walekr      '2024-06-27
 * 编    码 : walekr      '2024-06-27
 * 修    改 :
 ****************************************************************************************/
#include <iostream>
#include <sstream>
#include <cmath>
#include <arpa/inet.h>
#include <chrono>
#include <sys/vfs.h>
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
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include "common.h"
#include "serialport.h"
#include "VideoPro_MsgTable.h"
#include "VIdeoPro_TimeOut.h"

extern uint8_t trackerStatus[9];
extern ST_TriaxialAngle stTriaxialAngle ;
extern ST_TriaxialAngle_Speed stTriaxialAngleSpeed;
extern ST_XYZ_AcceleratedSpeed stXYZAcceleratedSpeed;
extern Serial serial137Link; //137通信串口
extern ST_SYS_STATUS stSysStatus;
extern ST_AttitudeAngle stAttitudeAngle;

extern unsigned long long lCurTime;            /*****当前时间****/
extern uint8_t  WorkModeFlag;
// TCP客户端连接句柄
extern int client_sockfd_tcp ;

/****************************************************************************************
 * 函 数 名 ： project205_VideoTrack
 * 功    能 ： 解析方位角度 俯仰  横滚角度
 * 输入参数 ： 输入结构体
 * 输出参数 ： 无
 * 返 回 值 ： 0: 成功；-1: 失败
 ***************************************************************************************/
void VideoPro_VideoTrack()
{
    uint8_t SendBuff[10]={0};
    ST_ACK_SIFU_CONFIG  *stAckSiFuConfig;
    stAckSiFuConfig=(ST_ACK_SIFU_CONFIG*)SendBuff;
    stAckSiFuConfig->head=HEAD;
    ST_MissDistanceReport stMissDistanceReport;
    stMissDistanceReport.head.head1=HEAD1;
    stMissDistanceReport.head.head2=HEAD2;
    stMissDistanceReport.head.head3=HEAD3;
    stMissDistanceReport.head.lLen =sizeof(ST_MissDistanceReport)-sizeof(xj_net_msg_hdr)-1;
    stMissDistanceReport.msg_type=XJ_TRACK;



    if( stSysStatus.trackOn != true)
    {
        goto OUT;
    }
    if(trackerStatus[4] &0x2)
    {
        stAckSiFuConfig->type=WORKMODE_VIDEOTRACKER;
        stAckSiFuConfig->uMissDistanceX=stSysStatus.trackMissDistance[0];
        stAckSiFuConfig->uMissDistanceY=stSysStatus.trackMissDistance[1];
        std::cout<<"walker ------输出 x: "<<stAckSiFuConfig->uMissDistanceX<<"------输出 Y: "<<stAckSiFuConfig->uMissDistanceY <<std::endl;
        //printf("------输出 X :%d, Y:%d-- \n", stAckSiFuConfig->uMissDistanceX,stAckSiFuConfig->uMissDistanceY);
        stAckSiFuConfig->crc=project137_serial_checksum(SendBuff+1,5);

        stMissDistanceReport.uMissDistanceX=stSysStatus.trackMissDistance[0];
        stMissDistanceReport.uMissDistanceY=stSysStatus.trackMissDistance[1];
        stMissDistanceReport.crc=project137_serial_checksum(stMissDistanceReport.head.content ,   stMissDistanceReport.head.lLen);
        send(client_sockfd_tcp, &stMissDistanceReport, sizeof(ST_MissDistanceReport), 0);
       
        // for(int i =0;i<sizeof(ST_ACK_SIFU_CONFIG);i++)
        // {
        //     printf("发送数据 %02x",SendBuff[i] );
        // }
        //  printf("结束\n");
    }
    else
    {
        stAckSiFuConfig->type=VIDEOTRACKER_LOSE;
    }
    stAckSiFuConfig->crc=project137_serial_checksum(SendBuff+1,5);
    serial137Link.serial_send(SendBuff, sizeof(ST_ACK_SIFU_CONFIG));
    // send(client_sockfd_tcp, SendBuff, sizeof(ST_ACK_SIFU_CONFIG), 0);
OUT:
    return;
}


/****************************************************************************************
 * 函 数 名 ： VideoPro_TestSend
 * 功    能 ： 解析方位角度 俯仰  横滚角度
 * 输入参数 ： 输入结构体
 * 输出参数 ： 无
 * 返 回 值 ： 0: 成功；-1: 失败
 ***************************************************************************************/
void VideoPro_TestSend()
{
    uint8_t SendBuff[10]={0};
    SendBuff[0]=0xcc;
    SendBuff[1]=0xA1;
    SendBuff[6]=project137_serial_checksum(SendBuff+1,5);
    serial137Link.serial_send(SendBuff, 7);
OUT:
    return;
}



void TimeOut()
{
    static unsigned long long s_CycleInitTime = 0;     /* 请求定时上报状态*/
    static unsigned long long s_CycleInit2Time = 0;     /* 请求定时上报状态*/
    if (lCurTime -  s_CycleInitTime > MAX_TIMEOUT_MODULE_INIT_30FPS)
    {
        // if(WorkModeFlag==WORKMODE_VIDEOTRACKER)
        // {
        //     VideoPro_VideoTrack();
        // }
        VideoPro_VideoTrack();
        // MsgReportedSttaus();
        s_CycleInitTime=lCurTime;
    }

    // if (lCurTime -  s_CycleInit2Time > MAX_TIMEOUT_MODULE_INIT_30FPS)
    // {
    // //    std::cout<<"发送脱靶量"<<std::endl;
    //     // if(WorkModeFlag==WORKMODE_VIDEOTRACKER)
    //     // {
    //     //     VideoPro_VideoTrack();
    //     // }
    //     // VideoPro_VideoTrack();
    //     VideoPro_Send_Frameang();
    // //    VideoPro_TestSend();
    //     s_CycleInit2Time=lCurTime;
    // }

}


