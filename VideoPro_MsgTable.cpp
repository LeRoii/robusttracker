/****************************************************************************************
 * 文 件 名 : Video_MsgTable.cpp
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
#include <stddef.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "common.h"
#include "realtracker.h"
#include "serialport.h"
#include "VideoPro_MsgTable.h"

extern uint8_t trackerStatus[9];
extern ST_TriaxialAngle stTriaxialAngle ;
extern ST_TriaxialAngle_Speed stTriaxialAngleSpeed;
extern ST_XYZ_AcceleratedSpeed stXYZAcceleratedSpeed;
extern Serial serial137Link; //137通信串口
extern ST_SYS_STATUS stSysStatus;
extern ST_AttitudeAngle stAttitudeAngle;

// TCP客户端连接句柄
extern int client_sockfd_tcp ;

uint8_t  WorkModeFlag=MODE_INIT;
extern realtracker *rtracker;


uint8_t project137_serial_checksum(uint8_t *buf,uint8_t len)
{
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        // printf("checksum:%#x\n", checksum);
        checksum = checksum^ buf[i];
    }
    return checksum;
}

/****************************************************************************************
 * 函 数 名 ： VideoPro_Send_Frameang
 * 功    能 ： 发送框架角到上位机
 * 输入参数 ： 输入结构体
 * 输出参数 ： 无
 * 返 回 值 ： 0: 成功；-1: 失败
 ***************************************************************************************/
uint8_t VideoPro_Send_Frameang()
{
    ST_VideoFrameang stVideoFrameang;
    stVideoFrameang.head.head1=HEAD1;
    stVideoFrameang.head.head2=HEAD2;
    stVideoFrameang.head.head3=HEAD3;
    
    stVideoFrameang.head.lLen =sizeof(ST_VideoFrameang)-sizeof(xj_net_msg_hdr)-1;
    stVideoFrameang.msg_type=XJ_FRAMEANG;
    stVideoFrameang.lyaw=stTriaxialAngle.azimuth;
    stVideoFrameang.lpitch= stTriaxialAngle.angleofpitch;
    stVideoFrameang.lRoll=stTriaxialAngle.roll;

    stVideoFrameang.lyaw      =stTriaxialAngle.azimuth;
    stVideoFrameang.lpitch      = stTriaxialAngle.angleofpitch;
    stVideoFrameang.lRoll     =stTriaxialAngle.roll;
   
    // stVideoFrameang.lyaw      =22;
    // stVideoFrameang.lpitch    =33;
    // stVideoFrameang.lRoll     =44;

    stVideoFrameang.Azimuth_AttitudeAngle= stAttitudeAngle.Azimuth_AttitudeAngle   ;
    stVideoFrameang.Angleofpitch_AttitudeAngle= stAttitudeAngle.Angleofpitch_AttitudeAngle;
    stVideoFrameang.Roll_AttitudeAngle=  stAttitudeAngle.Roll_AttitudeAngle   ;
    stVideoFrameang.crc=project137_serial_checksum(stVideoFrameang.head.content,stVideoFrameang.head.lLen);
    // std::cout<<"发送框架角到上位机 "<<std::endl;
    send(client_sockfd_tcp, &stVideoFrameang, sizeof(ST_VideoFrameang), 0);
    

}




/****************************************************************************************
 * 函 数 名 ： project137_AnalysisTriaxialAngleSpeed
 * 功    能 ： 解析陀螺角速度
 * 输入参数 ： 输入结构体
 * 输出参数 ： 无
 * 返 回 值 ： 0: 成功；-1: 失败
 ***************************************************************************************/
void project137_AnalysisTriaxialAngleSpeed(uint8_t *buf)
{

    stTriaxialAngleSpeed.azimuth_speed=buf[1]<<8|buf[2];
    stTriaxialAngleSpeed.angleofpitch_speed=buf[3]<<8|buf[4];
    stTriaxialAngleSpeed.roll_speed=buf[5]<<8|buf[6];
    
    printf(" stTriaxialAngleSpeed.roll_speed: %d  %d %d \n",stTriaxialAngleSpeed.azimuth_speed, stTriaxialAngleSpeed.angleofpitch_speed, stTriaxialAngleSpeed.roll_speed);

}




/****************************************************************************************
 * 函 数 名 ： project137_CmdRange
 * 功    能 ： 发送测距命令
 * 输入参数 ： 输入结构体
 * 输出参数 ： 无
 * 返 回 值 ： 0: 成功；-1: 失败
 ***************************************************************************************/
void project137_CmdRange()
{
    uint8_t SendBuff[10]={0};
    SendBuff[0]=0xCC;
    SendBuff[1]=0xA1;
    SendBuff[6]=project137_serial_checksum(SendBuff+1,5);
    serial137Link.serial_send(SendBuff, 7);
    return;
}


/****************************************************************************************
 * 函 数 名 ： project137_AnalysisAmplitude
 * 功    能 ： 解析幅值数据
 * 输入参数 ： 输入结构体
 * 输出参数 ： 无
 * 返 回 值 ： 0: 成功；-1: 失败
 ***************************************************************************************/
void project137_AnalysisAmplitude(uint8_t *buf)
{
    // uint8_t SendBuff[10]={0};
    int  lAmplitude=0;
    // stTriaxialAngle.azimuth=buf[1]<<8|buf[2];
    // stTriaxialAngle.angleofpitch=buf[3]<<8|buf[4];
    // stTriaxialAngle.roll=buf[5]<<8|buf[6];
    // lDistance=
    printf(" 解析幅值数据  %02x   %02x %02x %02x    \n",buf[1],buf[2],buf[3],buf[4]);

}

/****************************************************************************************
 * 函 数 名 ： project137_AnalysisDistance
 * 功    能 ： 解析测距数据
 * 输入参数 ： 输入结构体
 * 输出参数 ： 无
 * 返 回 值 ： 0: 成功；-1: 失败
 ***************************************************************************************/
void project137_AnalysisDistance(uint8_t *buf)
{
    printf("walker : 1111111111111111111111\n");
    // uint8_t SendBuff[10]={0};
    int  lDistance=0;

      ST_Rangev stRangev;
    stRangev.head.head1=HEAD1;
    stRangev.head.head2=HEAD2;
    stRangev.head.head3=HEAD3;
    stRangev.head.lLen =sizeof(ST_Rangev)-sizeof(xj_net_msg_hdr)-1;
    stRangev.msg_type=XJ_RANGEV;
    stRangev.distance= buf[1]<<8|buf[2];;
    // stRangev.uMissDistanceY=stSysStatus.trackMissDistance[1];
    stRangev.crc=project137_serial_checksum(stRangev.head.content ,  stRangev.head.lLen);
    send(client_sockfd_tcp, &stRangev, sizeof(ST_Rangev), 0);
    // stTriaxialAngle.azimuth=buf[1]<<8|buf[2];
    // stTriaxialAngle.angleofpitch=buf[3]<<8|buf[4];
    // stTriaxialAngle.roll=buf[5]<<8|buf[6];
    // lDistance=
    printf(" 解析测距数据  %02x   %02x %02x %02x    \n",buf[1],buf[2],buf[3],buf[4]);
   

}
/****************************************************************************************
 * 函 数 名 ： project137_VideoTrack
 * 功    能 ： 解析方位角度 俯仰  横滚角度
 * 输入参数 ： 输入结构体
 * 输出参数 ： 无
 * 返 回 值 ： 0: 成功；-1: 失败
 ***************************************************************************************/
void project137_VideoTrack(uint8_t *buf)
{
    uint8_t SendBuff[10]={0};
    ST_ACK_SIFU_CONFIG  *stAckSiFuConfig;
    stAckSiFuConfig=(ST_ACK_SIFU_CONFIG*)SendBuff;
    stAckSiFuConfig->head=HEAD;
    if( stSysStatus.trackOn != true)
    {
        goto OUT;
    }
    if(trackerStatus[4]&0x2)
    {
        stAckSiFuConfig->type=WORKMODE_VIDEOTRACKER;
        stAckSiFuConfig->uMissDistanceX=stSysStatus.trackMissDistance[0];
        stAckSiFuConfig->uMissDistanceY=stSysStatus.trackMissDistance[1];

    }
    else
    {
        stAckSiFuConfig->type=VIDEOTRACKER_LOSE;
    }
    stAckSiFuConfig->crc=project137_serial_checksum(SendBuff+1,5);
    serial137Link.serial_send(SendBuff, sizeof(ST_ACK_SIFU_CONFIG));
OUT:
    return;
}

void project137_handshake(uint8_t *buf)
{
    uint8_t buff[20]={0};
    buff[0]=0xCC;
    buff[2]=0x00;
    buff[6]=project137_serial_checksum(buff+1,5);
    serial137Link.serial_send(buff, 6);
}

/****************************************************************************************
 * 函 数 名 ： project137_AnalysisAcceleratedSpeed
 * 功    能 ： 解析加速度数据
 * 输入参数 ： 输入结构体
 * 输出参数 ： 无
 * 返 回 值 ： 0: 成功；-1: 失败
 ***************************************************************************************/
void project137_AnalysisAcceleratedSpeed(uint8_t *buf)
{
    // short XAcceleratedSpeed;    //X轴加速度
    // short YAcceleratedSpeed;    //Y轴加速度
    // short ZAcceleratedSpeed;    //Z轴加速度
    stXYZAcceleratedSpeed.XAcceleratedSpeed=buf[1]<<8|buf[2];
    stXYZAcceleratedSpeed.YAcceleratedSpeed=buf[3]<<8|buf[4];
    stXYZAcceleratedSpeed.ZAcceleratedSpeed=buf[5]<<8|buf[6];
    printf(" 解析加速度数据 %d  %d %d \n", stXYZAcceleratedSpeed.XAcceleratedSpeed,  stXYZAcceleratedSpeed.YAcceleratedSpeed, stXYZAcceleratedSpeed.ZAcceleratedSpeed);
}

/****************************************************************************************
 * 函 数 名 ： project137_AnalysisTriaxialAngle
 * 功    能 ： 解析方位角度 俯仰  横滚角度
 * 输入参数 ： 输入结构体
 * 输出参数 ： 无
 * 返 回 值 ： 0: 成功；-1: 失败
 ***************************************************************************************/
void project137_AnalysisTriaxialAngle(uint8_t *buf)
{
    uint8_t SendBuff[10]={0};
    stTriaxialAngle.azimuth=buf[1]<<8|buf[2];
    stTriaxialAngle.angleofpitch=buf[3]<<8|buf[4];
    stTriaxialAngle.roll=buf[5]<<8|buf[6];
    VideoPro_Send_Frameang();
    //printf( "解析方位角度 %d  %d\n",  stTriaxialAngle.azimuth,  stTriaxialAngle.angleofpitch);
   // std::cout<< "解析方位角度    : "<< stTriaxialAngle.azimuth << " 俯仰 "  <<   stTriaxialAngle.azimuth  << "横滚角度 :"<<  stTriaxialAngle.azimuth <<std::endl;

    // ST_ACK_SIFU_CONFIG  *stAckSiFuConfig;
    // stAckSiFuConfig=(ST_ACK_SIFU_CONFIG * )SendBuff;
    // stAckSiFuConfig->head=HEAD;
    // stAckSiFuConfig->type=WorkModeFlag;
    // stAckSiFuConfig->uMissDistanceX=stSysStatus.trackMissDistance[0];
    // stAckSiFuConfig->uMissDistanceY=stSysStatus.trackMissDistance[1];
    // stAckSiFuConfig->crc=project137_serial_checksum(SendBuff+1,5);
    // serial137Link.serial_send(SendBuff, sizeof(ST_ACK_SIFU_CONFIG));
}
/****************************************************************************************
 * 函 数 名 ： project137_AnalysisAttitudeAngle
 * 功    能 ： 解析姿态角
 * 输入参数 ： 输入结构体
 * 输出参数 ： 无
 * 返 回 值 ： 0: 成功；-1: 失败
 ***************************************************************************************/
void project137_AnalysisAttitudeAngle(uint8_t *buf)
{
    // short Azimuth_AttitudeAngle;    //x姿态角
    // short Angleofpitch_AttitudeAngle;    //Y姿态角
    // short Roll_AttitudeAngle;    //Z姿态角
    stAttitudeAngle.Azimuth_AttitudeAngle     =buf[1]<<8|buf[2];
    stAttitudeAngle.Angleofpitch_AttitudeAngle=buf[3]<<8|buf[4];
    stAttitudeAngle.Roll_AttitudeAngle        =buf[5]<<8|buf[6];
}


void project137_Sendstatus()
{
    uint8_t buff[20]={0};
    buff[0]=0xCC;
    buff[2]=WorkModeFlag;//工作状态
    buff[6]=project137_serial_checksum(buff+1,5);
    serial137Link.serial_send(buff, 6);
}


void project173_SendFollowYawPitch(short lyaw,short lpitch)
{
    uint8_t buff[20]={0};
    ST_ServoDirectionFollowYawPitch * pstServoDirectionFollowYawPitch=( ST_ServoDirectionFollowYawPitch * )buff;
    pstServoDirectionFollowYawPitch->head=0xCC;
    pstServoDirectionFollowYawPitch->type=0x05;//工作状态
    pstServoDirectionFollowYawPitch->lyaw=htons(lyaw);
    pstServoDirectionFollowYawPitch->lpitch=htons(lpitch);//工作状态

    // pstServoDirectionFollowYawPitch->lyaw=lyaw;
    // pstServoDirectionFollowYawPitch->lpitch=lpitch;//工作状态
    pstServoDirectionFollowYawPitch->crc=project137_serial_checksum(buff+1,5);
    serial137Link.serial_send(buff, 7);
}

void project173_SendFollowRoll(short lRoll)
{
    uint8_t buff[20]={0};
    ST_ServoDirectionFollowRoll * pstServoDirectionFollowRoll=( ST_ServoDirectionFollowRoll * )buff;
    pstServoDirectionFollowRoll->head=0xCC;
    pstServoDirectionFollowRoll->type=0x06;//工作状态
    // pstServoDirectionFollowRoll->lRoll=stTriaxialAngle.roll;
    pstServoDirectionFollowRoll->lRoll=htons(lRoll);//工作状态
    pstServoDirectionFollowRoll->crc=project137_serial_checksum(buff+1,5);
    serial137Link.serial_send(buff, 7);
}

// void project173_SendFollowRoll(short lRoll)
// {
//     uint8_t buff[20]={0};
//     ST_ServoDirectionFollowRoll * pstServoDirectionFollowRoll=( ST_ServoDirectionFollowRoll * )buff;
//     pstServoDirectionFollowRoll->head=0xCC;
//     pstServoDirectionFollowRoll->type=0x06;//工作状态
//     pstServoDirectionFollowRoll->lRoll=stTriaxialAngle.roll;
//     pstServoDirectionFollowRoll->crc=project137_serial_checksum(buff+1,5);
//     serial137Link.serial_send(buff, 7);
// }

/****************************************************************************************
 * 函 数 名 ： project137_ParseSerialData
 * 功    能 ： 项目137函数处理接口
 * 输入参数 ： 输入结构体
 * 输出参数 ： 无
 * 返 回 值 ： 0: 成功；-1: 失败
 ***************************************************************************************/
void project137_ParseSerialData(uint8_t *buf)
{
    uint8_t frameID = buf[0];
     printf("frameID__________________________________________%02x\n", frameID);
    switch (frameID)
    {
        case 0x00:  //握手
            project137_handshake(buf);
            break;
        case 0x05:  //随动
            WorkModeFlag=WORKMODE_FLOWUP;
            stSysStatus.trackOn = false;
            project137_AnalysisTriaxialAngle(buf);
            break;
        case 0x06:  //随动
            WorkModeFlag=WORKMODE_FLOWUP;
            stSysStatus.trackOn = false;
            project137_AnalysisTriaxialAngle(buf);
            break;
        case 0x10:  //初始化
            WorkModeFlag=WORKMODE_INIT;
            stSysStatus.trackOn = false;
            project137_AnalysisTriaxialAngle(buf);
            break;
        case 0x12:  //锁定
            WorkModeFlag=WORKMODE_LOCK;
            stSysStatus.trackOn = false;
            project137_AnalysisTriaxialAngle(buf);
            break;
        case 0x33:  //手动
            printf("手动模式\n");
            WorkModeFlag=WORKMODE_HANDMOVE;
            stSysStatus.trackOn = false;
            stSysStatus.trackerInited=false;
            // rtracker->reset();
            project137_AnalysisTriaxialAngle(buf);
            break;
        case 0x73:  //视频跟踪
            WorkModeFlag=WORKMODE_VIDEOTRACKER;
            if(stSysStatus.trackOn == false)
            {
                stSysStatus.trackOn = true;
                stSysStatus.trackAssignPoint = cv::Point(960, 540);
            }
            project137_AnalysisTriaxialAngle(buf);
            project137_VideoTrack(buf);
            break;
        case 0x14:  //惯性
            WorkModeFlag=WORKMODE_INERTIA;
            stSysStatus.trackOn = false;
            project137_AnalysisTriaxialAngle(buf);
            break;
        case 0x21:  //陀螺数据
            project137_AnalysisTriaxialAngleSpeed(buf);
            break;
        case 0x22:  //加速度数据
            project137_AnalysisAcceleratedSpeed(buf);
            break;
        case 0x23:  //姿态角数据
            project137_AnalysisAttitudeAngle(buf);
            break;
        case 0x15:  //启动识别
            project137_AnalysisTriaxialAngle(buf);
            break;            
        case 0x1D:  //黑/白
            project137_AnalysisTriaxialAngle(buf);
            break;       
        case 0x1c:  //增强
            project137_AnalysisTriaxialAngle(buf);
            break;  
        case 0x19:  //增强+
            project137_AnalysisTriaxialAngle(buf);
            break;  
        case 0x51:  //up
            rtracker->gateAdjust(0);
            break;   
        case 0x52:  //down
            rtracker->gateAdjust(1);
            break;   
        case 0x53:  //left
            rtracker->gateAdjust(2);
            break;   
        case 0x54:  //right
            rtracker->gateAdjust(3);
            break;         
        case 0x59:  //增强+
            project137_AnalysisTriaxialAngle(buf);
            break;   
        case 0x18:  //增强-
            project137_AnalysisTriaxialAngle(buf);
            break;    
        case 0x58:  //增强-
            project137_AnalysisTriaxialAngle(buf);
            break;
        case 0x11: 
            project137_AnalysisTriaxialAngle(buf);
            break;
        case 0xB1: 
            project137_AnalysisDistance(buf);
            break;
        case 0xB2: 
            project137_AnalysisAmplitude(buf);
            break;
        default:
            break;
    }
}


