#ifndef SERIAL_H
#define SERIAL_H

#include <stdio.h>  
#include <string.h>  
#include <stdlib.h>  
  
#include <fcntl.h>  
#include <unistd.h>  
  
#include <termios.h> //set baud rate  
  
#include <sys/select.h>  
#include <sys/time.h>  
#include <sys/types.h>  
#include <errno.h>  
#include <sys/stat.h> 

#include <thread>
#include "common.h"

inline const int GetMsgLen(uint8_t u8)
{
    // printf("us:%d\n", u8);
    return u8 & 0x3F;
}

void VL_ParseSerialData(uint8_t* buf);
uint8_t viewlink_protocal_checksum(uint8_t* buf);
EN_DATA_FRAME_TYPE GetFrameType(uint8_t *send_buf, int Len);
bool CheckFrameHeader(uint8_t *send_buf, int Len);
// int ProcessSerialData(uint8_t *inputBUf, int inputLen, uint8_t *OutputBuf, int &outputLen);



using namespace std;

class Serial
{
public:
    Serial();
    ~Serial();
    int set_serial(int port);
    int serial_send(uint8_t* buffSenData, unsigned int sendDataNum);    //buffSenData should len 1024
    int serial_recieve(uint8_t* buffRcvData);


    //设置状态机信息
    const int SetStatus(const int status);
    //获取状态机信息
    const int GetStatus() const;
            
    //启动线程,
    //返回值1-正在通信, 0-通道正常，准备开启接收消息的线程，-1-断开通信，通信通道不正常，需要重新设置通信MSG_Key值
    int OnStart();

    //设置回调函数
    // int set_callback_func(SERIAL_CALLBACK_FUNC func);

    int ProcessSerialData(uint8_t *inputBUf, int inputLen, uint8_t *OutputBuf, int &outputLen);
    int bufLen = 0;
    //关闭串口
    int closePort(int fd);

private:
    int openPort(int fd, int comport);
    int setOpt(int fd, int nSpeed, int nBits, char nEvent, int nStop);
    int readDataTty(int fd, uint8_t *rcv_buf, int TimeOut, int Len);
    int sendDataTty(int fd, uint8_t *send_buf, int Len);
    
    

    
    //使用C++11标准类thread创建线程，注意需要gcc版本支持c++11
    //从客户端接收数据执行体
    void OnReceive();
    
    //向客户端发送数据执行体
    //void OnSend();
    
    //解析数据执行体
    //void OnAnalysis();
    
    std::thread r_thread;
    //std::thread s_thread;
    //std::thread a_thread;


private:
    int iSetOpt;//SetOpt 的增量i  
    int fdSerial; 

    int portId;

    int m_RunStatus;//-1-未初始化，0-就绪，1-运行
    //针对IPC回调函数
    // SERIAL_CALLBACK_FUNC m_Send_Data_Func;

    uint8_t buf[1000];
    int st = 0;
};


#endif