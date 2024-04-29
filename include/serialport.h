// SerialPort.h
#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <cstdint>
#include <cstddef>
#include "common.h"

#include <termios.h>
#include <atomic>
#include <fcntl.h>
#include <cstring>
#include <unistd.h>


EN_DATA_FRAME_TYPE GetFrameType(std::vector<uint8_t> &send_buf, int Len);
void VL_ParseSerialData(uint8_t* buf);
bool CheckFrameHeader(uint8_t *send_buf, int Len);
uint8_t viewlink_protocal_checksum(uint8_t *buf);
uint16_t viewlink_protocal_tcp_checksum(uint8_t *buf);

inline const int GetMsgLen(uint8_t u8)
{
    // printf("us:%d\n", u8);
    return u8 & 0x3F;
}

class Serial
{
public:
    Serial();
    ~Serial();
    int set_serial(int port);
    int serial_send(uint8_t* buffSenData, unsigned int sendDataNum);    //buffSenData should len 1024
    int serial_receive(uint8_t* buffRcvData);


    //设置状态机信息
    const int SetStatus(const int status);
    //获取状态机信息
    const int GetStatus() const;
            
    //启动线程,
    //返回值1-正在通信, 0-通道正常，准备开启接收消息的线程，-1-断开通信，通信通道不正常，需要重新设置通信MSG_Key值
    // int OnStart();

    //设置回调函数
    // int set_callback_func(SERIAL_CALLBACK_FUNC func);

    int ProcessSerialData(uint8_t *inputBUf, int inputLen, uint8_t *OutputBuf, int &outputLen);
    int bufLen = 0;
    int fdSerial; 

private:
    int openPort(int fd, int comport);
    int setOpt(int fd, int nSpeed, int nBits, char nEvent, int nStop);
    int readDataTty(int fd, uint8_t *rcv_buf, int TimeOut, int Len);
    int sendDataTty(int fd, uint8_t *send_buf, int Len);
    
    //关闭串口
    int closePort(int fd);

    
    //使用C++11标准类thread创建线程，注意需要gcc版本支持c++11
    //从客户端接收数据执行体
    void OnReceive();
    
    //向客户端发送数据执行体
    //void OnSend();
    
    //解析数据执行体
    //void OnAnalysis();
    
    // std::thread r_thread;
    //std::thread s_thread;
    //std::thread a_thread;


private:
    int iSetOpt;//SetOpt 的增量i  
    

    int portId;

    int m_RunStatus;//-1-未初始化，0-就绪，1-运行
    //针对IPC回调函数
    // SERIAL_CALLBACK_FUNC m_Send_Data_Func;

    uint8_t buf[1000];
    int st = 0;
};





class SerialPort {
private:
    int fd; // File descriptor for the serial port

public:
    SerialPort() : fd(-1) {}

    ~SerialPort() {
        if (fd != -1) {
            close(fd);
        }
    }

    int set_serial(const char * port, int nSpeed, int nBits, char nEvent, int nStop) {
        fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0) {
            perror(port);
            return -1;
        }

        struct termios tty;
        memset(&tty, 0, sizeof tty);
        if (tcgetattr(fd, &tty) != 0) {
            perror("tcgetattr");
            return -1;
        }

        // Speed
        cfsetospeed(&tty, nSpeed);
        cfsetispeed(&tty, nSpeed);

        // Disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 0.5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls,
                                          // enable reading
        tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
        tty.c_cflag |= (nEvent == 'N') ? 0 : PARENB; // parity for none (N)
        if (nEvent == 'O') tty.c_cflag |= PARODD;
        tty.c_cflag &= ~CSTOPB;
        if (nStop == 2) tty.c_cflag |= CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8; // 8-bit chars
        if (nBits == 7) tty.c_cflag |= CS7; // 7-bit chars
        if (nBits == 6) tty.c_cflag |= CS6; // 6-bit chars
        if (nBits == 5) tty.c_cflag |= CS5; // 5-bit chars

        // clean the modem line and activate the settings for the port
        tcflush(fd, TCIOFLUSH);
        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            perror("tcsetattr");
            return -1;
        }

        return 0;
    }

    int serial_send(uint8_t* buffSenData, unsigned int sendDataNum) {
        if (sendDataNum > 1024) {
            std::cerr << "Error: send data length exceeds 1024 bytes." << std::endl;
            return -1;
        }
        return write(fd, buffSenData, sendDataNum);
    }

    int serial_receive(uint8_t* buffRcvData) {
        return read(fd, buffRcvData, 1024);
    }
};

#endif // SERIALPORT_H
