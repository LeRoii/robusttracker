#include <iostream>
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

#include "serialport.h"

#include "VideoPro_Uart.h"

extern   std::atomic<bool> interrupted;
extern Serial serialIRLink; //137通信串口


/****************************************************************************************
 * 函 数 名 ： projectIR_SendIRInit
 * 功    能 ： 发送红外初始化
 * 输入参数 ： 输入结构体
 * 输出参数 ： 无
 * 返 回 值 ： 无
 ***************************************************************************************/
void projectIR_SendIRInit()
{
    uint8_t  message[11] = {0x68,0x24,0x03,0x01,0x00,0x00,0x96,0x00,0x03,0x02,0x01};
    serialIRLink.serial_send(message, 11);
}

/****173项目IR串口通信协议****/
void serialIRFunc()
{
     int lSts;
     uint8_t buffRcvData[128] = {0};
     int retLen = 0;
     int i=0;

    // std::vector<uint8_t> receiveBuffer;
    // const std::vector<uint8_t> frameStart = {0xCC};

    struct pollfd stPollFd[1];
    stPollFd[0].fd = serialIRLink.fdSerial;
    stPollFd[0].events = POLLIN;
    if(stPollFd[0].fd < 0)
    {
    	// WriteLog(LOG_ERR,"[%s]open error", DEV_NAME1);
        printf(" serialIRLink.fdSerial  open error\n");
    	// goto OUT;
    }
    while (!interrupted.load())
    {

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

            // for(int i=0;i<retLen;i++)
            //     printf("接受字符串：%02x\r\n", buffRcvData[i]);
            //  for(int i=0;i< retLen-8;i++) 
            //  {
            //     if(buffRcvData[i]==0xCC && buffRcvData[i+8]==project137_serial_checksum(buffRcvData+i+1,7))
            //     {
            //     //    printf("进入处理%02x \n",buffRcvData[i+1]);
            //         // project137_ParseSerialData(buffRcvData+i+1);
            //     }
            //  }

        }
    }

   close(stPollFd[0].fd);
}
