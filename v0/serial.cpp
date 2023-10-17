#include <iostream>
#include "serial.h"


ST_A1_CONFIG stA1Cfg = {0};
ST_A2_CONFIG stA2Cfg = {0};
ST_C1_CONFIG stC1Cfg = {0};
ST_C2_CONFIG stC2Cfg = {0};
ST_E1_CONFIG stE1Cfg = {0};
ST_E2_CONFIG stE2Cfg = {0};
ST_S1_CONFIG stS1Cfg = {0};
ST_S2_CONFIG stS2Cfg = {0};
ST_U_CONFIG stUCfg = {0};
ST_A1C1E1_CONFIG stA1C1E1Cfg = {0};
ST_A2C2E2_CONFIG stA2C2E2Cfg = {0};
ST_A1C1E1S1_CONFIG stA1C1E1S1Cfg = {0};
ST_A2C2E2S2_CONFIG stA2C2E2S2Cfg = {0};

static uint8_t viewlink_protocal_checksum(uint8_t* buf)
{
    uint8_t len = buf[3];
    uint8_t checksum = len;
    for(uint8_t i=0;i<len-2;i++)
    {
        checksum = checksum ^ buf[4+i];
    }
    return(checksum);
}

static void VL_ParseSerialData_A1(uint8_t* buf)
{
    ST_A1_CONFIG *a1Cfg = (ST_A1_CONFIG*)buf;
    stA1Cfg.enServoCtrlMode = a1Cfg->enServoCtrlMode;
    memcpy(stA1Cfg.para1, a1Cfg->para1, 2);
    memcpy(stA1Cfg.para2, a1Cfg->para2, 2);
    memcpy(stA1Cfg.para3, a1Cfg->para3, 2);
    memcpy(stA1Cfg.para4, a1Cfg->para4, 2);
}

//to do
static void VL_ParseSerialData_C1(uint8_t* buf)
{
    ST_C1_CONFIG *c1Cfg = (ST_C1_CONFIG*)buf;
    stC1Cfg.enDispMode = c1Cfg->enDispMode;
    stC1Cfg.enOpCmd1 = c1Cfg->enOpCmd1;
    stC1Cfg.enOpCmd1Para = c1Cfg->enOpCmd1Para;
    stC1Cfg.laserCmd = c1Cfg->laserCmd;
}

//to do
static void VL_ParseSerialData_E1(uint8_t* buf)
{
    ST_E1_CONFIG *e1Cfg = (ST_E1_CONFIG*)buf;

    stE1Cfg.enTrackSourceMode = e1Cfg->enTrackSourceMode;
    stE1Cfg.u8Para1 = e1Cfg->u8Para1;
    stE1Cfg.enBaseOpMode = e1Cfg->enBaseOpMode;
    stE1Cfg.u8Para2 = e1Cfg->u8Para2;
}

static void VL_ParseSerialData_S1(uint8_t* buf)
{
    ST_S1_CONFIG *s1Cfg = (ST_S1_CONFIG*)buf;

    stS1Cfg.enCalcCtrlMode = s1Cfg->enCalcCtrlMode;
    stS1Cfg.para1 = s1Cfg->para1;
    memcpy(stS1Cfg.para2, s1Cfg->para2, 12);
}

static void VL_ParseSerialData_A2(uint8_t* buf)
{
    ST_A2_CONFIG *a2Cfg = (ST_A2_CONFIG*)buf;
    stA2Cfg.enServoOpMode = a2Cfg->enServoOpMode;
    stA2Cfg.enUnuseStateReturnCtrlMode = a2Cfg->enUnuseStateReturnCtrlMode;
    stA2Cfg.u2UnusedFrameCounter = a2Cfg->u2UnusedFrameCounter;
    stA2Cfg.adjustmentAmount = a2Cfg->adjustmentAmount;
}

//to do
static void VL_ParseSerialData_C2(uint8_t* buf)
{
    ST_C2_CONFIG *c2Cfg = (ST_C2_CONFIG*)buf;
    stC2Cfg.opCmd1 = c2Cfg->opCmd1;
    memcpy(stC2Cfg.opCmdPara1, c2Cfg->opCmdPara1, 2);
}

//to do
static void VL_ParseSerialData_E2(uint8_t* buf)
{
    ST_E2_CONFIG *e2Cfg = (ST_E2_CONFIG*)buf;
    stE2Cfg.enExtendCmd1 = e2Cfg->enExtendCmd1;
    memcpy(stE2Cfg.para1, e2Cfg->para1, 2);
    memcpy(stE2Cfg.para2, e2Cfg->para2, 2);
}

static void VL_ParseSerialData_S2(uint8_t* buf)
{
    ST_S2_CONFIG *s2Cfg = (ST_S2_CONFIG*)buf;
    stS2Cfg.enCfgCmd = s2Cfg->enCfgCmd;
    memcpy(stS2Cfg.para, s2Cfg->para, 4);
}

static void VL_ParseSerialData_U(uint8_t* buf)
{
    ST_U_CONFIG *uCfg = (ST_U_CONFIG*)buf;
    stUCfg.enOpCmd = uCfg->enOpCmd;
    memcpy(stUCfg.para, uCfg->para, 9);
}

static void VL_ParseSerialData_A1C1E1(uint8_t* buf)
{
    ST_A1_CONFIG *a1Cfg = (ST_A1_CONFIG*)buf;
    stA1C1E1Cfg.a1Config.enServoCtrlMode = a1Cfg->enServoCtrlMode;
    memcpy(stA1C1E1Cfg.a1Config.para1, a1Cfg->para1, 2);
    memcpy(stA1C1E1Cfg.a1Config.para2, a1Cfg->para2, 2);
    memcpy(stA1C1E1Cfg.a1Config.para3, a1Cfg->para3, 2);
    memcpy(stA1C1E1Cfg.a1Config.para4, a1Cfg->para4, 2);

    uint8_t *tempData = buf + 9; // 9个字节
    ST_C1_CONFIG *c1Cfg = (ST_C1_CONFIG*)tempData;
    stA1C1E1Cfg.c1Config.enDispMode =  c1Cfg->enDispMode;
    stA1C1E1Cfg.c1Config.enOpCmd1 = c1Cfg->enOpCmd1;
    stA1C1E1Cfg.c1Config.enOpCmd1Para = c1Cfg->enOpCmd1Para;
    stA1C1E1Cfg.c1Config.laserCmd = c1Cfg->laserCmd;

    tempData = tempData + 2; // 2个字节
    ST_E1_CONFIG *e1Cfg = (ST_E1_CONFIG*)tempData;
    stA1C1E1Cfg.e1Config.enTrackSourceMode = e1Cfg->enTrackSourceMode;
    stA1C1E1Cfg.e1Config.u8Para1 = e1Cfg->u8Para1;
    stA1C1E1Cfg.e1Config.enBaseOpMode = e1Cfg->enBaseOpMode;
    stA1C1E1Cfg.e1Config.u8Para2 = e1Cfg->u8Para2;
}

static void VL_ParseSerialData_A2C2E2(uint8_t* buf)
{
    ST_A2_CONFIG *a2Cfg = (ST_A2_CONFIG*)buf;
    stA2C2E2Cfg.a2Config.enServoOpMode = a2Cfg->enServoOpMode;
    stA2C2E2Cfg.a2Config.enUnuseStateReturnCtrlMode = a2Cfg->enUnuseStateReturnCtrlMode;
    stA2C2E2Cfg.a2Config.u2UnusedFrameCounter = a2Cfg->u2UnusedFrameCounter;
    stA2C2E2Cfg.a2Config.adjustmentAmount = a2Cfg->adjustmentAmount;

    uint8_t *tempData = buf + 2; // 2个字节
    ST_C2_CONFIG *c2Cfg = (ST_C2_CONFIG*)tempData;
    stA2C2E2Cfg.c2Config.opCmd1 = c2Cfg->opCmd1;
    memcpy(stA2C2E2Cfg.c2Config.opCmdPara1, c2Cfg->opCmdPara1, 2);

    tempData = tempData + 3; // 3个字节
    ST_E2_CONFIG *e2Cfg = (ST_E2_CONFIG*)tempData;
    stA2C2E2Cfg.e2Config.enExtendCmd1 = e2Cfg->enExtendCmd1;
    memcpy(stA2C2E2Cfg.e2Config.para1, e2Cfg->para1, 2);
    memcpy(stA2C2E2Cfg.e2Config.para2, e2Cfg->para2, 2);
}

static void VL_ParseSerialData_A1C1E1S1(uint8_t* buf)
{
    ST_A1_CONFIG *a1Cfg = (ST_A1_CONFIG*)buf;
    stA1C1E1Cfg.a1Config.enServoCtrlMode = a1Cfg->enServoCtrlMode;
    memcpy(stA1C1E1Cfg.a1Config.para1, a1Cfg->para1, 2);
    memcpy(stA1C1E1Cfg.a1Config.para2, a1Cfg->para2, 2);
    memcpy(stA1C1E1Cfg.a1Config.para3, a1Cfg->para3, 2);
    memcpy(stA1C1E1Cfg.a1Config.para4, a1Cfg->para4, 2);

    uint8_t *tempData = buf + 9;// 9个字节
    ST_C1_CONFIG *c1Cfg = (ST_C1_CONFIG*)tempData;
    stA1C1E1Cfg.c1Config.enDispMode =  c1Cfg->enDispMode;
    stA1C1E1Cfg.c1Config.enOpCmd1 = c1Cfg->enOpCmd1;
    stA1C1E1Cfg.c1Config.enOpCmd1Para = c1Cfg->enOpCmd1Para;
    stA1C1E1Cfg.c1Config.laserCmd = c1Cfg->laserCmd;

    tempData = tempData + 2; // 2个字节
    ST_E1_CONFIG *e1Cfg = (ST_E1_CONFIG*)tempData;
    stA1C1E1Cfg.e1Config.enTrackSourceMode = e1Cfg->enTrackSourceMode;
    stA1C1E1Cfg.e1Config.u8Para1 = e1Cfg->u8Para1;
    stA1C1E1Cfg.e1Config.enBaseOpMode = e1Cfg->enBaseOpMode;
    stA1C1E1Cfg.e1Config.u8Para2 = e1Cfg->u8Para2;

    tempData = tempData + 3; // 3个字节
    ST_S1_CONFIG *s1Cfg = (ST_S1_CONFIG*)tempData;

    stA1C1E1S1Cfg.s1Config.enCalcCtrlMode =  s1Cfg->enCalcCtrlMode;
    stA1C1E1S1Cfg.s1Config.para1 = s1Cfg->para1;
    memcpy(stA1C1E1S1Cfg.s1Config.para2, s1Cfg->para2, 12);
}

static void VL_ParseSerialData_A2C2E2S2(uint8_t* buf)
{
    ST_A2_CONFIG *a2Cfg = (ST_A2_CONFIG*)buf;
    stA2C2E2Cfg.a2Config.enServoOpMode = a2Cfg->enServoOpMode;
    stA2C2E2Cfg.a2Config.enUnuseStateReturnCtrlMode = a2Cfg->enUnuseStateReturnCtrlMode;
    stA2C2E2Cfg.a2Config.u2UnusedFrameCounter = a2Cfg->u2UnusedFrameCounter;
    stA2C2E2Cfg.a2Config.adjustmentAmount = a2Cfg->adjustmentAmount;

    uint8_t *tempData = buf + 2; // 2个字节
    ST_C2_CONFIG *c2Cfg = (ST_C2_CONFIG*)tempData;
    stA2C2E2Cfg.c2Config.opCmd1 = c2Cfg->opCmd1;
    memcpy(stA2C2E2Cfg.c2Config.opCmdPara1, c2Cfg->opCmdPara1, 2);

    tempData = tempData + 3; // 3个字节
    ST_E2_CONFIG *e2Cfg = (ST_E2_CONFIG*)tempData;
    stA2C2E2Cfg.e2Config.enExtendCmd1 = e2Cfg->enExtendCmd1;
    memcpy(stA2C2E2Cfg.e2Config.para1, e2Cfg->para1, 2);
    memcpy(stA2C2E2Cfg.e2Config.para2, e2Cfg->para2, 2);

    tempData = tempData + 5; // 5个字节
    ST_S2_CONFIG *s2Cfg = (ST_S2_CONFIG*)tempData;

    stA2C2E2S2Cfg.s2Config.enCfgCmd = s2Cfg->enCfgCmd;
    memcpy(stA2C2E2S2Cfg.s2Config.para, s2Cfg->para, 4);
}

static void VL_ParseSerialData(uint8_t* buf)
{
    uint8_t frameID = buf[4];
    switch(frameID)
    {
        case 0x30:
            VL_ParseSerialData_A1C1E1(buf);
            break;
        case 0x32:
            VL_ParseSerialData_A1C1E1S1(buf);
            break;
        case 0x1A:
            VL_ParseSerialData_A1(buf);
            break;
        case 0x1C:
            // uint8_t *data = buf + 5;
            VL_ParseSerialData_C1(buf);
            break;
        case 0x1E:
            VL_ParseSerialData_E1(buf);
            break;
        case 0x16:
            VL_ParseSerialData_S1(buf);
            break;
        case 0x31:
            VL_ParseSerialData_A2C2E2(buf);
            break;
        case 0x33:
            VL_ParseSerialData_A2C2E2S2(buf);
            break;
        case 0x2A:
            VL_ParseSerialData_A2(buf);
            break;
        case 0x2C:
            VL_ParseSerialData_C2(buf);
            break;
        case 0x2E:
            VL_ParseSerialData_E2(buf);
            break;
        case 0x26:
            VL_ParseSerialData_S2(buf);
            break;
        case 0x01:
            VL_ParseSerialData_U(buf);
            break;
        default:
            break;
    }
}

Serial::Serial()
{
    this->iSetOpt = 0;
    this->fdSerial = 0;

    this->m_RunStatus = -1;//串口通道未准备好
}

Serial::~Serial()
{
}

int Serial::openPort(int fd, int comport)
{
    /*if (comport == 1)  
    {  
        fd = open("/dev/ttyTHS1", O_RDWR | O_NOCTTY | O_NDELAY);  
        if (-1 == fd)  
        {  
            perror("Can't Open Serial Port /dev/ttyTHS1");  
            return(-1);  
        }  
        else  
        {  
            printf("open /dev/ttyTHS1 succeed .....\n");  
        }  
    }  
    else if (comport == 0)  
    {  
        fd = open("/dev/ttyTHS0", O_RDWR | O_NOCTTY | O_NDELAY);  
        if (-1 == fd)  
        {  
            perror("Can't Open Serial Port /dev/ttyTHS0");  
            return(-1);  
        }  
        else  
        {  
            printf("open /dev/ttyTHS0 succeed .....\n");  
        }  
    } 
    else if (comport == 2)  
    {  
        fd = open("/dev/ttyTCU0", O_RDWR | O_NOCTTY | O_NDELAY);  
        if (-1 == fd)  
        {  
            perror("Can't Open Serial Port /dev/ttyTCU0");  
            return(-1);  
        }  
        else  
        {  
            printf("open /dev/ttyTCU0 succeed .....\n");  
        }  
    }*/ 

    //根据串口参数读取设备文件
    char devFile[1024]={0};
    //串口1，读取串口设备文件
    if(comport == 1)
    {
        // devFile = "/dev/ttyTHS1";
        fd = open("/dev/ttyTHS1", O_RDWR | O_NOCTTY | O_NDELAY);  
    }
    //串口2，读取串口设备文件
    else if(comport == 2)
    {
        // devFile = "/dev/ttyTHS2";
        fd = open("/dev/ttyTHS2", O_RDWR | O_NOCTTY | O_NDELAY);  
    }

    fd = open(devFile, O_RDWR | O_NOCTTY | O_NDELAY);  
    if (-1 == fd)  
    {  
        perror("Can't Open Serial Port");  
        return(-1);  
    }  
    else  
    {  
        printf("open %s succeed .....\n", devFile);  
    }  

    if (fcntl(fd, F_SETFL, 0)<0)  
    {  
        printf("fcntl failed!\n");  
    }  
    else  
    {  
        printf("fcntl=%d\n", fcntl(fd, F_SETFL, 0));  
    }  
    if (isatty(STDIN_FILENO) == 0)  
    {  
        printf("standard input is not a terminal device\n");  
    }  
    else  
    {  
        printf("is a tty success!\n");  
    }  
    printf("fd-open=%d\n", fd);  
    return fd;
}

int Serial::setOpt(int fd, int nSpeed, int nBits, char nEvent, int nStop)  
{  
    struct termios newtio, oldtio;  
    if (tcgetattr(fd, &oldtio) != 0)  
    {  
        perror("SetupSerial 1");  
        return -1;  
    }  
    bzero(&newtio, sizeof(newtio));  
    newtio.c_cflag |= CLOCAL | CREAD;  
    newtio.c_cflag &= ~CSIZE;  

    switch (nBits)  
    {  
    case 7:  
        newtio.c_cflag |= CS7;  
        break;  
    case 8:  
        newtio.c_cflag |= CS8;  
        break;  
    }  

    switch (nEvent)  
    {  
    case 'O':                     //奇校验  
        newtio.c_cflag |= PARENB;  
        newtio.c_cflag |= PARODD;  
        newtio.c_iflag |= (INPCK | ISTRIP);  
        break;  
    case 'E':                     //偶校验  
        newtio.c_iflag |= (INPCK | ISTRIP);  
        newtio.c_cflag |= PARENB;  
        newtio.c_cflag &= ~PARODD;  
        break;  
    case 'N':                    //无校验  
        newtio.c_cflag &= ~PARENB;  
        break;  
    }  

    switch (nSpeed)  
    {  
    case 2400:  
        cfsetispeed(&newtio, B2400);  
        cfsetospeed(&newtio, B2400);  
        break;  
    case 4800:  
        cfsetispeed(&newtio, B4800);  
        cfsetospeed(&newtio, B4800);  
        break;  
    case 9600:  
        cfsetispeed(&newtio, B9600);  
        cfsetospeed(&newtio, B9600);  
        break;  
    case 115200:  
        cfsetispeed(&newtio, B115200);  
        cfsetospeed(&newtio, B115200);  
        break;  
    case 230400:  
        cfsetispeed(&newtio, B230400);  
        cfsetospeed(&newtio, B230400);  
        break; 
    default:  
        cfsetispeed(&newtio, B9600);  
        cfsetospeed(&newtio, B9600);  
        break;  
    }  
    if (nStop == 1)  
    {  
        newtio.c_cflag &= ~CSTOPB;  
    }  
    else if (nStop == 2)  
    {  
        newtio.c_cflag |= CSTOPB;  
    }  
    newtio.c_cc[VTIME] = 0;  
    newtio.c_cc[VMIN] = 0;  
    tcflush(fd, TCIFLUSH);  
    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)  
    {  
        perror("com set error");  
        return -1;  
    }  
    printf("set done!\n");  
    return 0;  
}

int Serial::readDataTty(int fd, uint8_t *rcv_buf, int TimeOut, int Len)  
{  
    int retval;  
    fd_set rfds;  
    struct timeval tv;  
    int ret, pos;  
    tv.tv_sec = TimeOut / 1000;  //set the rcv wait time    
    tv.tv_usec = TimeOut % 1000 * 1000;  //100000us = 0.1s    

    pos = 0;  
    while (1)  
    {  
        FD_ZERO(&rfds);  
        FD_SET(fd, &rfds);  
        retval = select(fd + 1, &rfds, NULL, NULL, &tv);  
        if (retval == -1)  
        {  
            perror("select()");  
            break;  
        }  
        else if (retval)  
        {  
            ret = read(fd, rcv_buf + pos, 1);  
            if (-1 == ret)  
            {  
                break;  
            }  

            pos++;  
            if (Len <= pos)  
            {  
                break;  
            }  
        }  
        else  
        {  
            break;  
        }  
    }  

    return pos;  
}  

int Serial::sendDataTty(int fd, uint8_t *send_buf, int Len)  
{  
    ssize_t ret;  

    ret = write(fd, send_buf, Len);  
    if (ret == -1)  
    {  
        printf("serial write device error\n");  
    }

    return ret;  
}

int Serial::set_serial(int port)
{
    //openPort  
    if ((fdSerial = openPort(fdSerial, port))<0)//1--"/dev/ttyS0",2--"/dev/ttyS1",3--"/dev/ttyS2",4--"/dev/ttyUSB0" 小电脑上是2--"/dev/ttyS1"  
    {  
        perror("open_port error");  
        return -1;  
    }

    if ((iSetOpt = setOpt(fdSerial, 230400, 8, 'N', 1))<0)  
    {  
        perror("set_opt error");  
        return -1;  
    }  

    /*if (port == 1 || port == 2)
    {
        if ((iSetOpt = setOpt(fdSerial, 230400, 8, 'N', 1))<0)  
        {  
            perror("set_opt error");  
            return -1;  
        }  
    }
    else if (port == 0)  
    {
            if ((iSetOpt = setOpt(fdSerial, 230400, 8, 'N', 1))<0)  
        {  
            perror("set_opt error");  
            return -1;  
        }  
    }*/

    printf("Serial fdSerial=%d\n", fdSerial);  

    tcflush(fdSerial, TCIOFLUSH);//清掉串口缓存  
    fcntl(fdSerial, F_SETFL, 0);

    this->m_RunStatus = 0;
}

int Serial::serial_send(uint8_t* buffSenData, unsigned int sendDataNum)
{
    return sendDataTty(fdSerial, buffSenData, sendDataNum); 
}

int Serial::serial_recieve(uint8_t* buffRcvData)
{ 
    //读取1024字节数据到bufferRcvData，超时时间设置为2ms
    return readDataTty(fdSerial, buffRcvData, 2, 1024);
    // std::cout <<  int(buffRcvData[0]) << " " <<  int(buffRcvData[1]) << " "  <<  int(buffRcvData[2]) << " " <<  int(buffRcvData[3]) << " " 
    //         <<  int(buffRcvData[4]) << " " <<  int(buffRcvData[5]) << " " <<  int(buffRcvData[6]) << std::endl; 
}


//从客户端接收数据执行体
void Serial::OnReceive()
{
    uint8_t buffRcvData_servo[1024] = {0};
    while(this->m_RunStatus == 1)
    {
        //从伺服接收数据
        int retLen = this->serial_recieve(buffRcvData_servo);
        if(retLen > 0)
        {
            std::cout<<"serial received "<<std::dec<<retLen<<"bytes"<<std::endl;
            for(int i=0; i< retLen ;i++)
            {
                printf("[%02X]", buffRcvData_servo[i]);
            }
            std::cout<<std::endl<<std::endl;

            //check frame header
            if(!(buffRcvData_servo[0] == 0x55 && buffRcvData_servo[1] == 0xaa && buffRcvData_servo[2] == 0xdc))
            {
                printf("frame header error, drop data\n");
                memset(buffRcvData_servo,0,1024);
                continue;
            }
            //checksum
            uint8_t checksum = viewlink_protocal_checksum(buffRcvData_servo);
            if(checksum != buffRcvData_servo[retLen - 1])
            {
                printf("frame checksum error, drop data\n");
                memset(buffRcvData_servo,0,1024);
                continue;
            }

            VL_ParseSerialData(buffRcvData_servo);

            //process data


        }
    }
}

int Serial::OnStart()
{
    this->SetStatus(1);
    // //创建接收消息的线程
    // ret = pthread_create(&this->p_Recv, NULL, OnMQReceive, this);
    // if(ret != 0)
    // {
    // 	perror("create OnMQReceive thread fail\n");
    // 	return -1;
    // }
    // else
    // {
    // 	printf("create OnMQReceive thread succeed\n");
    // }

    this->r_thread = std::thread(&Serial::OnReceive, this);
    r_thread.detach();//主线程与子线程分离，保证主线程结束不影响子线程，确保子线程在后台运行
    printf("Serial recv thread started\n");
    return this->m_RunStatus;
}


const int Serial::SetStatus(const int status)
{
    this->m_RunStatus = status;
    return 0;
}

const int Serial::GetStatus() const
{
    return this->m_RunStatus;
}


//关闭串口
int Serial::closePort(int fd)
{
    close(fd);

    return 0;
}


