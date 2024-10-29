#ifndef VIDEOPRO_MSGTABLE_H
#define VIDEOPRO_MSGTABLE_H

#define  HEAD1  0X55
#define  HEAD2  0XAA
#define  HEAD3  0XDC
#pragma pack(push)
#pragma pack(1)
/*支持解析的文件类别*/
typedef enum
{
    XJ_DIRECTION =	0x61,
    XJ_YR	=0x62,
    XJ_ROLL=	0x63,
    XJ_OSD	=0x64,
    XJ_CAMCTRL=	0x65,
    XJ_RANGE	=0x66,


    /***上行数据***/
    XJ_TRACK = 0x81,
    XJ_RANGEV = 0x82,
    XJ_FRAMEANG = 0x83,

}EN_XJ_MSGTYPE_T;


/*私有消息定义*/
struct xj_net_msg_hdr {
    uint8_t head1; 
    uint8_t head2;
    uint8_t head3;
    uint8_t lLen;
    unsigned char content[0];
};

/* 上报跟踪到上位机  0x81*/
typedef struct ST_missdistancereport
{
    struct xj_net_msg_hdr head;
    uint8_t msg_type;
    short uMissDistanceX; 
    short uMissDistanceY; 
    uint8_t  crc;
}ST_MissDistanceReport;

/*上报跟踪到上位机  0x82*/
typedef struct ST_rangev
{
    struct   xj_net_msg_hdr head;
    uint8_t msg_type;
    short    distance; 
    uint8_t  reserve[2]; 
    uint8_t  crc;
}ST_Rangev;


/* 接收伺服控制    0x61*/
typedef struct ST_servodirection
{
    struct xj_net_msg_hdr head;
     uint8_t msg_type;
    uint8_t lcmd;  
    uint8_t lres; 
    uint8_t fcrc;
}ST_ServoDirection;

/* 接收伺服控制方位  */
typedef struct ST_servodirection_yaw
{
    struct xj_net_msg_hdr head;
    uint8_t msg_type;
    short  lyaw;  
    uint8_t  crc;
}ST_ServoDirectionYaw;

/* 接收伺服控制俯仰*/
typedef struct ST_servodirection_pitch
{
    struct xj_net_msg_hdr head;
    uint8_t msg_type;
    short  lpitch;  
    uint8_t  crc;
}ST_ServoDirectionPitch;


/* 接收伺服控制横滚*/
typedef struct ST_servodirection_roll
{
    struct xj_net_msg_hdr head;
    uint8_t msg_type;
    short  lroll;  
    uint8_t  crc;
}ST_ServoDirectionRoll;


/* 接收伺服控制横滚*/
typedef struct ST_video_osdcontrol
{
    struct xj_net_msg_hdr head;
    uint8_t msg_type;
    uint8_t  isOsdenable;
    uint8_t  reserve;
    uint8_t  crc;
}ST_VideoOsdControl;

/* 相机控制  0x66*/
typedef struct ST_video_camctrl 
{
    struct xj_net_msg_hdr head;
    uint8_t msg_type;
    uint8_t  isIRinit;
    uint8_t  reserve;
    uint8_t  crc;
}ST_VideoCamCtrl;

/* 相机控制  0x67*/
typedef struct ST_video_range 
{
    struct xj_net_msg_hdr head;
        uint8_t msg_type;
    uint8_t  isRanging; //是否测距
    uint8_t  reserve;
    uint8_t  crc;
}ST_VideoRangeCtrl;

/* 相机控制  0x68*/
typedef struct ST_video_SuperResolution 
{
    struct xj_net_msg_hdr head;
    uint8_t msg_type;
    uint8_t  imagetype; //0 红外超分 1 可见光超分
    uint8_t    index;
    uint8_t  crc;
}ST_VideoSuperResolution ;

/* 与电机交互*/
/**随动命令*/
typedef struct ST_servodirection_FollowYawPitch
{
    uint8_t head; 
    uint8_t type; 
    short  lyaw; 
    short  lpitch;  
    uint8_t  crc;
}ST_ServoDirectionFollowYawPitch;

typedef struct ST_servodirection_FollowRoll
{
    uint8_t head; 
    uint8_t type; 
    short  lRoll; 
    short  lres;  
    uint8_t  crc;
}ST_ServoDirectionFollowRoll;

/* 与上位机交互*/
/* 相机控制  0x83*/
typedef struct ST_video_Frameang  
{
    struct xj_net_msg_hdr head;
    uint8_t msg_type;
    short   lyaw; //0 红外超分 1 可见光超分
    short   lpitch;
    short   lRoll; 
    short   Azimuth_AttitudeAngle   ;
    short  Angleofpitch_AttitudeAngle;
    short Roll_AttitudeAngle;
    uint8_t  crc;
}ST_VideoFrameang ;




/* 相机控制  0x67*/
typedef struct ST_Video_Enhance 
{
    struct xj_net_msg_hdr head;
    uint8_t  msg_type;
    uint8_t  type; //类型 0 对比度增强 1 细节增强  2 线性变换
    uint8_t  state; //0  增强关  1  增强开
    float fZ_upperThreshold_;
    float fZ_lowerThreshold_;
    float fZ_k1_;
    float fZ_k2_;
    float fZ_k3_;
    uint8_t  crc;
}ST_VideoEnhance;


#pragma pack(pop)

uint8_t project137_serial_checksum(uint8_t *buf,uint8_t len);
void   project137_ParseSerialData(uint8_t *buf);





void project173_SendFollowYawPitch(short lyaw,short lpitch);
void project173_SendFollowRoll(short lRoll);


/****************************************************************************************
 * 函 数 名 ： VideoPro_Send_Frameang
 * 功    能 ： 发送框架角到上位机
 ***************************************************************************************/
uint8_t VideoPro_Send_Frameang();
/****************************************************************************************
 * 函 数 名 ： project137_CmdRange
 * 功    能 ： 发送测距命令
 ***************************************************************************************/
void project137_CmdRange();
#endif	// VIDEOPRO_MSGTABLE_H
