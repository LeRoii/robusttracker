#ifndef GUIDESCUSB2_H
#define GUIDESCUSB2_H

#ifdef _WIN32
#define dllexport __declspec(dllexport)
#define stdcall _stdcall
#else
#define dllexport __attribute__ ((visibility("default")))
#define stdcall
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    LOG_LEVEL_NONE = 0,
    LOG_LEVEL_FATAL,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_WARN,
    LOG_LEVEL_INFO,
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_VERBOSE
}guide_usb_log_level_e;

typedef enum
{
    YUV = 0,
    YUV_PARAM = 1,
    Y16 = 2,
    Y16_PARAM = 3,
    Y16_YUV = 4,
    Y16_PARAM_YUV = 5,
    X16 = 6,
    X16_PARAM = 7,
    TMP = 8,
    TMP_PARAM = 9,
    TMP_YUV = 10,
    TMP_PARAM_YUV = 11
}guide_usb_video_mode_e;

typedef enum
{
    DEVICE_CONNECT_OK = 1,
    DEVICE_DISCONNECT_OK = -1
}guide_usb_device_status_e;

typedef enum
{
    GSDK_SUCCESS                             =  0,
    GSDK_ERROR_USB_INIT                      = -1,
    GSDK_ERROR_USB_DESCRIPTOR                = -2,
    GSDK_ERROR_USB_OPEN                      = -3,
    GSDK_ERROR_USB_DETACH_KERNEL             = -4,
    GSDK_ERROR_USB_CLAIM_INTERFACE           = -5,
    GSDK_ERROR_INIT_QUEUE                    = -6,
    GSDK_ERROR_SERIAL_THREAD_CREATE          = -7,
    GSDK_ERROR_RECEIVE_THREAD_CREATE         = -8,
    GSDK_ERROR_DEAL_THREAD_CREATE            = -9,
    GSDK_ERROR_SEND_COMMAND                  = -10,
    GSDK_ERROR_SEND_COMMAND_TIMEOUT          = -11,
    GSDK_ERROR_UPGRADE_VIDEO_ON              = -12,
    GSDK_ERROR_SERIAL_THREAD_EXIT            = -13,
    GSDK_ERROR_RELEASE_INTERFACE             = -14,
    GSDK_ERROR_FILE_NO_EXIST                 = -15,
    GSDK_ERROR_FILE_FORMAT                   = -16,
    GSDK_ERROR_FILE_OVER_SIZE                = -17,
    GSDK_ERROR_POINTER_NULL                  = -18,
    GSDK_ERROR_LOG_LEVEL                     = -19,
    GSDK_ERROR_VIDEO_MODE                    = -20,
    GSDK_ERROR_NO_DEVICES                    = -21,
    GSDK_ERROR_COMMUNICATION_DISENABLE       = -22,
    GSDK_ERROR_DEVICE_ID                     = -23

}gsdk_usb_ret_code_e;


typedef struct
{
    int width;                              //图像宽度
    int height;                             //图像高度
    guide_usb_video_mode_e video_mode;      //视频模式
}guide_usb_device_info_t;

typedef struct
{
    int frame_width;
    int frame_height;
    short* frame_src_data;
    int frame_src_data_length;
    short* frame_yuv_data;
    int frame_yuv_data_length;
    short* paramLine;
    int paramLine_length;
}guide_usb_frame_data_t;

typedef struct
{
    unsigned char* serial_recv_data;
    int serial_recv_data_length;
}guide_usb_serial_data_t;


typedef int (stdcall *OnDeviceConnectStatusCB)(int id, guide_usb_device_status_e deviceStatus);
typedef int (stdcall *OnFrameDataReceivedCB)(int id, guide_usb_frame_data_t *pVideoData);
typedef int (stdcall *OnSerialDataReceivedCB)(int id, guide_usb_serial_data_t *pSerialData);

dllexport int guide_usb_get_devcount(void);
dllexport int guide_usb_initial(int id);
dllexport int guide_usb_exit(int id);
dllexport int guide_usb_openstream(int id, guide_usb_device_info_t* deviceInfo,OnFrameDataReceivedCB frameRecvCB,OnDeviceConnectStatusCB connectStatusCB);//连接设备
dllexport int guide_usb_closestream(int id);
dllexport int guide_usb_opencommandcontrol(int id, OnSerialDataReceivedCB serialRecvCB);
dllexport int guide_usb_closecommandcontrol(int id);
dllexport int guide_usb_sendcommand(int id,unsigned char* cmd, int length);

dllexport int guide_usb_setloglevel(guide_usb_log_level_e level);
dllexport int guide_usb_upgrade(int id,const char* file);
dllexport int guide_usb_upgradecolor(int id,const char* file);//升级color
dllexport int guide_usb_upgradecurve(int id,const char* file);//升级curve

#ifdef __cplusplus
}
#endif


#endif // GUIDESCUSB2_H
