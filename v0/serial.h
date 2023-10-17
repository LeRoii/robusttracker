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

enum EN_SERVO_CTRL_MODE
{
    MotorSwitch = 0,
    SpeedMode = 1,
    FollowCurrGeographicLocationByServoCtrl = 2, // 暂不支持
    FollowNose = 3,  // follow yaw
    HomePosition = 4,
    AzimuthScan = 5, // 暂不支持
    TrackMode = 6,
    PitchScan = 7,  // 暂不支持
    FixedPointFollowUp = 8, // 指向经纬度，暂不支持
    RelativeAngleMode = 9, // 当前位置为零点
    LockNoseMode = 0xA, // follow yaw disable
    AbsAngleMode = 0xB, // 回中位置为零点
    FollowUpSpaceAngle = 0xC, // 暂不支持
    RCMode = 0xD,
    PointingMovement = 0xE, 
    MeaningLessPara = 0xF,
    GyroAzimuthZerDriftManualAdjustment = 0x10, // 陀螺仪航向校正
    OnlyFrameAngleCompensationServoSys = 0x11, // 框架航向角
    PitchVerticalDown = 0x12,
    OnlyFrameSpeedCompensationServoSys = 0x13,
    FrameAngleAndSpeedCompensatingServoSys = 0x14,
    DisableFrameCompensationServoSys = 0x15,
    HeadingBackToCenter = 0x16,
    PitchLevel = 0x17,
    AbsAngleModeWith720 = 0x1B, // 回中位置为零点
    ServoCtrlButt
};

// 伺服控制常用
struct ST_A1_CONFIG
{
    uint8_t enServoCtrlMode; // @ref EN_SERVO_CTRL_MODE
    uint8_t para1[2];
    uint8_t para2[2];
    uint8_t para3[2];
    uint8_t para4[2];
};

enum EN_SERVO_OP_MODE
{
    NoneServoOpMode = 0,
    PosCalculationPitchAngleErrorAdjustManually = 1,
    PosCalculationHeadingAngleErrorAdjustManually = 2,
    Rsvd = 0x1E,
    NotCommonlyUsedCtrlCmd = 0x1F,
    ServoOpButt
};

enum EN_UNUSED_STATE_RETURN_CTRL_MODE
{
    NotReturnUnusedFrame = 0,
    ReturnUnusedFrame = 1,
    UnusedStateReturnCtrlButt
};

// 伺服控制不常用
struct ST_A2_CONFIG
{
    uint8_t enServoOpMode : 5; // @ref EN_SERVO_OP_MODE
    uint8_t enUnuseStateReturnCtrlMode : 1; // @ref EN_UNUSED_STATE_RETURN_CTRL_MODE
    uint8_t u2UnusedFrameCounter : 2;
    int8_t adjustmentAmount;
};

enum EN_DISP_MODE
{
    NoneDispMode = 0,
    Vision = 1,
    Ir = 2,
    VisIrPip = 3,
    IrVisPip = 4,
    Fusion = 5,
    Ir1 = 6,
    Ir2 = 7,
    DispModeButt
};

enum EN_OPERATION_CMD1
{
    NoneOpCmd1 = 0x0,
    ZoomStop = 0x01,
    BrightnessInc = 0x02,
    BrightnessDec = 0x03,
    ContrastInc = 0x04,
    ContrastDec = 0x05,
    ApertureInc = 0x06,
    ApertureDec = 0x07,
    FovInc = 0x08,
    FovDec = 0x09,
    FocusInc = 0x0A,
    FocusDec = 0x0B,
    IntNUC = 0x0C,
    ExttNUC = 0x0D,
    IrWhite = 0x0E,
    IrBlack = 0x0F,
    GainInc = 0x10,
    GainDec = 0x11,
    IrRainbow = 0x12,
    ScreenShoot = 0x13,
    RecordStart = 0x14,
    RecordEnd = 0x15,
    ScreenShootMode = 0x16,
    RecordMode = 0x17,
    ToggleScreenShootRecord = 0x18,
    AutoFocus = 0x19,
    ManualFocus = 0x1A,
    IrZoomInc = 0x1B,
    IrZoomDec = 0x1C,
    FormatSDCard = 0x1D,
    QuirySDStatus = 0x1E,
    QuirySDCapacity = 0x1F,
    QuirySDRemainCapacity = 0x20,
    IrColorExt1 = 0x21,
    IrColorExt2 = 0x22,
    IrColorExt3 = 0x23,
    IrColorExt4 = 0x24,
    IrColorExt5 = 0x25,
    IrColorExt6 = 0x26,
    VEOn = 0x27,
    VEOff = 0x28,
    WDOn = 0x29,
    WDOff = 0x2A,
    NROn = 0x2B,
    NROff = 0x2C,
    SpotAEOn = 0x2D,
    SpotAEOff = 0x2E,
    SpotFocusOn = 0x2F,
    SpotFocusOff = 0x30,
    IrOriShoot = 0x33,
    IrOriRecordStart = 0x34,
    IrOriRecordEnd = 0x55,
    OperationButt
};

struct ST_C1_CONFIG
{
    uint16_t enDispMode : 3;
    uint16_t enOpCmd1Para : 3;
    uint16_t enOpCmd1 : 7;
    uint16_t laserCmd : 3;
};

enum EN_C2_OP_CMD1
{
    NoneC2OpCmd1 = 0,
    TimingNonUniformCorrectionOn = 4, // 保留
    CancelTimedNonUniformCorrection = 5, // 保留
    VisiblePhotoElectronAmplificationOn = 6,
    VisiblePhotoElectronAmplificationOff = 7,
    VisiblePhotoElectronMagnificationMaximumSetting = 8,
    ForcedOpenThermalImageShutter = 9,
    ThermalImageShutterForcedClosed = 0xA,
    EnableThermalImageAutoProtectfunc = 0xB,
    TurnOffThermalImageAutoProtectFunc = 0xC,
    DisplayThermalImageProtectOSD = 0xD,
    NotDisplayThermalImageProtectOSD = 0xE,
    ImageEnhancementOn = 0x10,
    ImageEnhancementOff = 0x11,
    ThermalImageColorCodeBarOpen = 0x12,
    ThermalImageColorCodeBarOff = 0x13,
    InfraredAlarmThresholdSetting = 0x14,
    VisibleLightImageUprightInverted = 0x15,
    DemistOff = 0x16,
    DemistOn = 0x17,
    SensorComesWithCharacterOn = 0x18, // OSD开
    SensorComesWithCharacterOff = 0x19, // OSD关
    ThermalImagingImageUpright = 0x1A,
    ThermalImagingImageInverted = 0x1B,
    AutoBrightnessOn = 0x1C,
    OutputTemperatureMeasureDataOnceSendOnceOutput = 0x1D,
    OutputTemperatureMeasureDataFrameIntervalPeriodAsParamOfThisPackage = 0x1E,
    StopCycleOutputTemperatureMeasureData = 0x1F,
    FocusingFullyAutomatic = 0x28,
    FocusStopChange = 0x2B, // 保留
    VisibleLightNearInfraredModeOn = 0x4A,
    VisibleLightNearInfraredModeOff = 0x4B,
    RebootOrSelfTest = 0x4E,
    BrightnessSetToParameterValueOfThisPackageInstruction = 0x50,
    ContrastSetToParameterValueOfThisPackageInstruction = 0x51,
    ApertureSetToParameterValueOfThisPackageInstruction = 0x52,
    SetTheVisibleLightMultiToParameterValueOfThisPackageInstruction = 0x53,
    FocusingSetToParameterValueOfThisPackageInstruction = 0x54,
    GainOrISOSetToParameterValueOfThisPackageInstruction = 0x55,
    SetThermalImageElectronicAmplificationToParameterValueOfThisPackageCmd = 0x56,
    LaserCtrlCmd = 0x74,
    OpticalPowerCtrlCmd = 0x75, // 保留
    AutoVideoPackageTimeSetting = 0x76,
};

// 光学控制不常用
struct ST_C2_CONFIG
{
    uint8_t opCmd1; // @ref EN_C2_OP_CMD1
    uint8_t opCmdPara1[2];
};

enum EN_TRACK_SOURCE_MODE
{
    NoneTrackSource = 0,
    VisibleLight1 = 1,
    ThermalImager = 2,
    VisibleLight2 = 3,
    TrackSourceButt
};

enum EN_BASE_OP_MODE
{
    NoneBaseOp = 0,
    Stop = 1,
    Search = 2,
    OnTrack = 3,
    ModifyTrackPointToCurrSmallCross = 4,
    AiIdentifySwitch = 5,
    EnableAILinkageOpticalMagnification = 6,
    DisableAILinkageOpticalMagnification = 7,
    AutoTransferToTrackingAfterAIRecognition = 8,
    AIPointSelectTrackingTargetAutoJumpsToRecogTarget = 9,
    AIPointSelectTrackingTargetNotAutoJumpsToRecogTarget = 0xA,
    TrackingSpeedAdjustment = 0xB,
    SixteenSquareExtraSmallTemplates = 0x20,
    ThirtyTwoSquareSmallTemplates = 0x21,
    SixtyFourSquareMidTemplates = 0x22,
    OneHundredAndTwentyEightSquareBigTemplates = 0x23,
    SmallMidAdaptiveTemplates = 0x24,
    SmallBigAdaptiveTemplates = 0x25,
    MidBigAdaptiveTemplates = 0x26,
    SmallMidBigAdaptiveTemplates = 0x28,
    BaseOpButt
};

//  跟踪器指令常用
struct ST_E1_CONFIG
{
    uint8_t enTrackSourceMode : 3; // @ref EN_TRACK_SOURCE_MODE
    uint8_t u8Para1 : 5;
    uint8_t enBaseOpMode; // @ref EN_BASE_OP_MODE
    uint8_t u8Para2;
};

enum EN_TRACKER_CONTENT_AND_TRACK_CTRL
{
    NoneE2 = 0,
    OSDSettingInstruction = 1,
    TracePointTransferredToInstructionPos = 0xA,
    TrackCoordinatePointSettingOfUpperLeftCornerOfRectangularArea = 0xB,
    TrackCoordinatePointSettingOfBottomLeftCornerOfRectangularArea = 0xC,
    VisibleLightFocusToSpecifiedPos = 0xD,
    VisibleLightExposureToSpecifiedPos = 0xE,
    SetOSDColor = 0x1F,
    EnableRecogInformationOutput = 0x20,
    TurnOffRecogInformationOutput = 0x21,
    EnableRecogInformationOutputForInternalAnalysis = 0x22,
    TurnOffRecogInformationOutputForInternalAnalysis = 0x23,
};

//  跟踪器指令不常用
struct ST_E2_CONFIG
{
    uint8_t enExtendCmd1; // @ref EN_TRACKER_CONTENT_AND_TRACK_CTRL
    uint8_t para1[2];
    uint8_t para2[2];
};

enum EN_CALC_CTRL_MODE
{
    NoneCalcCtrlCmd = 0,
    PodTowardsTargetWithGivenLatitudeAndLongitude = 1,
    AirToGroundManually = 2, // 暂不支持
    FollowCurrGeographicLocation = 3, // 暂不支持
    FollowUpSpaceAngleByCalcCtrl = 4,
    CalcPointCalibrationToGivenLatitudeAndLongitudePoint = 5,
};

struct ST_S1_CONFIG
{
    uint8_t enCalcCtrlMode; // @ref EN_CALC_CTRL_MODE
    uint8_t para1;
    uint8_t para2[12];
};

enum EN_S2_CONFIG_CMD
{
    NoneS2CfgCmd = 0,
    SetTheCurrAltitudeToTakeOffAltitude = 1, // 保留
    GivesTheCurrTargetCoordinateAltitude = 2, // 保留
    SaveCurrTargetAltitude = 3, // 保留
    OpenTheOutputT2PackageToThePodMeshPort = 4,
    CloseTheOutputT2PackageToThePodMeshPort = 5,
    OpenTheOutputT2PackageToThePodSerialPort = 6,
    CloseTheOutputT2PackageToThePodSerialPort = 7,
};

// TGCC控制不常用
struct ST_S2_CONFIG
{
    uint8_t enCfgCmd; // @ref EN_S2_CONFIG_CMD
    uint8_t para[4];
};

enum EN_U_OP_CMD
{
    NoneUOpCmd = 0,
    ProtocolCtrl = 2,
    SetTimeZone = 4,
    SetOSD_1 = 5,
    SetOSD_2 = 7,
    SetSerialPortBaudRate = 8,
    SetThermalImageAlarmTemperature = 0xA,
    SetRemoteCtrlChannelMapping = 0x10,
    QueryProtocolCtrlSettings = 0x82,
    QueryTimeZoneSettings = 0x84,
    QueryOSDSettings = 0x85,
    QuerySerialPortBaudRateSettings = 0x88,
    QueryThermalImageAlarmTemperatureSettings = 0x8A,
    QueryRemoteCtrlChannelMappingSettings = 0x90,
    QueryDeviceFirmwareVersionNumber = 0xD0,
    QueryDeviceModel = 0xE4,
    QueryDeviceSerialNumber = 0xEE,
    QueryImageBoardIDNumber = 0xEF,
    AdjustableVariableSetting = 0xF1, // 掉电不保存
    AdjustableVariableSettingStatusQuery = 0xF2,
    AdjustableVariableSettingByPowerOffSave = 0xF3, // 掉电保存
    InitialPositionSettingsForPodOrientationPitchAndRollStartup = 0xF4,
};

struct ST_U_CONFIG
{
    uint8_t enOpCmd; // @ref EN_U_OP_CMD
    uint8_t para[9];
};

struct ST_A1C1E1_CONFIG
{
    ST_A1_CONFIG a1Config;
    ST_C1_CONFIG c1Config;
    ST_E1_CONFIG e1Config;
};

struct ST_A2C2E2_CONFIG
{
    ST_A2_CONFIG a2Config;
    ST_C2_CONFIG c2Config;
    ST_E2_CONFIG e2Config;
};

struct ST_A1C1E1S1_CONFIG
{
    ST_A1_CONFIG a1Config;
    ST_C1_CONFIG c1Config;
    ST_E1_CONFIG e1Config;
    ST_S1_CONFIG s1Config;
};

struct ST_A2C2E2S2_CONFIG
{
    ST_A2_CONFIG a2Config;
    ST_C2_CONFIG c2Config;
    ST_E2_CONFIG e2Config;
    ST_S2_CONFIG s2Config;
};



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
    
    std::thread r_thread;
    //std::thread s_thread;
    //std::thread a_thread;


private:
    int iSetOpt;//SetOpt 的增量i  
    int fdSerial; 

    int m_RunStatus;//-1-未初始化，0-就绪，1-运行
    //针对IPC回调函数
    // SERIAL_CALLBACK_FUNC m_Send_Data_Func;
};


#endif