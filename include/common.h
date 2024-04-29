#ifndef _COMMON_H_
#define _COMMON_H_

#include <opencv2/opencv.hpp>
#include <sys/time.h>
#include <dirent.h>

#include "rknn_api.h"

#define OBJ_NAME_MAX_SIZE 16
#define OBJ_NUMB_MAX_SIZE 200
#define MAX_OUTPUTS 3

const int RET_OK = 1;
const int RET_ERR = 0;

struct bbox_t
{
    unsigned int x, y, w, h;     // (x,y) - top-left corner, (w, h) - width & height of bounded box
    float prop;                  // confidence - probability that the object was found correctly
    unsigned int obj_id;         // class of object - from range [0, classes-1]
    unsigned int track_id;       // tracking id for video (0 - untracked, 1 - inf - tracked object)
    unsigned int frames_counter; // counter of frames on which the object was detected
    float x_3d, y_3d, z_3d;      // center of object (in Meters) if ZED 3D Camera is used
    bbox_t(unsigned int xx, unsigned int yy, unsigned int ww, unsigned int hh, unsigned int cls, unsigned int id, float conf) : x(xx), y(yy), w(ww), h(hh), obj_id(cls), track_id(id), prop(conf){};
    bbox_t() {}
};

typedef struct
{
    int id;
    int count;
    bbox_t results[OBJ_NUMB_MAX_SIZE];
} object_detect_result_list;

typedef struct
{
    rknn_context rknn_ctx;
    rknn_input_output_num io_num;
    rknn_tensor_attr *input_attrs;
    rknn_tensor_attr *output_attrs;
    int model_channel;
    int model_width;
    int model_height;
    bool is_quant;
} rknn_app_context_t;

inline double __get_us(struct timeval t) { return (t.tv_sec * 1000000 + t.tv_usec); }

enum EN_SERVO_CTRL_MODE
{
    MotorSwitch = 0,
    SpeedMode = 1,
    FollowCurrGeographicLocationByServoCtrl = 2, // 暂不支持
    FollowNose = 3,                              // follow yaw
    HomePosition = 4,
    AzimuthScan = 5, // 暂不支持
    TrackMode = 6,
    PitchScan = 7,            // 暂不支持
    FixedPointFollowUp = 8,   // 指向经纬度，暂不支持
    RelativeAngleMode = 9,    // 当前位置为零点
    LockNoseMode = 0xA,       // follow yaw disable
    AbsAngleMode = 0xB,       // 回中位置为零点
    FollowUpSpaceAngle = 0xC, // 暂不支持
    RCMode = 0xD,
    PointingMovement = 0xE,
    MeaningLessPara = 0xF,
    GyroAzimuthZerDriftManualAdjustment = 0x10, // 陀螺仪航向校正
    OnlyFrameAngleCompensationServoSys = 0x11,  // 框架航向角
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
struct ST_A1_CONFIG // 9bytes
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
struct ST_A2_CONFIG // 2Bytes
{
    uint8_t enServoOpMode : 5;              // @ref EN_SERVO_OP_MODE
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

// struct ST_C1_CONFIG //2bytes
// {
//     uint16_t laserCmd : 3;
//     uint16_t enOpCmd1 : 7;
//     uint16_t enOpCmd1Para : 3;
//     uint16_t enDispMode : 3;
// };

struct ST_C1_CONFIG // 2bytes
{
    uint16_t enDispMode : 3;
    uint16_t enOpCmd1Para : 3;
    uint16_t enOpCmd1 : 7;
    uint16_t laserCmd : 3;
};

enum EN_C2_OP_CMD1 // 3bytes
{
    NoneC2OpCmd1 = 0,
    TimingNonUniformCorrectionOn = 4,    // 保留
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
    SensorComesWithCharacterOn = 0x18,  // OSD开
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
struct ST_C2_CONFIG // 3bytes
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
struct ST_E1_CONFIG // 3bytes
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
struct ST_E2_CONFIG // 5bytes
{
    uint8_t enExtendCmd1; // @ref EN_TRACKER_CONTENT_AND_TRACK_CTRL
    uint8_t para1[2];
    uint8_t para2[2];
};

enum EN_CALC_CTRL_MODE
{
    NoneCalcCtrlCmd = 0,
    PodTowardsTargetWithGivenLatitudeAndLongitude = 1,
    AirToGroundManually = 2,          // 暂不支持
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
    SetTheCurrAltitudeToTakeOffAltitude = 1,  // 保留
    GivesTheCurrTargetCoordinateAltitude = 2, // 保留
    SaveCurrTargetAltitude = 3,               // 保留
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

enum EN_DATA_FRAME_TYPE
{
    TypeUnkonwn = 0,
    HandShake = 1,
    HeartBeat = 2,
    HeartBeat15 = 3,
    Status40 = 4,
    Status41 = 5,
    FrameS2 = 6,
    HeartBeat14 = 7,
    CtrlSdCmd = 8,
    HeartBeat12 = 9,
    FrameE2 = 10,
    Status42 = 11,
    IPInq = 12,
};

enum EN_ONCE_SEND_DOWN_TO_UP_TYPE
{
    SdCmdReply = 0,
};

struct ST_COORDINATE_CONFIG
{
    double longitude;
    double latitude;
    int16_t altitude;
};

enum class EN_IRIMG_MODE
{
    WHITEHOT = 0,
    BLACKHOT = 1,
    PSEUDOCOLOR = 2,
};

enum class EN_SCREEN_OP_MODE
{
    SCREEN_NONE = 0,
    SCREEN_SHOOT = 1,
    RECORDING_START = 2,
    RECORDING_END = 3,
};

struct OSD_SET1_CTRL
{
    bool enOSDShow;
    bool enCrossShow;
    bool enAttitudeAngleShow;
    bool enMissDistanceShow;
    bool enACFTGPS1Show;
    bool enTimeShow;
    bool enEOFieldOfViewOrMultiplyShow;
    bool enSmallFontShow;
};

struct OSD_SET2_CTRL
{
    bool enSaveSet;
    bool enIRShow;
    bool enLRFShow;
    bool enGPSIsMGRS;
    bool enTFShow;
    bool enTAGGPSShow;
    bool enMultiplyGreenOrFieldOfViewAngleWhiteShow;
    bool enGPSIsDegMinSecShow;
};

// OSD控制
struct ST_OSD_CONFIG_CONFIG
{
    bool osdSwitch;
    bool crosshairSwitch;
    bool attitudeAngleSwitch;
    bool MissToTargetSwitch;
    bool GPSSwitch;
    bool dateSwitch;
    bool EOwitch;
    bool fontSizeSwitch;
    bool saveConfigSwitch;
    bool IRSwitch;
    bool LRFSwitch;
    bool GPSOrMGRSSwitch;
    bool TFSwitch;
    bool TargetGPSSwitch;
    bool fontColorSwitch;
    bool GPSShowModeSwitch;
};

struct ST_SYS_STATUS
{
    bool trackOn;
    bool trackerInited;
    int trackerGateSize;
    bool detOn;
    EN_DISP_MODE enDispMode;
    double rollAngle;
    double pitchAngle;
    ST_COORDINATE_CONFIG ACFTCoordinate;
    ST_COORDINATE_CONFIG TAGCoordinate;
    double lrfValue;
    double eoValue;
    double fovValue;
    EN_IRIMG_MODE enIrImgMode;
    bool detRetOutput;
    cv::Point trackAssignPoint;
    int trackerInitPt[2]; // x,y -960~960, -540~540
    int trackerArea[4];   // top left x, y, right bottom x, y
    EN_SCREEN_OP_MODE enScreenOpMode;
    OSD_SET1_CTRL osdSet1Ctrl;
    OSD_SET2_CTRL osdSet2Ctrl;
    ST_OSD_CONFIG_CONFIG osdCtrl;
    bool isTSeriesDevice;
    double osdFontSize;
    // 相机变焦参数
    bool isOSDopen;

    int16_t trackMissDistance[2];

    ST_SYS_STATUS() : trackOn(false), trackerInited(false), trackerGateSize(32),
                      detOn(true), enDispMode(Vision), enIrImgMode(EN_IRIMG_MODE::WHITEHOT),
                      enScreenOpMode(EN_SCREEN_OP_MODE::SCREEN_NONE), osdFontSize(0.8){};
};

enum EN_SERVO_STATUS_MODE
{
    ServoStatusMotorSwitch = 0,
    ManualSpeedMode = 1,
    FollowTheCurrGeographicLocation = 2, // 当前不支持
    FollowYaw = 3,
    ServoStatusHomePosition = 4,
    AzimuthScanning = 5, // 当前不支持
    TrackingMode = 6,
    PitchScanning = 7,                 // 当前不支持
    ServoStatusFixedPointFollowUp = 8, // 指向经纬度，暂不支持
    ManualRelativeAngleMode = 9,       // 当前角度为零点
    LockYawMode = 0xA,
    ManualAbsAngleMode = 0xB,            // 回中位置为零点
    ServoStatusFollowUpSpaceAngle = 0xC, // 当前不支持
    ManualRCMode = 0xD,
    MeaninglessServoMode = 0xF,
    ServoStatusButt
};

// 伺服状态常用
struct ST_B1_CONFIG
{
    uint8_t servoStatus : 4; // @ref EN_SERVO_STATUS_MODE
    uint8_t rollAngleH4 : 4; // 1bit=180/4095°
    uint8_t rollAngleL8;     // 1bit=180/4095°，数值0-90对应负90~0（值减 90 得到实际角度）数值90-180对应0~正90度；横滚角度共计12位，高四位在字节1的低四位
    short azimuthAngle;      // 1bit=360/65536°
    short pitchAngle;        // 1bit=360/65536°
};

enum EN_SERVO_ACTION_RESP_MODE
{
    NoneAction = 0,
    PosCalcPitchAngleErrorAdjustManually = 1,
    PosCalcHeadingAngleErrorAdjustManually = 2,
    ZeroDriftAdjust = 8,      // 暂不支持
    ZeroDriftCalibration = 9, // 保留
    Fault = 0xEE,
    ServoActionRespModeButt
};

// 伺服状态不常用
struct ST_B2_CONFIG
{
    uint8_t servoActionResp : 5; // @ref EN_SERVO_ACTION_RESP_MODE
    uint8_t rsvdB2 : 1;
    uint8_t notCommonlyUsedFrameCounter : 2;
    uint16_t posCalcAdjustmentAmount; /* 1bit = 0.001 度，
                                       * 当伺服动作=0x01时，为俯仰角误差量, 正数表示向上偏，负数表示向下偏；
                                       * 当伺服动作=0x02时，为航向角误差量，正数表示向右偏，负数表示向左偏；
                                       */
    short rollAngle;                  // 1bit=360/65536°
    short rollAngleSpeed;             // 1bit=0.01°/S
    short azimuthAngleSpeed;          // 1bit=0.01°/S，
    short pitchAngleSpeed;            // 1bit=0.01°/S，
    uint8_t rsvd;
};

enum EN_CURR_CHOOSE_VIDEO_STREAM
{
    CurrChooseVideoStreamVisibleLight1 = 0,
    SingleThermalImager = 1,
    VisibleLight1AndThermalImagerPicInPic = 2,
    ThermalImagerAndVisibleLight1PicInPic = 3,
    Fusionmode = 4,
    ThermalImager1For1352Model = 5,
    ThermalImager2For1352Model = 6,
    CurrChooseVideoStreamButt
};

enum EN_ELEC_MAGNIFY_MODE
{
    ElecMagnifyX1 = 0,
    ElecMagnifyX2 = 1,
    ElecMagnifyX3 = 2,
    ElecMagnifyX4 = 3,
    ElecMagnifyX5 = 4,
    ElecMagnifyX6 = 5,
    ElecMagnifyX7 = 6,
    ElecMagnifyX8 = 7,
    ElecMagnifyX9 = 8,
    ElecMagnifyX10 = 9,
    ElecMagnifyX11 = 0xA,
    ElecMagnifyX12 = 0xB,
    ElecMagnifyX13 = 0xC,
    ElecMagnifyX14 = 0xD,
    ElecMagnifyX15 = 0xE,
    ElecMagnifyX16 = 0xF,
    ElecMagnifyButt
};

enum EN_INFRARED_GRAYSCALE_MODE
{
    whiteHeat = 0,
    BlackHeat = 1,
    InfraredGrayScaleButt
};

enum EN_RECORD_STATE
{
    StopRecording = 0,
    InRecording = 1,
    PhotographyMode = 2,
    RecordStateButt
};

enum EN_INFRARED_STATE_EXTEND_MODE
{
    GrayscaleMode = 0,
    Rainbow = 1,
    infraredStateExtModeButt
};

// 光学状态常用
struct ST_D1_CONFIG
{
    uint8_t opticalSensor : 3;                         // @ref EN_CURR_CHOOSE_VIDEO_STREAM
    uint8_t thermalImagingElectronicMagnification : 4; // @ref EN_ELEC_MAGNIFY_MODE
    uint8_t whiteHeatOrBlackHeatState : 1;             // @ref EN_INFRARED_GRAYSCALE_MODE
    uint8_t distanceMeasurementReturnValueH;
    uint16_t recordingStatus : 2;                     // @ref EN_RECORD_STATE
    uint16_t infraredStateExt : 4;                    // @ref EN_INFRARED_STATE_EXTEND_MODE
    uint16_t visibleLightElectronicMagnification : 4; // @ref EN_ELEC_MAGNIFY_MODE
    uint16_t rsvdD2 : 6;
    uint16_t distanceMeasurementReturnValueL;     // 1bit 表示 0.1m，全零代表无效，无符号整形
    uint16_t currSensorVertiFieldOfViewAngle;     // 1bit=0.01 度
    uint16_t currSensorHoriFieldOfViewAngle;      // 1bit=0.01 度
    uint16_t currSensorOpticsAmplificationFactor; // 1bit=0.1倍
};

enum EN_DETECTOR_TYPE
{
    DetectorVisibleLight1 = 0,
    DetectorThermalImager = 1,
    DetectorVisibleLight1AndThermalImagerInPic = 2,
    DetectorVisibleLight1AndThermalImagerInPicInPic = 3,
    DetectorVisibleLight2 = 4,
    DetectorButt
};

enum EN_OPTICAL_SENSOR_PIXEL_COUNT
{
    opticalSensorPixelCount1080P = 1,
    opticalSensorPixelCount2K = 2,
    opticalSensorPixelCount4K = 3,
    opticalSensorPixelCount960 = 4,
    opticalSensorPixelCount720 = 5,
    opticalSensorPixelCount640 = 6,
    opticalSensorPixelCountButt
};

// 光学状态不常用
struct ST_D2_CONFIG
{
    uint8_t currDetectorType : 3;                 // @ref EN_DETECTOR_TYPE
    uint8_t equipmentFaultIdentification : 5;     //
    uint8_t isTheCurrDetectorTheMainDetector : 1; // 0：是 1：否
    uint8_t opticalSensorPixelCount : 7;          // @ref EN_OPTICAL_SENSOR_PIXEL_COUNT
    uint8_t extendedParameters;                   // 1: 电子放大
    int8_t extendedParameterValues[2];
};

enum EN_TRACKER_TYPE
{
    TrackerVisibleLight1 = 0,
    TrackerThermalImager = 1,
    TrackerVisibleLight2 = 2,
    TrackerButt
};

// 跟踪器状态常用
struct ST_F1_CONFIG
{
    uint8_t tracker : 3; // @ref EN_TRACKER_TYPE
    uint8_t trackerCurrStatus : 2;
    uint8_t rsvd : 3;
};

// 跟踪器状态不常用
struct ST_F2_CONFIG
{
    short azimuthTargetPixelDifference;
    short pitchTargetPixelDifference;
};

struct ST_TARGET_INFO
{
    uint8_t targetType;
    uint16_t targetId;
    int16_t targetAzimuthCoordinate;
    int16_t targetPitchCoordinate;
    uint16_t targetLength;
    uint16_t targetWidth;
    uint16_t targetDetectionConfidence;
};

// AI 系列识别状态反馈
struct ST_F3_CONFIG
{
    uint8_t targetSum;
    uint8_t totalPacketNum;
    uint8_t currPacketId;
    uint8_t rsvd[5];
    ST_TARGET_INFO targetInfo[16];
};

// 目标距离来源类型
enum EN_TARGET_DISTANCE_SOURCE_TYPE
{
    NoneTargetDisSrcType = 0,
    LaserRangingValue = 1,
    EqualHeightEstimate = 2,
    RF = 3,
    TargetDistanceSrcTypeButt
};

enum EN_GPS_SIGNAL_CAP_STAGE
{
    NoneSignal = 0,
    TimeLocked = 1,
    LockInFor2D = 2,
    LockInFor3D = 3,
    GpsSignalCapStageButt
};

enum EN_N_PACKET_RESP_MODE
{
    GyroOffsetAutoAdjusting = 1,
    GyroOffsetSaving = 2,
    GyroOffsetRecoveredToFactoryDefaultValue = 3,
    Angle0PosOfAHRSAdjusted = 4,
    AHRSAttitudeOffsetSaving = 5,
    AHRSAttitudeOffsetReset = 6,
    CalibratingGyroTemperatureDrift = 7,
    GyroTemperatureDriftCalibratedOver = 8,
    NPacketRespModeButt
};

// TGCC状态常用
struct ST_T1_CONFIG
{
    uint8_t targetDistanceSrcType : 3;      // @ref EN_TARGET_DISTANCE_SOURCE_TYPE
    uint8_t gpsSignalAcquisitionStage : 2;  // @ref EN_GPS_SIGNAL_CAP_STAGE
    uint8_t gpsHorizontalSignalQuality : 3; // rsvd
    uint8_t gpsHeightSignalQuality : 3;     // rsvd
    uint8_t s2PacketInstructionResp : 1;    // 0：未收到 1：响应了 S2 包指令，只持续一帧
    uint8_t nPacketInstructionResp : 4;     // @ref EN_N_PACKET_RESP_MODE
    ST_COORDINATE_CONFIG ACFTCoordinate;
    ST_COORDINATE_CONFIG TAGCoordinate;
};

struct Date
{
    uint16_t day : 5;
    uint16_t month : 4;
    uint16_t year : 7;
};

// TGCC状态不常用
struct ST_T2_CONFIG
{
    uint8_t rsvdT2;
    Date date;
    uint8_t time[3];
    uint16_t gpsHeading;
    uint16_t carrierAttitudeAngleAzimuth;
    uint16_t carrierAttitudeAnglePitch;
    uint16_t carrierAttitudeAngleRoll;
    uint8_t rsvd1T2[3];
};

enum EN_V_CTRL_CMD
{
    NoneVCtrlCmd = 0,
    VCtrlProtocolCtrl = 2,
    TimeZone = 4,
    OSD = 0x85, // 这与协议说明书上为0x5不同，但后面携带的信息确实是OSD设置信息无误
    SerialPortBaudRate = 8,
    ThermalImageAlarmTemperature = 0xA,
    RemoteCtrlChannelMapping = 0x10,
    ImageBoardIDNumber = 0xEF,
    DeviceFirmwareVersionNumber = 0xFC,
    DeviceModel = 0xE4, // 这与协议说明书上为0xFD不同，但后面携带的信息确实是设备型号无误
    EquipmentSerialNumber = 0xFE,
    VCtrlCmdButt
};

struct ST_T1F1B1D1_CONFIG
{
    ST_T1_CONFIG t1Config;
    ST_F1_CONFIG f1Config;
    ST_B1_CONFIG b1Config;
    ST_D1_CONFIG d1Config;
};

struct ST_T2F2B2D2_CONFIG
{
    ST_T2_CONFIG t2Config;
    ST_F2_CONFIG f2Config;
    ST_B2_CONFIG b2Config;
    ST_D2_CONFIG d2Config;
};

// 吊舱配置反馈
struct ST_V_CONFIG
{
    uint8_t ctrlCmd; // @ref EN_V_CTRL_CMD
    uint8_t data[22];
};

enum EN_SD_CTRL_CMD
{
    NoActionForSD = 0,
    FormatSdCard = 0x8A,
    QuerySDCard = 0x8B,
    SDCtrlCmdButt
};

enum EN_SD_QUERY_CMD
{
    InquirySDCardStatus = 2,
    InquirySDCardTotalCapacity = 3,
    InquirySDCardRemainCapacity = 4,
    InquirySDCardRemainPicturesQuantity = 5,
    InquirySDCardRemainVideoTime = 6,
    SDQueryCmdButt
};

struct ST_CMD_SD_CONFIG
{
    uint8_t ctrlCmd; // @ref EN_SD_CTRL_CMD
    uint8_t para;    // @ref EN_SD_QUERY_CMD
};

struct ST_ACK_SD_CONFIG
{
    uint8_t ctrlCmd; // @ref (EN_SD_QUERY_CMD - 1)
    uint8_t ackSDData[4];
};

#endif