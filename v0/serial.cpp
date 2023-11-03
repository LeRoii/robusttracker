#include <iostream>
#include "serial.h"
#include "common.h"
#include<arpa/inet.h>

extern ST_SYS_STATUS stSysStatus;

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
ST_T1F1B1D1_CONFIG stT1F1B1D1Cfg = {0};
ST_T2F2B2D2_CONFIG stT2F2B2D2Cfg = {0};

ST_CMD_SD_CONFIG stCmdSDCfg = {0};
ST_ACK_SD_CONFIG stAckSDCfg = {0};


bool CheckFrameHeader(uint8_t *send_buf, int Len)
{
    if(Len < 3)
        return false;
    if(send_buf[0] == 0x55 && send_buf[1] == 0xaa && send_buf[2] == 0xdc)
	{
        return true;
    }
    else
        return false;
}

EN_DATA_FRAME_TYPE GetFrameType(uint8_t *send_buf, int Len)
{
    if(Len < 5)
        return TypeUnkonwn;

    if(Len == 7)
    {
        if(send_buf[4] == 0x00)
            return HandShake;
        else if(send_buf[4] == 0x15)
            return HeartBeat15;
        else if(send_buf[4] == 0x14)
            return HeartBeat14;
        else if(send_buf[4] == 0x12)
            return HeartBeat12;
    }

    if (Len == 8) {
        if (send_buf[4] == 0x5D) {
            return CtrlSdCmd;
        }
    }

    if(Len > 10)
    {
        if(send_buf[4] == 0x40)
            return Status40;
        else if(send_buf[4] == 0x41)
            return Status41;
        else if(send_buf[4] == 0x26)
            return FrameS2;

    }

    return TypeUnkonwn;
}

static void SetTrackerGateSize()
{
	switch(stE1Cfg.enBaseOpMode)
	{
		case 0x20:
			stSysStatus.trackerGateSize = 16;
			break;
		case 0x22:
			stSysStatus.trackerGateSize = 64;
			break;
		case 0x23:
			stSysStatus.trackerGateSize = 128;
			break;
		default:
			stSysStatus.trackerGateSize = 32;
			break;
	}
}

uint8_t viewlink_protocal_checksum(uint8_t* buf)
{
    uint8_t len = GetMsgLen(buf[3]);
    uint8_t checksum = buf[3];
    for(uint8_t i=0;i<len-2;i++)
    {
        // printf("checksum:%#x\n", checksum);
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
    printf("buf[0]:%#x,buf[1]:%#x,buf[2]:%#x,\n", buf[0], buf[1], buf[2]);

    ST_E1_CONFIG *e1Cfg = (ST_E1_CONFIG*)buf;

    stE1Cfg.enTrackSourceMode = e1Cfg->enTrackSourceMode;
    stE1Cfg.u8Para1 = e1Cfg->u8Para1;
    stE1Cfg.enBaseOpMode = e1Cfg->enBaseOpMode;
    stE1Cfg.u8Para2 = e1Cfg->u8Para2;

    //stop tracking
	if(stE1Cfg.enBaseOpMode == Stop)
	{
		stSysStatus.trackOn = false;
		stSysStatus.trackerInited = false;
	}

    SetTrackerGateSize();
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
    memcpy(&stC2Cfg.opCmdPara1, c2Cfg->opCmdPara1, 2);
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

static bool IsAllZero(uint8_t data)
{
    if (data == 0) {
        return true;
    }
    return false;
}

static bool IsNotAllZero(uint8_t data)
{
    if (data == 0) {
        return false;
    }
    return true;
}
static void VL_ParseSerialData_U(uint8_t* buf)
{
#if 0
    ST_U_CONFIG *uCfg = (ST_U_CONFIG*)buf;
    printf("\n\n--------------------------------------VL_ParseSerialData_U:%#x\n\n", uCfg->enOpCmd);
    if (uCfg->enOpCmd == SetOSD_1) {
        stSysStatus.osdSet1Ctrl.enOSDShow = IsAllZero(uCfg->para[0] & 0x1);
        stSysStatus.osdSet1Ctrl.enCrossShow = IsAllZero(uCfg->para[0] & 0x2);
        stSysStatus.osdSet1Ctrl.enAttitudeAngleShow = IsAllZero(uCfg->para[0] & 0x4);
        stSysStatus.osdSet1Ctrl.enMissDistanceShow = IsAllZero(uCfg->para[0] & 0x8);
        stSysStatus.osdSet1Ctrl.enACFTGPS1Show = IsAllZero(uCfg->para[0] & 0x10);
        stSysStatus.osdSet1Ctrl.enTimeShow = IsAllZero(uCfg->para[0] & 0x20);
        stSysStatus.osdSet1Ctrl.enEOFieldOfViewOrMultiplyShow = IsAllZero(uCfg->para[0] & 0x40);
        stSysStatus.osdSet1Ctrl.enSmallFontShow = IsAllZero(uCfg->para[0] & 0x80);
    } else if (uCfg->enOpCmd == SetOSD_2) {
        stSysStatus.osdSet2Ctrl.enSaveSet = IsNotAllZero(uCfg->para[0] & 0x1);
        stSysStatus.osdSet2Ctrl.enIRShow = IsNotAllZero(uCfg->para[0] & 0x2);
        stSysStatus.osdSet2Ctrl.enLRFShow = IsNotAllZero(uCfg->para[0] & 0x4);
        stSysStatus.osdSet2Ctrl.enGPSIsMGRS = IsNotAllZero(uCfg->para[0] & 0x8);
        stSysStatus.osdSet2Ctrl.enTFShow = IsNotAllZero(uCfg->para[0] & 0x10);
        stSysStatus.osdSet2Ctrl.enTAGGPSShow = IsNotAllZero(uCfg->para[0] & 0x20);
        stSysStatus.osdSet2Ctrl.enMultiplyGreenOrFieldOfViewAngleWhiteShow = IsNotAllZero(uCfg->para[0] & 0x40);
        stSysStatus.osdSet2Ctrl.enGPSIsDegMinSecShow = IsNotAllZero(uCfg->para[0] & 0x80);
    }

    if (!stSysStatus.osdSet1Ctrl.enSmallFontShow) {
        stSysStatus.osdFontSize = 0.8;
    } else {
        stSysStatus.osdFontSize = 0.6;
    }

    printf("\n\nenOSDShow=%d enCrossShow=%d enAttitudeAngleShow=%d enMissDistanceShow=%d enACFTGPS1Show=%d enTimeShow=%d enEOFieldOfViewOrMultiplyShow=%d enSmallFontShow=%d\n",
        stSysStatus.osdSet1Ctrl.enOSDShow,
        stSysStatus.osdSet1Ctrl.enCrossShow,
        stSysStatus.osdSet1Ctrl.enAttitudeAngleShow,
        stSysStatus.osdSet1Ctrl.enMissDistanceShow,
        stSysStatus.osdSet1Ctrl.enACFTGPS1Show,
        stSysStatus.osdSet1Ctrl.enTimeShow,
        stSysStatus.osdSet1Ctrl.enEOFieldOfViewOrMultiplyShow,
        stSysStatus.osdSet1Ctrl.enSmallFontShow);
    printf("\n\n enSaveSet=%d enIRShow=%d enLRFShow=%d enGPSIsMGRS=%d enTFShow=%d enTAGGPSShow=%d enMultiplyGreenOrFieldOfViewAngleWhiteShow=%d enGPSIsDegMinSecShow=%d\n",
        stSysStatus.osdSet2Ctrl.enSaveSet,
        stSysStatus.osdSet2Ctrl.enIRShow,
        stSysStatus.osdSet2Ctrl.enLRFShow,
        stSysStatus.osdSet2Ctrl.enGPSIsMGRS,
        stSysStatus.osdSet2Ctrl.enTFShow,
        stSysStatus.osdSet2Ctrl.enTAGGPSShow,
        stSysStatus.osdSet2Ctrl.enMultiplyGreenOrFieldOfViewAngleWhiteShow,
        stSysStatus.osdSet2Ctrl.enGPSIsDegMinSecShow);
#endif
}

static void VL_ParseSerialData_A1C1E1(uint8_t* buf)
{
    // printf("buf[0]:%#x,buf[1]:%#x,buf[2]:%#x,\n", buf[0], buf[1], buf[2]);
    ST_A1_CONFIG *a1Cfg = (ST_A1_CONFIG*)buf;
    stA1C1E1Cfg.a1Config.enServoCtrlMode = a1Cfg->enServoCtrlMode;
    memcpy(stA1C1E1Cfg.a1Config.para1, a1Cfg->para1, 2);
    memcpy(stA1C1E1Cfg.a1Config.para2, a1Cfg->para2, 2);
    memcpy(stA1C1E1Cfg.a1Config.para3, a1Cfg->para3, 2);
    memcpy(stA1C1E1Cfg.a1Config.para4, a1Cfg->para4, 2);

    // printf("%#x\n", a1Cfg->enServoCtrlMode);

    uint8_t *tempData = buf + 9; // 9个字节
    uint16_t tempData1 = *(uint16_t*)tempData;
    uint16_t tempData2 = ntohs(tempData1);
    ST_C1_CONFIG *c1Cfg = (ST_C1_CONFIG*)&tempData2;

    printf("C1[0]:%#x,C1[1]:%#x\n", tempData[0], tempData[1]);

    stA1C1E1Cfg.c1Config.enDispMode =  c1Cfg->enDispMode;
    stA1C1E1Cfg.c1Config.enOpCmd1 = c1Cfg->enOpCmd1;
    stA1C1E1Cfg.c1Config.enOpCmd1Para = c1Cfg->enOpCmd1Para;
    stA1C1E1Cfg.c1Config.laserCmd = c1Cfg->laserCmd;

    tempData = tempData + 2; // 2个字节
    ST_E1_CONFIG *e1Cfg = (ST_E1_CONFIG*)tempData;

    printf("E1[0]:%#x,E1[1]:%#x,E1[2]:%#x,\n", tempData[0], tempData[1], tempData[2]);

    stA1C1E1Cfg.e1Config.enTrackSourceMode = e1Cfg->enTrackSourceMode;
    stA1C1E1Cfg.e1Config.u8Para1 = e1Cfg->u8Para1;
    stA1C1E1Cfg.e1Config.enBaseOpMode = e1Cfg->enBaseOpMode;
    stA1C1E1Cfg.e1Config.u8Para2 = e1Cfg->u8Para2;

    if(stA1C1E1Cfg.e1Config.enBaseOpMode == AiIdentifySwitch)
	{
		if(stA1C1E1Cfg.e1Config.u8Para2 == 1)
		{
			stSysStatus.detOn = true;
		}
		else
		{
			stSysStatus.detOn = false;
		}
	}
    //start tracking
	if(stA1C1E1Cfg.e1Config.enBaseOpMode == OnTrack && stA1C1E1Cfg.a1Config.enServoCtrlMode == TrackMode)
	{
		stSysStatus.trackOn = true;
        stSysStatus.trackAssignPoint = cv::Point(960,540);
	}

    if(stA1C1E1Cfg.e1Config.enBaseOpMode > TrackingSpeedAdjustment &&
        stA1C1E1Cfg.e1Config.enBaseOpMode < BaseOpButt)
    {
        switch(stA1C1E1Cfg.e1Config.enBaseOpMode)
        {
            case SixteenSquareExtraSmallTemplates:
                stSysStatus.trackerGateSize = 16;
                break;
            case ThirtyTwoSquareSmallTemplates:
                stSysStatus.trackerGateSize = 32;
                break;
            case SixtyFourSquareMidTemplates:
                stSysStatus.trackerGateSize = 64;
                break;
            case OneHundredAndTwentyEightSquareBigTemplates:
                stSysStatus.trackerGateSize = 128;
                break;
            default:
                stSysStatus.trackerGateSize = 32;
                break;
        }
    }
    // printf("laserCmd:%d, enOpCmd1Para:%d, enOpCmd1:%d, enDispMode%d, \n", 
    // stA1C1E1Cfg.c1Config.laserCmd, stA1C1E1Cfg.c1Config.enOpCmd1Para, stA1C1E1Cfg.c1Config.enOpCmd1,\
    // stA1C1E1Cfg.c1Config.enDispMode);
    if(stA1C1E1Cfg.c1Config.enDispMode != 0)
        stSysStatus.enDispMode = (EN_DISP_MODE)stA1C1E1Cfg.c1Config.enDispMode;

    // if (stA1C1E1Cfg.c1Config.enOpCmd1 == IrWhite) {
    //     stSysStatus.enIrImgMode = EN_IRIMG_MODE::WHITEHOT;
    // } else if (stA1C1E1Cfg.c1Config.enOpCmd1 == IrBlack) {
    //     stSysStatus.enIrImgMode = EN_IRIMG_MODE::BLACKHOT;
    // } else if (stA1C1E1Cfg.c1Config.enOpCmd1 == IrRainbow) {
    //     stSysStatus.enIrImgMode = EN_IRIMG_MODE::PSEUDOCOLOR;
    // } else if (stA1C1E1Cfg.c1Config.enOpCmd1 == ScreenShoot) {
    //     stSysStatus.enScreenOpMode = EN_SCREEN_OP_MODE::SCREEN_SHOOT;
    // } else if (stA1C1E1Cfg.c1Config.enOpCmd1 == RecordStart) {
    //     stSysStatus.enScreenOpMode = EN_SCREEN_OP_MODE::RECORDING_START;
    // } else if (stA1C1E1Cfg.c1Config.enOpCmd1 == RecordEnd) {
    //     stSysStatus.enScreenOpMode = EN_SCREEN_OP_MODE::RECORDING_END;
    // }

    switch(stA1C1E1Cfg.c1Config.enOpCmd1)
    {
        case IrWhite:
            stSysStatus.enIrImgMode = EN_IRIMG_MODE::WHITEHOT;
            break;
        case IrBlack:
            stSysStatus.enIrImgMode = EN_IRIMG_MODE::BLACKHOT;
            break;
        case IrRainbow:
            stSysStatus.enIrImgMode = EN_IRIMG_MODE::PSEUDOCOLOR;
            break;
        case ScreenShoot:
            stSysStatus.enScreenOpMode = EN_SCREEN_OP_MODE::SCREEN_SHOOT;
            break;
        case RecordStart:
            stSysStatus.enScreenOpMode = EN_SCREEN_OP_MODE::RECORDING_START;
            break;
        case RecordEnd:
            stSysStatus.enScreenOpMode = EN_SCREEN_OP_MODE::RECORDING_END;
            break;
        default:
            break;
    }

    printf("c1Config.enOpCmd1:%#x\n", stA1C1E1Cfg.c1Config.enOpCmd1);
    printf("c1Config.enBaseOpMode:%#x\n", stA1C1E1Cfg.e1Config.enBaseOpMode);
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
    memcpy(&stA2C2E2Cfg.c2Config.opCmdPara1, c2Cfg->opCmdPara1, 2);

    tempData = tempData + 3; // 3个字节
    ST_E2_CONFIG *e2Cfg = (ST_E2_CONFIG*)tempData;
    stA2C2E2Cfg.e2Config.enExtendCmd1 = e2Cfg->enExtendCmd1;
    memcpy(stA2C2E2Cfg.e2Config.para1, e2Cfg->para1, 2);
    memcpy(stA2C2E2Cfg.e2Config.para2, e2Cfg->para2, 2);
    uint16_t trackPointX = 0;
    uint16_t trackPointY = 0;
    switch(stA2C2E2Cfg.e2Config.enExtendCmd1)
    {
        case EnableRecogInformationOutput:
            stSysStatus.detRetOutput = true;
            break;
        case TurnOffRecogInformationOutput:
            stSysStatus.detRetOutput = false;
            break;
        case TracePointTransferredToInstructionPos:
            memcpy(&trackPointX, e2Cfg->para1, 2);
            memcpy(&trackPointY, e2Cfg->para2, 2);
            trackPointX = ntohs(trackPointX);
            trackPointY = ntohs(trackPointY);
            stSysStatus.trackAssignPoint.x = 1920 / 2 + (int16_t)trackPointX;
            stSysStatus.trackAssignPoint.y = 1080 / 2 + (int16_t)trackPointY;
            printf("\n============>>Track Point:(%d, %d)<<============\n", stSysStatus.trackAssignPoint.x, stSysStatus.trackAssignPoint.y);
            stSysStatus.trackOn = true;
            break;
        //to do
        case TrackCoordinatePointSettingOfUpperLeftCornerOfRectangularArea:
        case TrackCoordinatePointSettingOfBottomLeftCornerOfRectangularArea:

        default:
            break;
    }

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

static void VL_ParseSerialData_T1F1B1D1(uint8_t* buf)
{
    memset(&stSysStatus.ACFTCoordinate, 0, sizeof(ST_COORDINATE_CONFIG));
    memset(&stSysStatus.TAGCoordinate, 0, sizeof(ST_COORDINATE_CONFIG));

    uint8_t *tempData = buf + 2;

    uint32_t longitude = 0;
    uint32_t latitude = 0;
    uint16_t altitude = 0;

    memcpy(&longitude, tempData, 4);
    memcpy(&latitude, tempData + 4, 4);
    memcpy(&altitude, tempData + 8, 2);
    
    stSysStatus.ACFTCoordinate.longitude = (double)longitude / 10000000;
    stSysStatus.ACFTCoordinate.latitude = (double)latitude / 10000000;
    stSysStatus.ACFTCoordinate.altitude = (double)altitude;

    tempData = buf + 12;

    memcpy(&longitude, tempData, 4);
    memcpy(&latitude, tempData + 4, 4);
    memcpy(&altitude, tempData + 8, 2);

    stSysStatus.TAGCoordinate.longitude = (double)longitude / 10000000;
    stSysStatus.TAGCoordinate.latitude = (double)latitude / 10000000;
    stSysStatus.TAGCoordinate.altitude = (int16_t)altitude;

    tempData = buf + 23;
    ST_B1_CONFIG *b1Cfg = (ST_B1_CONFIG*)tempData;
    b1Cfg->azimuthAngle = ntohs(b1Cfg->azimuthAngle);
    stSysStatus.rollAngle = (double)(b1Cfg->azimuthAngle) * 360 / 65536;
    printf("rollAngle= %.2f\n", stSysStatus.rollAngle);

    b1Cfg->pitchAngle = ntohs(b1Cfg->pitchAngle);
    stSysStatus.pitchAngle = (double)(b1Cfg->pitchAngle) * 360 / 65536;
    printf("pitchAngle= %.2f\n", stSysStatus.pitchAngle);

    tempData = buf + 29;
    ST_D1_CONFIG *d1Cfg = (ST_D1_CONFIG*)tempData;
    stT1F1B1D1Cfg.d1Config.thermalImagingElectronicMagnification = d1Cfg->thermalImagingElectronicMagnification;
    stSysStatus.lrfValue = (double)((d1Cfg->distanceMeasurementReturnValueH << 8) ^ ntohs(d1Cfg->distanceMeasurementReturnValueL)) * 0.1;
    stT1F1B1D1Cfg.d1Config.visibleLightElectronicMagnification = d1Cfg->visibleLightElectronicMagnification;
    stT1F1B1D1Cfg.d1Config.currSensorHoriFieldOfViewAngle = ntohs(d1Cfg->currSensorHoriFieldOfViewAngle);
    stT1F1B1D1Cfg.d1Config.currSensorOpticsAmplificationFactor = ntohs(d1Cfg->currSensorOpticsAmplificationFactor);
}

static void VL_ParseSerialData_T1F1B1D1OSD(uint8_t* buf)
{
    
}

static void VL_ParseSerialData_T2F2B2D2(uint8_t* buf)
{
}



static void VL_ParseSerialData_V(uint8_t* buf)
{
#if 0
    ST_V_CONFIG *vCfg = (ST_V_CONFIG*)buf;
    printf("-----------------------vvvvvvvvvvvvvvvvvvv[%02x]\n", vCfg->ctrlCmd);
    if (vCfg->ctrlCmd == DeviceModel) {
        std::ostringstream oss;
        oss << static_cast<char>(vCfg->data[0]);
        std::string deviceModelId = oss.str();
        if (deviceModelId == "A") {
            stSysStatus.isTSeriesDevice = false;
        } else {
            stSysStatus.isTSeriesDevice = true;
        }
    } else if (vCfg->ctrlCmd == OSD) {
        stSysStatus.osdSet1Ctrl.enOSDShow = IsAllZero(vCfg->data[0] & 0x1);
        stSysStatus.osdSet1Ctrl.enCrossShow = IsAllZero(vCfg->data[0] & 0x2);
        stSysStatus.osdSet1Ctrl.enAttitudeAngleShow = IsAllZero(vCfg->data[0] & 0x4);
        stSysStatus.osdSet1Ctrl.enMissDistanceShow = IsAllZero(vCfg->data[0] & 0x8);
        stSysStatus.osdSet1Ctrl.enACFTGPS1Show = IsAllZero(vCfg->data[0] & 0x10);
        stSysStatus.osdSet1Ctrl.enTimeShow = IsAllZero(vCfg->data[0] & 0x20);
        stSysStatus.osdSet1Ctrl.enEOFieldOfViewOrMultiplyShow = IsAllZero(vCfg->data[0] & 0x40);
        stSysStatus.osdSet1Ctrl.enSmallFontShow = IsAllZero(vCfg->data[0] & 0x80);
        stSysStatus.osdSet2Ctrl.enSaveSet = IsNotAllZero(vCfg->data[1] & 0x1);
        stSysStatus.osdSet2Ctrl.enIRShow = IsNotAllZero(vCfg->data[1] & 0x2);
        stSysStatus.osdSet2Ctrl.enLRFShow = IsNotAllZero(vCfg->data[1] & 0x4);
        stSysStatus.osdSet2Ctrl.enGPSIsMGRS = IsNotAllZero(vCfg->data[1] & 0x8);
        stSysStatus.osdSet2Ctrl.enTFShow = IsNotAllZero(vCfg->data[1] & 0x10);
        stSysStatus.osdSet2Ctrl.enTAGGPSShow = IsNotAllZero(vCfg->data[1] & 0x20);
        stSysStatus.osdSet2Ctrl.enMultiplyGreenOrFieldOfViewAngleWhiteShow = IsNotAllZero(vCfg->data[1] & 0x40);
        stSysStatus.osdSet2Ctrl.enGPSIsDegMinSecShow = IsNotAllZero(vCfg->data[1] & 0x80);
        
        printf("\nVL_ParseSerialData_V\nenOSDShow=%d enCrossShow=%d enAttitudeAngleShow=%d enMissDistanceShow=%d enACFTGPS1Show=%d enTimeShow=%d enEOFieldOfViewOrMultiplyShow=%d enSmallFontShow=%d\n",
        stSysStatus.osdSet1Ctrl.enOSDShow,
        stSysStatus.osdSet1Ctrl.enCrossShow,
        stSysStatus.osdSet1Ctrl.enAttitudeAngleShow,
        stSysStatus.osdSet1Ctrl.enMissDistanceShow,
        stSysStatus.osdSet1Ctrl.enACFTGPS1Show,
        stSysStatus.osdSet1Ctrl.enTimeShow,
        stSysStatus.osdSet1Ctrl.enEOFieldOfViewOrMultiplyShow,
        stSysStatus.osdSet1Ctrl.enSmallFontShow);

        printf("\nenSaveSet=%d enIRShow=%d enLRFShow=%d enGPSIsMGRS=%d enTFShow=%d enTAGGPSShow=%d enMultiplyGreenOrFieldOfViewAngleWhiteShow=%d enGPSIsDegMinSecShow=%d\n",
        stSysStatus.osdSet2Ctrl.enSaveSet,
        stSysStatus.osdSet2Ctrl.enIRShow,
        stSysStatus.osdSet2Ctrl.enLRFShow,
        stSysStatus.osdSet2Ctrl.enGPSIsMGRS,
        stSysStatus.osdSet2Ctrl.enTFShow,
        stSysStatus.osdSet2Ctrl.enTAGGPSShow,
        stSysStatus.osdSet2Ctrl.enMultiplyGreenOrFieldOfViewAngleWhiteShow,
        stSysStatus.osdSet2Ctrl.enGPSIsDegMinSecShow);
    }
    if (!stSysStatus.osdSet1Ctrl.enSmallFontShow) {
        stSysStatus.osdFontSize = 0.8;
    } else {
        stSysStatus.osdFontSize = 0.6;
    }
#endif
}

static void VL_ParseSerialData_CMD_SD(uint8_t* buf)
{
    ST_CMD_SD_CONFIG *cmdSdCfg = (ST_CMD_SD_CONFIG*)buf;
    stCmdSDCfg.ctrlCmd = cmdSdCfg->ctrlCmd;
    stCmdSDCfg.para = cmdSdCfg->para;
}

static void VL_ParseSerialData_ACK_SD(uint8_t* buf)
{
    ST_ACK_SD_CONFIG *ackSdCfg = (ST_ACK_SD_CONFIG*)buf;
    stAckSDCfg.ctrlCmd = ackSdCfg->ctrlCmd;
    if (stAckSDCfg.ctrlCmd == (InquirySDCardStatus - 1)) {
        stAckSDCfg.ackSDData[0] = ackSdCfg->ackSDData[0];
        stAckSDCfg.ackSDData[1] = ackSdCfg->ackSDData[1];
    } else if ((stAckSDCfg.ctrlCmd >= (InquirySDCardTotalCapacity - 1)) && (stAckSDCfg.ctrlCmd < SDQueryCmdButt)) {
        memcpy(&stAckSDCfg.ackSDData, ackSdCfg->ackSDData, 4);
    }
}


void VL_ParseSerialData(uint8_t* buf)
{
    uint8_t frameID = buf[4];
    switch(frameID)
    {
        case 0x30:
            VL_ParseSerialData_A1C1E1(buf+5);
            break;
        case 0x32:
            VL_ParseSerialData_A1C1E1S1(buf);
            break;
        case 0x1A:
            VL_ParseSerialData_A1(buf);
            break;
        case 0x1C:
            // uint8_t *data = buf + 5;
            VL_ParseSerialData_C1(buf+5);
            break;
        case 0x1E:
            VL_ParseSerialData_E1(buf+5);
            break;
        case 0x16:
            VL_ParseSerialData_S1(buf);
            break;
        case 0x31:
            VL_ParseSerialData_A2C2E2(buf + 5);
            break;
        case 0x33:
            VL_ParseSerialData_A2C2E2S2(buf + 5);
            break;
        case 0x2A:
            VL_ParseSerialData_A2(buf);
            break;
        case 0x2C:
            VL_ParseSerialData_C2(buf + 5);
            break;
        case 0x2E:
            VL_ParseSerialData_E2(buf);
            break;
        case 0x26:
            VL_ParseSerialData_S2(buf);
            break;
        case 0x01:
            VL_ParseSerialData_U(buf + 5);
            break;
        case 0x40:
            VL_ParseSerialData_T1F1B1D1(buf + 5);
            break;
        case 0x42:
            VL_ParseSerialData_T1F1B1D1OSD(buf + 5);
            break;
        case 0x41:
            VL_ParseSerialData_T2F2B2D2(buf + 5);
            break;
        case 0x02:
            VL_ParseSerialData_V(buf + 5);
            break;
        case 0x5D:
            VL_ParseSerialData_CMD_SD(buf + 5);
            break;
        case 0xD5:
            VL_ParseSerialData_ACK_SD(buf + 5);
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
    portId = comport;
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
        fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);  
    }

    // fd = open(devFile, O_RDWR | O_NOCTTY | O_NDELAY);  
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
    printf("fd-open=%d, portId:%d\n", fd, portId);  
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
        // retval = select(fd + 1, &rfds, NULL, NULL, NULL);  
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

    if ((iSetOpt = setOpt(fdSerial, 115200, 8, 'N', 1))<0)  
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
    return readDataTty(fdSerial, buffRcvData, 100, 70);
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
            

            if(portId == 1) //serial up
            {
                printf("serial[%d] received serial up data\n", portId);
                for(int i=0; i< retLen ;i++)
                {
                    printf("[%02X]", buffRcvData_servo[i]);
                }
                std::cout<<std::endl<<std::endl;
            }

            else if(portId == 2) //serial down
            {
                printf("serial[%d] received serial down data\n", portId);
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
            }

            

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
    return this->st;
}


//关闭串口
int Serial::closePort(int fd)
{
    close(fd);

    return 0;
}

int Serial::ProcessSerialData(uint8_t *inputBUf, int inputLen, uint8_t *OutputBuf, int &outputLen)
{
    int loopCnt = inputLen;
    int dataLen;

    int remainBytes = loopCnt;

    // printf("ProcessSerialData start loopCnt:%d\n", loopCnt);
    // for(int i=0; i< inputLen ;i++)
    // {
    //     printf("[%02X]", inputBUf[i]);
    // }
    // printf("\n");

    if(st == 4)
    {
        uint8_t tmp[1000] = {0};
        memcpy(tmp, inputBUf, inputLen);
        memcpy(inputBUf, buf, bufLen);
        memcpy(inputBUf+bufLen, tmp, inputLen);
        loopCnt = inputLen + bufLen;
        st = 0;

        // printf("in if after memcpy\n");
        // for(int i=0; i< loopCnt ;i++)
        // {
        //     printf("[%02X]", inputBUf[i]);
        // }
        // printf("\n");
    }


    for(int i=0;i<loopCnt;i++)
    {
        remainBytes = loopCnt - i;
        // printf("i=%d, st:%d\n", i, st);
        switch(st)
        {
            case 0:
                if(inputBUf[i] == 0x55)
                {
                    st = 1;
                }
                // --remainBytes;
                break;
            case 1:
                if(inputBUf[i] == 0xAA)
                {
                    st = 2;
                }
                else
                {
                    st = 0;
                }
                // --remainBytes;
                break;
            case 2:
                if(inputBUf[i] == 0xDC)
                {
                    st = 3;
                }
                else
                {
                    st = 0;
                }
                // --remainBytes;
                break;
            case 3:
                dataLen = GetMsgLen(inputBUf[i]);
                // printf("dataLen:%d, remainBytes:%d\n", dataLen, remainBytes);
                if(dataLen == 4)
                {
                    st = 0;
                    if(remainBytes == 4)
                    {
                        memcpy(OutputBuf, inputBUf+i-3, dataLen + 3);
                        outputLen = dataLen + 3;
                        // printf("return heart beat\n");
                        return RET_OK;
                    }

                    
                    i += 3;
                    remainBytes -= 4;
                    break;
                }
                
                if(dataLen == remainBytes)  
                {
                    printf("*****complete frame*****\n");
                    memcpy(OutputBuf, inputBUf+i-3, remainBytes + 3);
                    outputLen = remainBytes + 3;

                    // for(int i=0; i< outputLen;i++)
                    // {
                    //     printf("[%02X]", OutputBuf[i]);
                    // }
                    // printf("\n");
                    st = 6;
                    break;
                }
                else if(dataLen < remainBytes)
                {
                    // printf("dataLen < remainBytes\n");
                    bufLen = remainBytes - dataLen;
                    memcpy(buf, inputBUf +i + dataLen, bufLen);
                    memset(inputBUf + dataLen + i, 0, bufLen);

                    memcpy(OutputBuf, inputBUf+i-3, dataLen + 3);
                    outputLen = dataLen + 3;
                    st = 4;

                    // for(int i=0; i< outputLen;i++)
                    // {
                    //     printf("[%02X]", OutputBuf[i]);
                    // }
                    // printf("\n");


                    // printf("OutputBuf[dataLen + 2]:%#x\n", OutputBuf[dataLen + 2]);
                    // printf("1111checksum:%#x\n", viewlink_protocal_checksum(OutputBuf));
                    if(OutputBuf[outputLen-1] == viewlink_protocal_checksum(OutputBuf))// complete frame
                    {
                        return RET_OK;
                    }
                    else
                    {
                        printf("frame checksum error, drop data\n\n");
                        memset(OutputBuf,0,1024);

                        for(int i=0; i< outputLen;i++)
                        {
                            printf("[%02X]", OutputBuf[i]);
                        }
                        printf("\n");
                        return RET_ERR;
                    }

                    break;
                }
                else //dataLen + 3 > remainBytes
                {
                    // printf("dataLen > remainBytes\n");
                    memcpy(buf, inputBUf + i-3, remainBytes);
                    bufLen = remainBytes+3;
                    st = 4;

                    // for(int i=0; i< bufLen;i++)
                    // {
                    //     printf("[%02X]", buf[i]);
                    // }
                    // printf("\n");
                    

                    return RET_ERR;
                }
                break;
            case 6:
                if(OutputBuf[outputLen-1] == viewlink_protocal_checksum(OutputBuf))// complete frame
                {
                    st = 0;
                    // memcpy(OutputBuf, inputBUf+i-1, dataLen + 3);
                    return RET_OK;
                }
                else
                {
                    st = 0;
                    printf("frame checksum error, drop data\n\n");
                    memset(inputBUf,0,1024);

                    for(int i=0; i< outputLen;i++)
                    {
                        printf("[%02X]", OutputBuf[i]);
                    }
                    printf("\n");
                    return RET_ERR;
                }
                break;
            default:
                st = 0;

        }
    }

    printf("return \n");
    return RET_ERR;
}


