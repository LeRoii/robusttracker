#ifndef GUIDEMT_H
#define GUIDEMT_H

#ifdef _WIN32
#define dllexport __declspec(dllexport)
#else
#define dllexport __attribute__ ((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif


typedef struct
{
    unsigned short emiss;
    unsigned short relHum;
    short reflectedTemper;
    short atmosphericTemper;
    unsigned short modifyK;
    short modifyB;
}guide_measure_external_param_t;

typedef enum
{
    ISOTHERM_MODE_RANGE_NONE = 0,
    ISOTHERM_MODE_RANGE_MIDDLE,
    ISOTHERM_MODE_RANGE_UP_DOWN
}guide_isothermmode_e;


typedef enum
{
    GMT_SUCCESS                     =  0,
    GMT_ERROR_PARAMLINE             = -1,
    GMT_ERROR_EMISS                 = -2,
    GMT_ERROR_RELHUM                = -3,
    GMT_ERROR_DISTANCE              = -4,
    GMT_ERROR_REFLECTED_TEMPER      = -5,
    GMT_ERROR_ATMOSPHERIC_TEMPER    = -6,
    GMT_ERROR_MODIFY_K              = -7,
    GMT_ERROR_MODIFY_B              = -8,
    GMT_ERROR_SHUTTER               = -9,
    GMT_ERROR_POINTER_NULL          = -10,
    GMT_ERROR_PALETTE_INDEX         = -11,
    GMT_ERROR_INPUT_PARAM           = -12
}gmt_ret_code_e;

/**
 * @brief 距离修正函数
 * @param al
 * @param bl
 * @param cl
 * @param distance_enable 0-不修正 1-修正
 * @return @ref gmt_ret_code_e  0 成功 <0 失败
 */
dllexport int guide_measure_distance_correction(double al,double bl,double cl,int distance_enable);

/**
 * @brief 将Y16数据转换成温度数据
 * @param[in]   devType     机芯型号。
 * @param[in]   lensType    镜头类型。
 * @param[in]   pGray       从机芯数字口获取的16位灰度数据。
 * @param[in]   pParamLine  从机芯数字口获取的参数行数据，传入格式为0xAA、0x55、0x38、0x00……。
 * @param[in]   len         温度转换的个数。1-单点，n-多点，width*height-全图。
 * @param[in]   pParamExt   外部设置的测温参数。
 * @param[out]  pTemper     转换后的温度数据。外部分配内存，大小和pGray相同。
 * @return      @ref gmt_ret_code_e  0 成功 <0 失败
 */
dllexport int guide_measure_convertgray2temper(int devType, int lensType, short* pGray, char* pParamLine, int len, guide_measure_external_param_t* pParamExt, float* pTemper);
/**
 * @brief 将温度数据转换成Y16数据
 * @param[in]   devType     机芯型号。
 * @param[in]   lensType    镜头类型。
 * @param[in]   pTemper     温度数据
 * @param[in]   pParamLine  从机芯数字口获取的参数行数据，传入格式为0xAA、0x55、0x38、0x00……。
 * @param[in]   len         转换的个数。1-单点，n-多点，width*height-全图。
 * @param[in]   pParamExt   外部设置的测温参数。
 * @param[out]  pGray       转换后的Y16数据。外部分配内存，大小和pTemper相同。
 * @return      @ref gmt_ret_code_e  0 成功 <0 失败
 */
dllexport int guide_measure_converttemper2gray(int devType, int lensType, float *pTemper,  char* pParamLine, int len, guide_measure_external_param_t *pParamExt, short *pGray);

/**
 * @brief 等温线
 * @param[in]   devType        机芯型号。
 * @param[in]   lensType       镜头类型。
 * @param[in]   temperal       最低温度
 * @param[in]   temperah       最高温度。
 * @param[in]   y16data        从机芯获取的y16数据。
 * @param[in]   yuvsrcdata     从机芯获取的yuv数据
 * @param[out]  yuvdstdata     等温线yuv数据,外部分配内存，大小和yuvsrcdata相同
 * @param[in]   paramline      从机芯获取的参数行数据。
 * @param[in]   width          图像宽度。
 * @param[in]   height         图像高度。
 * @param[in]   pParamExt      外部设置的测温参数。
 * @param[in]   isothermmode   等温线模式，参见guide_isothermmode_e
 * @param[in]   paletteIndex   伪彩序号0~9
 * @return      @ref gmt_ret_code_e  0 成功 <0 失败
 */

dllexport int guide_isotherm(int devtype, int lenstype,float temperal, float temperah, int* y16Data, short* yuvsrcData,short* yuvdstData, unsigned char* paramline, int width, int height, guide_measure_external_param_t* pParamExt, guide_isothermmode_e isothermmode,int paletteIndex);

/**
 * @brief 温度矩阵转rgb图像
 * @param[in]   pTemp 温度矩阵
 * @param[out]  pRgb  外部分配内存，大小width * height * 3
 * @param[in]   width 图像宽度
 * @param[in]   height 图像高度
 * @param[in]   minT 最小温度
 * @param[in]   maxT 最大温度
 * @param[in]   paletteIndex伪彩序号0~9
 * @return      @ref gmt_ret_code_e  0 成功 <0 失败
 */
dllexport int guide_temp_to_rgb24(float* pTemp, unsigned char* pRgb, int width,int height,float minT, float maxT,int paletteIndex);

#ifdef __cplusplus
}
#endif

#endif // GUIDEMT_H
