#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <sys/prctl.h>
#include <pthread.h>
#include <CL/cl.h>
#include <iostream>
#include <poll.h>
#include <stdio.h>  
#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "VideoPro_Enhancement.h"

#define INTPUT_Width              1920
#define INTPUT_Height             1080
#define OUTPUT_Width              1920
#define OUTPUT_Height             1080

#define MAX_SOURCE_SIZE (0x100000)

FILE *kernelFile;
char *kernelSource;
size_t kernelSize;
    cl_mem inputBuffer;


cl_context context ;
// 创建命令队列
cl_command_queue commandQueue;

cl_mem outputBuffer ;
cl_mem innerBuffer;
// 创建内核程序并设置参数
cl_mem gauss_kernel ;
// 创建内核程序并设置参数
cl_program program ;

cl_kernel kernel1; 


    // 执行内核函数
    size_t globalSize_i[2] = { INTPUT_Width, INTPUT_Height };
    size_t globalSize[2] = { OUTPUT_Width, OUTPUT_Height };


    size_t origin[3] = { 0, 0, 0 };
    size_t region[3] = { OUTPUT_Width, OUTPUT_Height, 1 };
    size_t region_i[3] = { INTPUT_Width, INTPUT_Height, 1 };

 size_t localSize[2] = { 32, 32 };
/****************************************************************************************
 * 函 数 名 ： VideoPro_Enhancement_Init
 * 功    能 ： 图像增强初始化
 * 输入参数 ： 输入结构体
 * 输出参数 ： 无
 * 返 回 值 ： 0: 成功；-1: 失败
 ***************************************************************************************/
int VideoPro_Enhancement_Init()
{
    int  count=0;
    int  Ret;
    int u32BuffSize;
    // 读取内核源代码文件
 
    kernelFile = fopen("/home/rpdzkj/2/kernel_.cl", "r");
    if (!kernelFile) {
        std::cout << "无法打开内核文件！" << std::endl;
        return -1;
    }
       std::cout << "111111111111111111" << std::endl; // 打印图像大小
    kernelSource = (char *)malloc(MAX_SOURCE_SIZE);
    kernelSize = fread(kernelSource, 1, MAX_SOURCE_SIZE, kernelFile);
    fclose(kernelFile);

    // 初始化OpenCL设备和上下文
    cl_platform_id platformId;
    cl_device_id deviceId;
    cl_uint numDevices, numPlatforms;
    cl_int ret;

    ret = clGetPlatformIDs(1, &platformId, &numPlatforms);

    ret = clGetDeviceIDs(platformId, CL_DEVICE_TYPE_GPU, 1, &deviceId, &numDevices);
    cl_context context = clCreateContext(NULL, 1, &deviceId, NULL, NULL, &ret);
       std::cout << "2222222222222222222222" << std::endl; // 打印图像大小
    // 创建命令队列
    cl_command_queue commandQueue = clCreateCommandQueue(context, deviceId, 0, &ret);
           std::cout << "3333333333333333" << std::endl; // 打印图像大小
    // 创建图像对象并加载图像数据
    size_t inputWidth = INTPUT_Width;
    size_t inputHeight = INTPUT_Height;
    size_t outputWidth =INTPUT_Width;
    size_t outputHeight = INTPUT_Height;
    size_t image_row_pitch = 0;
    size_t image_slice_pitch = 0;

    cl_image_format format;

    format.image_channel_order = CL_LUMINANCE;
    format.image_channel_data_type = CL_UNORM_INT8;

    cl_image_desc imageDesc;
    imageDesc.image_type = CL_MEM_OBJECT_IMAGE2D;
    imageDesc.image_width = inputWidth;
    imageDesc.image_height = inputHeight;
    imageDesc.image_row_pitch = 0;
    imageDesc.image_slice_pitch = 0;
    imageDesc.num_mip_levels = 0;
    imageDesc.num_samples = 0;
    imageDesc.buffer = NULL; 
    int kernel_size = 3;
    float gauss_sigma = 5.0;
    float gauss_alpha = 2.0;
    float gausskernel[(2*3+1)*(2*3+1)]={0};

    for (int i = 0; i <= (2*kernel_size+1); i++) 
    { 
        for (int j = 0; j <= (2*kernel_size+1); j++) 
        { 
            float weight = exp(-((i-kernel_size)*(i-kernel_size) + (j-kernel_size)*(j-kernel_size))/ (2.0 * gauss_sigma * gauss_sigma));
            gausskernel[i*(2*kernel_size+1)+j] = weight;
        }
    }
               std::cout << "44444444444444444444444" << std::endl; // 打印图像大小
     inputBuffer = clCreateImage2D(context, CL_MEM_READ_ONLY | CL_MEM_ALLOC_HOST_PTR, &format, inputWidth, inputHeight, 0, NULL, (cl_int*)&ret);
    //  outputBuffer = clCreateImage2D(context, CL_MEM_WRITE_ONLY, &format, outputWidth, outputHeight, 0, NULL, &ret);
     innerBuffer = clCreateImage2D(context, CL_MEM_WRITE_ONLY, &format, outputWidth, outputHeight, 0, NULL, &ret);
    // 创建内核程序并设置参数
     gauss_kernel = clCreateBuffer(context,CL_MEM_READ_ONLY| CL_MEM_COPY_HOST_PTR, sizeof(float)*(2*3+1),&gausskernel,(cl_int*)&ret);
    // 创建内核程序并设置参数
     program = clCreateProgramWithSource(context, 1, (const char **)&kernelSource, (const size_t *)&kernelSize,(cl_int*) &ret);
    ret = clBuildProgram(program, 1, &deviceId, NULL, NULL, NULL);
     kernel1 = clCreateKernel(program, "image_process", &ret);
               std::cout << "5555555555555555555" << std::endl; // 打印图像大小
    ret = clSetKernelArg(kernel1, 0, sizeof(cl_mem), (void *)&inputBuffer);
    ret = clSetKernelArg(kernel1, 1, sizeof(cl_mem), (void *)&innerBuffer);
    ret = clSetKernelArg(kernel1, 2, sizeof(cl_mem), (void *)&gauss_kernel);
    ret = clSetKernelArg(kernel1, 3, sizeof(int), &kernel_size);
    ret = clSetKernelArg(kernel1, 4, sizeof(float), &gauss_sigma);
    ret = clSetKernelArg(kernel1, 5, sizeof(float), &gauss_alpha);
               std::cout << "6666666666666666666" << std::endl; // 打印图像大小

    // cl_kernel kernel2 = clCreateKernel(program, "interpolate_image", &ret);
    // ret = clSetKernelArg(kernel2, 0, sizeof(cl_mem), (void *)&innerBuffer);
    // ret = clSetKernelArg(kernel2, 1, sizeof(cl_mem), (void *)&outputBuffer);
    // ret = clSetKernelArg(kernel2, 2, sizeof(cl_uint), (void *)&outputWidth);
    // ret = clSetKernelArg(kernel2, 3, sizeof(cl_uint), (void *)&outputHeight);

   
    // 从设备中读取输出图像
    // cv::Mat outputImage(outputHeight, outputWidth, CV_8U);

    std::cout << "777777777777777" << std::endl; // 打印图像大小

  //  unsigned char *data = (unsigned char *)clEnqueueMapImage(commandQueue, inputBuffer ,CL_TRUE,CL_MAP_READ,origin,region_i, &image_row_pitch, &image_slice_pitch,0,NULL,NULL,&ret);
    unsigned char *outputdata = (unsigned char *)clEnqueueMapImage(commandQueue,  innerBuffer, CL_TRUE, CL_MAP_READ, origin, region, &image_row_pitch, &image_slice_pitch, 0, NULL, NULL, &ret);
 
    std::cout << "888888888888888" << std::endl; // 打印图像大小
    sleep(3);
    return -1;
}


  cv::Mat FrameOutSend; 
//绘制界面上的其他参数
void VideoPro_Enhancement_Pro(cv::Mat &frame)
{

     std::cout << "9999999999999" << std::endl; // 打印图像大小
      // 从设备中读取输出图像
    cv::Mat outputImage(OUTPUT_Height, OUTPUT_Width, CV_8U);
    cl_int ret;

    ret = clEnqueueWriteImage(commandQueue, inputBuffer, CL_TRUE, origin, region_i, 0, 0, frame.data, 0, NULL, NULL);
    //std::cout << "000000000000000000" << std::endl; // 打印图像大小
    //clEnqueueUnmapMemObject(commandQueue, inputBuffer, data, 0, NULL, NULL);
    ret = clEnqueueNDRangeKernel(commandQueue, kernel1, 2, NULL, globalSize_i, localSize, 0, NULL, NULL);
    // ret = clEnqueueNDRangeKernel(commandQueue, kernel2, 2, NULL, globalSize, localSize, 0, NULL, NULL);

    std::cout << "11111111111111" << std::endl; // 打印图像大小
    clFinish(commandQueue);

    // auto end = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    // std::cout<<"overall time:"<< duration.count()<<"ms"<< std::endl;   

    //  memcpy(outputImage.data, outputdata, outputWidth*outputHeight * outputImage.elemSize());
    ret = clEnqueueReadImage(commandQueue, outputBuffer, CL_TRUE, origin, region, 0, 0, outputImage.data, 0, NULL, NULL);
    frame=outputImage.clone();
    std::cout << "1111图像大小: " << FrameOutSend.size() << std::endl; // 打印图像大小


}

/****************************************************************************************
 * 函 数 名 ： VideoPro_Enhancement_Destroy
 * 功    能 ： 图像增强句柄销毁
 * 输入参数 ： 输入结构体
 * 输出参数 ： 无
 * 返 回 值 ： 0: 成功；-1: 失败
 ***************************************************************************************/
int VideoPro_Enhancement_Destroy()
{
        // free(pstVFrame);
    // 清理内存和OpenCL资源
    clReleaseMemObject(inputBuffer);
    clReleaseMemObject(innerBuffer);
    clReleaseMemObject(gauss_kernel);
    clReleaseMemObject(outputBuffer);
    clReleaseProgram(program);
    clReleaseKernel(kernel1);
    // clReleaseKernel(kernel2);
    clReleaseCommandQueue(commandQueue);
    clReleaseContext(context);
    free(kernelSource);

    
}


