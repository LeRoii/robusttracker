#ifndef _JETSON_ENCODER_H_
#define _JETSON_ENCODER_H_

#include <opencv2/opencv.hpp>
#include <stdint.h>
#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <memory>
#include "xop/RtspServer.h"
#include "net/Timer.h"
#include "NvVideoEncoder.h"



using namespace std;

#define TEST_ERROR(cond, str) if(cond) { \
                                        cerr << str << endl; \
                                        }



typedef struct
{
    NvVideoEncoder *enc;
    uint32_t encoder_pixfmt;

    char *in_file_path;
    std::ifstream *in_file;

    uint32_t width;
    uint32_t height;

    char *out_file_path;
    std::ofstream *out_file;

    uint32_t bitrate;
    uint32_t fps_n;
    uint32_t fps_d;
    std::shared_ptr<xop::RtspServer> RTSPserver;
    xop::MediaSession *session; 
    xop::MediaSessionId session_id;
    xop::RtspServer* rtsp_server;
    std::shared_ptr<xop::EventLoop> event_loop;

    bool got_error;
} context_t;


class jetsonEncoder
{
    public:
    jetsonEncoder(int port);
    ~jetsonEncoder();
    int encodeFrame(uint8_t *yuv_bytes);
    int process(cv::Mat &img);
    int sendBase64(cv::Mat &img);
    
    private:
    void set_defaults(context_t * ctx);
    
    void copyYuvToBuffer(uint8_t *yuv_bytes, NvBuffer &buffer);

    context_t ctx;
    int frame_count;
    bool websocketOn;

    


};

#endif