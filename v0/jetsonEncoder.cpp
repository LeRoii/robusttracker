

#include <opencv2/imgproc/types_c.h>
#include "jetsonEncoder.h"
#include <chrono>
#include <iomanip>
// #include "spdlog/spdlog.h"
// #include "stitcherglobal.h"

/**
 * Callback function called after capture plane dqbuffer of NvVideoEncoder class.
 * See NvV4l2ElementPlane::dqThread() in sample/common/class/NvV4l2ElementPlane.cpp
 * for details.
 *
 * @param v4l2_buf       : dequeued v4l2 buffer
 * @param buffer         : NvBuffer associated with the dequeued v4l2 buffer
 * @param shared_buffer  : Shared NvBuffer if the queued buffer is shared with
 *                         other elements. Can be NULL.
 * @param arg            : private data set by NvV4l2ElementPlane::startDQThread()
 *
 * @return               : true for success, false for failure (will stop DQThread)
 */
bool
encoder_capture_plane_dq_callback(struct v4l2_buffer *v4l2_buf, NvBuffer * buffer,
                                  NvBuffer * shared_buffer, void *arg)
{
    char str[80];
    size_t saved_size;
    context_t *ctx = (context_t *) arg;
    NvVideoEncoder *enc = ctx->enc;

    if (!v4l2_buf)
    {
        cerr << "Failed to dequeue buffer from encoder capture plane" << endl;
        //abort(ctx);
        return false;
    }
    //spdlog::warn("encoder_capture_plane_dq_callback,size:{}",buffer->planes[0].bytesused);

    if(buffer->planes[0].bytesused>0){
        xop::AVFrame videoFrame = {0};
        videoFrame.type = 0; // 建议确定帧类型。I帧(xop::VIDEO_FRAME_I) P帧(xop::VIDEO_FRAME_P)
        videoFrame.size = buffer->planes[0].bytesused;  // 视频帧大小 
        videoFrame.timestamp = xop::H264Source::GetTimestamp(); // 时间戳, 建议使用编码器提供的时间戳
        videoFrame.buffer.reset(new uint8_t[videoFrame.size]);                    
        memcpy(videoFrame.buffer.get(), buffer->planes[0].data, videoFrame.size);
        ctx->rtsp_server->PushFrame(ctx->session_id, xop::channel_0, videoFrame);  
        				   
        
    }
    else{
        cout<<"buffer no data"<<endl;
    }
    
    if (enc->capture_plane.qBuffer(*v4l2_buf, NULL) < 0)
    {
        cerr << "Error while Qing buffer at capture plane" << endl;
        //abort(ctx);
        return false;
    }

    /* GOT EOS from encoder. Stop dqthread. */
    if (buffer->planes[0].bytesused == 0)
    {
        return false;
    }
    return true;
}

jetsonEncoder::jetsonEncoder(int port)
{
    int ret = 0;
    frame_count = 0;
    set_defaults(&ctx);

    std::time_t tt = std::chrono::system_clock::to_time_t (std::chrono::system_clock::now());
    std::stringstream ss;
    ss << std::put_time(std::localtime(&tt), "%F-%H-%M-%S");
    std::string str = "/home/nvidia/"+ss.str()+".h264";
    ss.str("");
    ss << str;
    ss >> ctx.out_file_path;

    printf("jetsonEncoder jetsonEncoderjetsonEncoderjetsonEncoderjetsonEncoderjetsonEncoder\n");

    //spdlog::debug("ctx.out_file_path:{}", ctx.out_file_path);

    // ctx.out_file_path = "/home/nvidia/"+str+".h264";

    ctx.encoder_pixfmt = V4L2_PIX_FMT_H264;

    ctx.enc = NvVideoEncoder::createVideoEncoder("enc0");
    TEST_ERROR(!ctx.enc, "Could not create encoder");

    // ctx.out_file = new ofstream(ctx.out_file_path);
    // TEST_ERROR(!ctx.out_file->is_open(), "Could not open output file");

    /**
     * It is necessary that Capture Plane format be set before Output Plane
     * format.
     * It is necessary to set width and height on the capture plane as well.
     */
    ret = ctx.enc->setCapturePlaneFormat(ctx.encoder_pixfmt, ctx.width,
                                      ctx.height, 2 * 1024 * 1024);
    TEST_ERROR(ret < 0, "Could not set capture plane format");

    ret = ctx.enc->setOutputPlaneFormat(V4L2_PIX_FMT_YUV420M, ctx.width,
                                      ctx.height);
    TEST_ERROR(ret < 0, "Could not set output plane format");

    ret = ctx.enc->setIFrameInterval(25);
    ret = ctx.enc->setIDRInterval(30);  //ok
    ret = ctx.enc->setInsertSpsPpsAtIdrEnabled(true);

    ret = ctx.enc->setBitrate(ctx.bitrate);
    TEST_ERROR(ret < 0, "Could not set bitrate");

    ret = ctx.enc->setProfile(V4L2_MPEG_VIDEO_H264_PROFILE_HIGH);
    TEST_ERROR(ret < 0, "Could not set encoder profile");

    ret = ctx.enc->setLevel(V4L2_MPEG_VIDEO_H264_LEVEL_5_0);
    TEST_ERROR(ret < 0, "Could not set encoder level");

    ret = ctx.enc->setFrameRate(ctx.fps_n, ctx.fps_d);
    TEST_ERROR(ret < 0, "Could not set framerate");

    /**
     * Query, Export and Map the output plane buffers so that we can read
     * raw data into the buffers
     */
    ret = ctx.enc->output_plane.setupPlane(V4L2_MEMORY_MMAP, 10, true, false);
    TEST_ERROR(ret < 0, "Could not setup output plane");

    /**
     * Query, Export and Map the capture plane buffers so that we can write
     * encoded data from the buffers
     */
    ret = ctx.enc->capture_plane.setupPlane(V4L2_MEMORY_MMAP, 6, true, false);
    TEST_ERROR(ret < 0, "Could not setup capture plane");

    /* output plane STREAMON */
    ret = ctx.enc->output_plane.setStreamStatus(true);
    TEST_ERROR(ret < 0, "Error in output plane streamon");

    /* capture plane STREAMON */
    ret = ctx.enc->capture_plane.setStreamStatus(true);
    TEST_ERROR(ret < 0, "Error in capture plane streamon");

    ctx.enc->capture_plane.setDQThreadCallback(encoder_capture_plane_dq_callback);

    /**
     * startDQThread starts a thread internally which calls the
     * encoder_capture_plane_dq_callback whenever a buffer is dequeued
     * on the plane
     */
    ctx.enc->capture_plane.startDQThread(&ctx);

    /* Enqueue all the empty capture plane buffers */
    for (uint32_t i = 0; i < ctx.enc->capture_plane.getNumBuffers(); i++)
    {
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];

        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, MAX_PLANES * sizeof(struct v4l2_plane));

        v4l2_buf.index = i;
        v4l2_buf.m.planes = planes;

        ret = ctx.enc->capture_plane.qBuffer(v4l2_buf, NULL);
        if (ret < 0)
        {
            cerr << "Error while queueing buffer at capture plane" << endl;
        }
    }

    
    ctx.RTSPserver = xop::RtspServer::Create((new xop::EventLoop));
	if (!ctx.RTSPserver->Start("0.0.0.0", port)) {
        // printf("encoder exittttttt\n");
		exit;
	}
    

    ctx.session = xop::MediaSession::CreateNew("live"); // url: rtsp://ip/live
	ctx.session->AddSource(xop::channel_0, xop::H264Source::CreateNew()); 
	ctx.session->AddNotifyConnectedCallback([] (xop::MediaSessionId sessionId, std::string peer_ip, uint16_t peer_port){
		printf("RTSP client connect, ip=%s, port=%hu \n", peer_ip.c_str(), peer_port);
	});
	ctx.session->AddNotifyDisconnectedCallback([](xop::MediaSessionId sessionId, std::string peer_ip, uint16_t peer_port) {
		printf("RTSP client disconnect, ip=%s, port=%hu \n", peer_ip.c_str(), peer_port);
	});

	// std::cout << "URL: " << rtsp_url << std::endl;   
	ctx.session_id = ctx.RTSPserver->AddSession(ctx.session); 
    ctx.rtsp_server = ctx.RTSPserver.get();
    printf("encoder end\n");
}



jetsonEncoder::~jetsonEncoder()
{
    //spdlog::warn("jetsonEncoder destructor ");
    // c.close(hdl, websocketpp::close::status::normal, "");
    if(ctx.enc)
        delete ctx.enc;
}

void jetsonEncoder::set_defaults(context_t * ctx)
{
    memset(ctx, 0, sizeof(context_t));

    ctx->bitrate = 2 * 1024* 1024;
    ctx->fps_n = 30;
    ctx->fps_d = 1;
    //modify
    ctx->width = 1280;
    ctx->height = 720;
    ctx->out_file_path = new char[256];
}

void jetsonEncoder::copyYuvToBuffer(uint8_t *yuv_bytes, NvBuffer &buffer)
{

    uint32_t i, j, k;
    uint8_t *src = yuv_bytes;
    uint8_t *dst;

    for (i = 0; i < buffer.n_planes; i++) {

        NvBuffer::NvBufferPlane &plane = buffer.planes[i];
        uint32_t bytes_per_row = plane.fmt.bytesperpixel * plane.fmt.width;
        dst = plane.data;
        plane.bytesused = 0;

        for (j = 0; j < plane.fmt.height; j++) {
            for (k = 0; k < bytes_per_row; k++) {
                *dst = *src;
                dst++;
                src++;
            }
            dst += (plane.fmt.stride - bytes_per_row);
        }
        plane.bytesused = plane.fmt.stride * plane.fmt.height;
        // spdlog::warn("i:{},plane.bytesused:{}", i, plane.bytesused);
    }
}

int jetsonEncoder::encodeFrame(uint8_t *yuv_bytes)
{
    //spdlog::warn("jetsonEncoder::encodeFrame");
    int ret = 0;
    struct v4l2_buffer v4l2_buf;
    struct v4l2_plane planes[MAX_PLANES];
    NvBuffer *buffer = ctx.enc->output_plane.getNthBuffer(frame_count);
    //spdlog::warn("jetsonEncoder::529");
    memset(&v4l2_buf, 0, sizeof(v4l2_buf));
    memset(planes, 0, MAX_PLANES * sizeof(struct v4l2_plane));

    v4l2_buf.m.planes = planes;

    if (frame_count < ctx.enc->output_plane.getNumBuffers()) {
        v4l2_buf.index = frame_count;

        if (yuv_bytes)
            copyYuvToBuffer(yuv_bytes, *buffer);
        else
            v4l2_buf.m.planes[0].bytesused = 0;

    } else {
        //spdlog::warn("jetsonEncoder::544");
        if (ctx.enc->output_plane.dqBuffer(v4l2_buf, &buffer, nullptr, 10) < 0) {
            cerr << "ERROR while DQing buffer at output plane" << endl;
            // spdlog::warn("dqBuffer failed");
        }
    }

    //spdlog::warn("jetsonEncoder::552");
    
    if (frame_count >= ctx.enc->output_plane.getNumBuffers()) {
        if (yuv_bytes)
            copyYuvToBuffer(yuv_bytes, *buffer);
        else
            v4l2_buf.m.planes[0].bytesused = 0;
    }
    //spdlog::warn("ctx.enc->output_plane.qBuffer");
    //
    ret = ctx.enc->output_plane.qBuffer(v4l2_buf, nullptr);
    if (ret < 0) {
        cerr << "Error while queueing buffer at output plane" << endl;
        //abort(&ctx);
        //return cleanup(ctx, 1);
    }

    frame_count++;
    return ret;
}

int jetsonEncoder::process(cv::Mat &img)
{
    // spdlog::warn("jetsonEncoder::process");
    
    

    cv::Mat yuvImg;
    cv::resize(img, img, cv::Size(1280,720));

    cvtColor(img, yuvImg,CV_BGR2YUV_I420);

    //spdlog::warn("yuvImg size:{}", yuvImg.total()*yuvImg.elemSize());

    encodeFrame(yuvImg.data); 
    
}



