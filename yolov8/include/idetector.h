#ifndef _IDETECTOR_H_
#define _IDETECTOR_H_

#include <string>
#include <opencv2/opencv.hpp>
#include "NvInfer.h"

using namespace nvinfer1;

struct bbox_t {
    unsigned int x, y, w, h;       // (x,y) - top-left corner, (w, h) - width & height of bounded box
    float prob;                    // confidence - probability that the object was found correctly
    unsigned int obj_id;           // class of object - from range [0, classes-1]
    unsigned int track_id;         // tracking id for video (0 - untracked, 1 - inf - tracked object)
    unsigned int frames_counter;   // counter of frames on which the object was detected
    float x_3d, y_3d, z_3d;        // center of object (in Meters) if ZED 3D Camera is used
    bbox_t(unsigned int xx, unsigned int yy, unsigned int ww, unsigned int hh, unsigned int cls, unsigned int id, float conf):
    x(xx), y(yy), w(ww), h(hh), obj_id(cls), track_id(id), prob(conf){};
    bbox_t(){}
};



class idetector
{
public:
    explicit idetector(const std::string& enginepath);
    ~idetector();
    void init();
    void process(cv::Mat& img);
    void process(cv::Mat& img, std::vector<bbox_t>& boxs);
    void processNoDraw(cv::Mat& img, std::vector<bbox_t>& boxs);

private:
    std::string engine_name;
    int model_bboxes;
    std::string cuda_post_process;

    // Deserialize the engine from file
    IRuntime *runtime;
    ICudaEngine *engine;
    IExecutionContext *context;
    cudaStream_t stream;
    
    // Prepare cpu and gpu buffers
    float *device_buffers[2];
    float *output_buffer_host = nullptr;
    float *decode_ptr_host=nullptr;
    float *decode_ptr_device=nullptr;

};


#endif