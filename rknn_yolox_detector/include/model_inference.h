#ifndef __IDETECTOR_H__
#define __IDETECTOR_H__

#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "RgaUtils.h"
#include "im2d.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/opencv.hpp"
#include "postprocess.h"
#include "rga.h"
#include "rknn_api.h"
#include <dirent.h>

#define OBJ_NAME_MAX_SIZE 16
#define OBJ_NUMB_MAX_SIZE 200
#define NMS_THRESHOLD 0.45
#define CONF_THRESHOLD 0.25
#define MAX_OUTPUTS 3
#define OBJ_CLASS_NUM 12
#define PROP_BOX_SIZE 17 // 5 + OBJ_CLASS_NUM

struct bbox_t
{
    unsigned int x, y, w, h;     // (x,y) - top-left corner, (w, h) - width & height of bounded box
    float prob;                  // confidence - probability that the object was found correctly
    unsigned int obj_id;         // class of object - from range [0, classes-1]
    unsigned int track_id;       // tracking id for video (0 - untracked, 1 - inf - tracked object)
    unsigned int frames_counter; // counter of frames on which the object was detected
    float x_3d, y_3d, z_3d;      // center of object (in Meters) if ZED 3D Camera is used
    bbox_t(unsigned int xx, unsigned int yy, unsigned int ww, unsigned int hh, unsigned int cls, unsigned int id, float conf) : x(xx), y(yy), w(ww), h(hh), obj_id(cls), track_id(id), prob(conf){};
    bbox_t() {}
};

typedef struct _detect_result_group_t
{
    int id;
    int count;
    bbox_t results[OBJ_NUMB_MAX_SIZE];
} detect_result_group_t;

typedef enum
{
    NORMAL_API = 0,
    ZERO_COPY_API,
} API_TYPE;

inline double __get_us(struct timeval t) { return (t.tv_sec * 1000000 + t.tv_usec); }

typedef struct _RKDEMO_INPUT_PARAM
{
    /*
        RKNN_INPUT has follow param:
        index, buf, size, pass_through, fmt, type

        Here we keep:
            pass_through,
            'fmt' as 'layout_fmt',
            'type' as 'dtype'

        And add:
            api_type to record normal_api/ zero_copy_api
            enable to assign if this param was used
            _already_init to record if this param was already init
    */
    uint8_t pass_through;
    rknn_tensor_format layout_fmt;
    rknn_tensor_type dtype;

    API_TYPE api_type;
    bool enable = false;
    bool _already_init = false;
} RKDEMO_INPUT_PARAM;

typedef struct _RKDEMO_OUTPUT_PARAM
{
    uint8_t want_float;

    API_TYPE api_type;
    bool enable = false;
    bool _already_init = false;
} RKDEMO_OUTPUT_PARAM;

typedef struct _MODEL_INFO
{
    rknn_context ctx;
    bool is_dyn_shape = false;

    int n_input;
    rknn_tensor_attr *in_attr = nullptr;
    rknn_tensor_attr *in_attr_native = nullptr;
    rknn_input *inputs;
    rknn_tensor_mem **input_mem;
    RKDEMO_INPUT_PARAM *rkdmo_input_param;
    int height;
    int width;
    int channel;
    int anchors[18];
    int anchor_per_branch;

    // bool inputs_alreay_init = false;

    int n_output;
    rknn_tensor_attr *out_attr = nullptr;
    rknn_tensor_attr *out_attr_native = nullptr;
    rknn_output *outputs;
    rknn_tensor_mem **output_mem;
    RKDEMO_OUTPUT_PARAM *rkdmo_output_param;
    int strides[3] = {8, 16, 32};

    // bool outputs_alreay_init = false;
    bool verbose_log = false;
    int diff_input_idx = -1;

    // memory could be set ousider
    int init_flag = 0;

} MODEL_INFO;

class modelInference
{
public:
    modelInference(char *model_path_, int thread_index_);
    virtual ~modelInference();
    int init();
    int detect();
    int post_process(void **rk_outputs, detect_result_group_t *group, float ratio, int startX, int startY);
    cv::Mat preprocess(const cv::Mat originalImage, int &startX, int &startY, float &ratio);
    cv::Mat img;
    std::vector<bbox_t> boxs;
    // bool run_flag;

private:
    char *model_path;
    unsigned char *model_data;
    int img_width;
    int img_height;
    int img_channel;
    GetResultRectYolov8 PostProcess;
    std::vector<float> DetectiontRects;
    rga_buffer_t src;
    rga_buffer_t dst;
    im_rect src_rect;
    im_rect dst_rect;
    MODEL_INFO *model_info;
    int model_channel;
    int model_width;
    int model_height;
    int thread_index;
    void *resize_buf = nullptr;
    float CalculateOverlap(float xmin0, float ymin0, float xmax0, float ymax0, float xmin1, float ymin1, float xmax1, float ymax1);
    int quick_sort_indice_inverse(
        std::vector<float> &input,
        int left,
        int right,
        std::vector<int> &indices);
    int8_t qnt_f32_to_affine(float f32, int32_t zp, float scale);
    int process_i8(int8_t *input, int *anchor, int anchor_per_branch, int grid_h, int grid_w, int height, int width, int stride,
                   std::vector<float> &boxes, std::vector<float> &boxScores, std::vector<int> &classId,
                   float threshold, int32_t zp, float scale);
    float deqnt_affine_to_f32(int8_t qnt, int32_t zp, float scale);
    int32_t __clip(float val, float min, float max);
    int nms(int validCount, std::vector<float> &outputLocations, std::vector<int> &class_id, std::vector<int> &order, float threshold, bool class_agnostic);
};

#endif