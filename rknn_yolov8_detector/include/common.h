/*
 * @Author: WJM
 * @Date: 2024-03-08 15:09:55
 * @LastEditors: WJM
 * @LastEditTime: 2024-03-08 16:57:12
 * @Description:
 * @FilePath: /rknn_yolov8_detector_v2/include/common.h
 * @custom_string: http://www.aiar.xjtu.edu.cn/
 */
#ifndef __COMMON_H__
#define __COMMON_H__

#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <dirent.h>

#include "rknn_api.h"

#define OBJ_NAME_MAX_SIZE 16
#define OBJ_NUMB_MAX_SIZE 200
#define MAX_OUTPUTS 3

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

#endif