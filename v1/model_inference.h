#ifndef _MODEL_INFERENCE_H_
#define _MODEL_INFERENCE_H_


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include "common.h"

class CModelInference
{
public:
    CModelInference(char *modelPath, int threadIndex, int objClassNum, float nmsThreshold, float confThreshold);

    ~CModelInference();

    int InferenceModel(cv::Mat img);

public:
    object_detect_result_list detectResultGroup;

private:
    cv::Mat PreProcess(const cv::Mat originalImage, int &startX, int &startY, float &ratio);
    int Init();

    int ProcessFp32(float *box_tensor, float *score_tensor, float *score_sum_tensor,
                    int grid_h, int grid_w, int stride, int dfl_len,
                    std::vector<float> &boxes,
                    std::vector<float> &objProbs,
                    std::vector<int> &classId,
                    float threshold);

    int PostProcess(rknn_output *outputs, float ratio, int startX, int startY);

    int ProcessI8(int8_t *box_tensor, int32_t box_zp, float box_scale,
                  int8_t *score_tensor, int32_t score_zp, float score_scale,
                  int8_t *score_sum_tensor, int32_t score_sum_zp, float score_sum_scale,
                  int grid_h, int grid_w, int stride, int dfl_len,
                  std::vector<float> &boxes,
                  std::vector<float> &objProbs,
                  std::vector<int> &classId,
                  float threshold);

    int NMS(int validCount, std::vector<float> &outputLocations, std::vector<int> classIds, std::vector<int> &order,
            int filterId, float threshold);

    float CalculateOverlap(float xmin0, float ymin0, float xmax0, float ymax0, float xmin1, float ymin1, float xmax1,
                           float ymax1);

    int QuickSortIndiceInverse(std::vector<float> &input, int left, int right, std::vector<int> &indices);

    int8_t QntF32ToAffine(float f32, int32_t zp, float scale);

    int ReadDataFromFile(const char *path, char **out_data);

    inline int Clamp(float val, int min, int max);

    float DeqntAffineToF32(int8_t qnt, int32_t zp, float scale);

    inline int32_t Clip(float val, float min, float max);

    void DumpTensorAttr(rknn_tensor_attr *attr);

    void ComputeDfl(float *tensor, int dfl_len, float *box);

private:
    char *modelPath;
    unsigned char *modelData;
    int threadIndex;
    int objClassNum;
    float nmsThreshold;
    float confThreshold;

    cv::Mat InterenceImg;
    rknn_app_context_t ModelInfo;
};

#endif