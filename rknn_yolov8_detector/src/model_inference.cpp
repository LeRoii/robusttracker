/*
 * @Author: WJM
 * @Date: 2024-03-08 15:07:41
 * @LastEditors: WJM
 * @LastEditTime: 2024-03-08 17:59:23
 * @Description:
 * @FilePath: /rknn_yolov8_detector_v2/src/model_inference.cpp
 * @custom_string: http://www.aiar.xjtu.edu.cn/
 */
#include "model_inference.h"

void CModelInference::DumpTensorAttr(rknn_tensor_attr *attr)
{
    printf("  index=%d, name=%s, n_dims=%d, dims=[%d, %d, %d, %d], n_elems=%d, size=%d, fmt=%s, type=%s, qnt_type=%s, "
           "zp=%d, scale=%f\n",
           attr->index, attr->name, attr->n_dims, attr->dims[0], attr->dims[1], attr->dims[2], attr->dims[3],
           attr->n_elems, attr->size, get_format_string(attr->fmt), get_type_string(attr->type),
           get_qnt_type_string(attr->qnt_type), attr->zp, attr->scale);
}

int CModelInference::NMS(int validCount, std::vector<float> &outputLocations, std::vector<int> classIds, std::vector<int> &order,
                         int filterId, float threshold)
{
    for (int i = 0; i < validCount; ++i)
    {
        if (order[i] == -1 || classIds[i] != filterId)
        {
            continue;
        }
        int n = order[i];
        for (int j = i + 1; j < validCount; ++j)
        {
            int m = order[j];
            if (m == -1 || classIds[i] != filterId)
            {
                continue;
            }
            float xmin0 = outputLocations[n * 4 + 0];
            float ymin0 = outputLocations[n * 4 + 1];
            float xmax0 = outputLocations[n * 4 + 0] + outputLocations[n * 4 + 2];
            float ymax0 = outputLocations[n * 4 + 1] + outputLocations[n * 4 + 3];

            float xmin1 = outputLocations[m * 4 + 0];
            float ymin1 = outputLocations[m * 4 + 1];
            float xmax1 = outputLocations[m * 4 + 0] + outputLocations[m * 4 + 2];
            float ymax1 = outputLocations[m * 4 + 1] + outputLocations[m * 4 + 3];

            float iou = CalculateOverlap(xmin0, ymin0, xmax0, ymax0, xmin1, ymin1, xmax1, ymax1);

            if (iou > threshold)
            {
                order[j] = -1;
            }
        }
    }
    return 0;
}

float CModelInference::CalculateOverlap(float xmin0, float ymin0, float xmax0, float ymax0, float xmin1, float ymin1, float xmax1,
                                        float ymax1)
{
    float w = fmax(0.f, fmin(xmax0, xmax1) - fmax(xmin0, xmin1) + 1.0);
    float h = fmax(0.f, fmin(ymax0, ymax1) - fmax(ymin0, ymin1) + 1.0);
    float i = w * h;
    float u = (xmax0 - xmin0 + 1.0) * (ymax0 - ymin0 + 1.0) + (xmax1 - xmin1 + 1.0) * (ymax1 - ymin1 + 1.0) - i;
    return u <= 0.f ? 0.f : (i / u);
}

int CModelInference::QuickSortIndiceInverse(std::vector<float> &input, int left, int right, std::vector<int> &indices)
{
    float key;
    int key_index;
    int low = left;
    int high = right;
    if (left < right)
    {
        key_index = indices[left];
        key = input[left];
        while (low < high)
        {
            while (low < high && input[high] <= key)
            {
                high--;
            }
            input[low] = input[high];
            indices[low] = indices[high];
            while (low < high && input[low] >= key)
            {
                low++;
            }
            input[high] = input[low];
            indices[high] = indices[low];
        }
        input[low] = key;
        indices[low] = key_index;
        QuickSortIndiceInverse(input, left, low - 1, indices);
        QuickSortIndiceInverse(input, low + 1, right, indices);
    }
    return low;
}
int8_t CModelInference::QntF32ToAffine(float f32, int32_t zp, float scale)
{
    float dst_val = (f32 / scale) + zp;
    int8_t res = (int8_t)Clip(dst_val, -128, 127);
    return res;
}

int CModelInference::ReadDataFromFile(const char *path, char **out_data)
{
    FILE *fp = fopen(path, "rb");
    if (fp == NULL)
    {
        printf("fopen %s fail!\n", path);
        return -1;
    }
    fseek(fp, 0, SEEK_END);
    int file_size = ftell(fp);
    char *data = (char *)malloc(file_size + 1);
    data[file_size] = 0;
    fseek(fp, 0, SEEK_SET);
    if (file_size != fread(data, 1, file_size, fp))
    {
        printf("fread %s fail!\n", path);
        free(data);
        fclose(fp);
        return -1;
    }
    if (fp)
    {
        fclose(fp);
    }
    *out_data = data;
    return file_size;
}

inline int CModelInference::Clamp(float val, int min, int max) { return val > min ? (val < max ? val : max) : min; }

float CModelInference::DeqntAffineToF32(int8_t qnt, int32_t zp, float scale) { return ((float)qnt - (float)zp) * scale; }
inline int32_t CModelInference::Clip(float val, float min, float max)
{
    float f = val <= min ? min : (val >= max ? max : val);
    return f;
}

void CModelInference::ComputeDfl(float *tensor, int dfl_len, float *box)
{
    for (int b = 0; b < 4; b++)
    {
        float exp_t[dfl_len];
        float exp_sum = 0;
        float acc_sum = 0;
        for (int i = 0; i < dfl_len; i++)
        {
            exp_t[i] = exp(tensor[i + b * dfl_len]);
            exp_sum += exp_t[i];
        }

        for (int i = 0; i < dfl_len; i++)
        {
            acc_sum += exp_t[i] / exp_sum * i;
        }
        box[b] = acc_sum;
    }
}

CModelInference::CModelInference(char *modelPath, int threadIndex, int objClassNum, float nmsThreshold, float confThreshold)
{
    this->modelPath = (char *)malloc(strlen(modelPath) + 1); // 需要加1，以便为字符串结尾的'\0'预留空间

    // 复制原始字符串到临时变量
    strcpy(this->modelPath, modelPath);
    this->threadIndex = threadIndex;
    this->objClassNum = objClassNum;
    this->nmsThreshold = nmsThreshold;
    this->confThreshold = confThreshold;
    Init();
}

int CModelInference::Init()
{
    int ret = 0;
    int model_len = 0;
    char *model;
    ModelInfo.rknn_ctx = 0;
    memset(&detectResultGroup, 0x00, sizeof(detectResultGroup));

    model_len = ReadDataFromFile(modelPath, &model);
    if (model == NULL)
    {
        printf("load_model fail!\n");
        return -1;
    }

    rknn_core_mask core_mask;
    if (threadIndex == 0)
        core_mask = RKNN_NPU_CORE_0;
    else if (threadIndex == 1)
        core_mask = RKNN_NPU_CORE_1;
    else
        core_mask = RKNN_NPU_CORE_2;

    ret = rknn_init(&(ModelInfo.rknn_ctx), model, model_len, 0, NULL);
    rknn_set_core_mask(ModelInfo.rknn_ctx, core_mask);
    free(model);

    if (ret < 0)
    {
        printf("rknn_init fail! ret=%d\n", ret);
        return -1;
    }

    rknn_sdk_version version;
    ret = rknn_query(ModelInfo.rknn_ctx, RKNN_QUERY_SDK_VERSION, &version, sizeof(rknn_sdk_version));
    if (ret != RKNN_SUCC)
    {
        printf("rknn_init error ret=%d\n", ret);
        return -1;
    }

    printf("sdk version: %s driver version: %s\n", version.api_version, version.drv_version);

    // Get Model Input Output Number
    rknn_input_output_num io_num;
    ret = rknn_query(ModelInfo.rknn_ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret != RKNN_SUCC)
    {
        printf("rknn_query fail! ret=%d\n", ret);
        return -1;
    }
    printf("model input num: %d, output num: %d\n", io_num.n_input, io_num.n_output);
    // Get Model Input Info
    printf("input tensors:\n");
    rknn_tensor_attr input_attrs[io_num.n_input];
    memset(input_attrs, 0, sizeof(input_attrs));

    for (int i = 0; i < io_num.n_input; i++)
    {
        input_attrs[i].index = i;
        ret = rknn_query(ModelInfo.rknn_ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]), sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC)
        {
            printf("rknn_query fail! ret=%d\n", ret);
            return -1;
        }
        DumpTensorAttr(&(input_attrs[i]));
    }

    // Get Model Output Info
    printf("output tensors:\n");
    rknn_tensor_attr output_attrs[io_num.n_output];
    memset(output_attrs, 0, sizeof(output_attrs));
    for (int i = 0; i < io_num.n_output; i++)
    {
        output_attrs[i].index = i;
        ret = rknn_query(ModelInfo.rknn_ctx, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]), sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC)
        {
            printf("rknn_query fail! ret=%d\n", ret);
            return -1;
        }
        DumpTensorAttr(&(output_attrs[i]));
    }

    // 此处判断模型是否使用int8或者FP16
    if (output_attrs[0].qnt_type == RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC && output_attrs[0].type != RKNN_TENSOR_FLOAT16)
    {
        ModelInfo.is_quant = true;
    }
    else
    {
        ModelInfo.is_quant = false;
    }

    ModelInfo.io_num = io_num;
    ModelInfo.input_attrs = (rknn_tensor_attr *)malloc(io_num.n_input * sizeof(rknn_tensor_attr));
    memcpy(ModelInfo.input_attrs, input_attrs, io_num.n_input * sizeof(rknn_tensor_attr));
    ModelInfo.output_attrs = (rknn_tensor_attr *)malloc(io_num.n_output * sizeof(rknn_tensor_attr));
    memcpy(ModelInfo.output_attrs, output_attrs, io_num.n_output * sizeof(rknn_tensor_attr));
    if (input_attrs[0].fmt == RKNN_TENSOR_NCHW)
    {
        printf("model is NCHW input fmt\n");
        ModelInfo.model_channel = input_attrs[0].dims[1];
        ModelInfo.model_height = input_attrs[0].dims[2];
        ModelInfo.model_width = input_attrs[0].dims[3];
    }
    else
    {
        printf("model is NHWC input fmt\n");
        ModelInfo.model_height = input_attrs[0].dims[1];
        ModelInfo.model_width = input_attrs[0].dims[2];
        ModelInfo.model_channel = input_attrs[0].dims[3];
    }
    printf("model input height=%d, width=%d, channel=%d\n",
           ModelInfo.model_height, ModelInfo.model_width, ModelInfo.model_channel);
    return 0;
}

cv::Mat CModelInference::PreProcess(const cv::Mat originalImage, int &startX, int &startY, float &ratio)
{
    int originalWidth = originalImage.cols;
    int originalHeight = originalImage.rows;
    int model_input_img_size = ModelInfo.model_height;
    // 创建一个新的 640x640 大小的黑色图像
    cv::Mat resizedImage(model_input_img_size, model_input_img_size, CV_8UC3, cv::Scalar(0, 0, 0));

    // 计算调整大小后的图像的宽度和高度
    int resizedWidth, resizedHeight;
    if (originalWidth > originalHeight)
    {
        resizedWidth = model_input_img_size;
        ratio = static_cast<float>(originalWidth) / static_cast<float>(model_input_img_size);
        resizedHeight = originalHeight * model_input_img_size / static_cast<float>(originalWidth);
    }
    else
    {
        resizedHeight = model_input_img_size;
        ratio = static_cast<float>(originalHeight) / static_cast<float>(model_input_img_size);
        resizedWidth = originalWidth * model_input_img_size / static_cast<float>(originalHeight);
        ;
    }
    // 计算调整大小后图像的起始坐标
    startX = (model_input_img_size - resizedWidth) / 2;
    startY = (model_input_img_size - resizedHeight) / 2;

    // 调整大小并将原始图像复制到新图像中
    cv::resize(originalImage, resizedImage(cv::Rect(startX, startY, resizedWidth, resizedHeight)), cv::Size(resizedWidth, resizedHeight));
    return resizedImage;
}

int CModelInference::InferenceModel(cv::Mat src_img)
{
    int startX, startY;
    float ratio;
    int ret;

    // Set Input Data
    rknn_input inputs[ModelInfo.io_num.n_input];
    rknn_output outputs[ModelInfo.io_num.n_output];

    memset(inputs, 0, sizeof(inputs));
    memset(outputs, 0, sizeof(outputs));

    cv::Mat dst_img = PreProcess(src_img, startX, startY, ratio);
    // std::cout << "startX,startY,ratio:" << startX << " " << startY << " " << ratio << std::endl;
    inputs[0].index = 0;
    inputs[0].type = RKNN_TENSOR_UINT8;
    inputs[0].size = ModelInfo.model_width * ModelInfo.model_height * ModelInfo.model_channel;
    inputs[0].fmt = RKNN_TENSOR_NHWC;
    inputs[0].buf = dst_img.data;

    ret = rknn_inputs_set(ModelInfo.rknn_ctx, ModelInfo.io_num.n_input, inputs);
    if (ret < 0)
    {
        printf("rknn_input_set fail! ret=%d\n", ret);
        return -1;
    }

    // Run
    ret = rknn_run(ModelInfo.rknn_ctx, nullptr);
    if (ret < 0)
    {
        printf("rknn_run fail! ret=%d\n", ret);
        return -1;
    }

    // Get Output
    memset(outputs, 0, sizeof(outputs));
    for (int i = 0; i < ModelInfo.io_num.n_output; i++)
    {
        outputs[i].index = i;
        outputs[i].want_float = (!ModelInfo.is_quant);
    }
    ret = rknn_outputs_get(ModelInfo.rknn_ctx, ModelInfo.io_num.n_output, outputs, NULL);
    if (ret < 0)
    {
        printf("rknn_outputs_get fail! ret=%d\n", ret);
        return -1;
    }
    PostProcess(outputs, ratio, startX, startY);

    rknn_outputs_release(ModelInfo.rknn_ctx, ModelInfo.io_num.n_output, outputs);
    return 0;
}

int CModelInference::PostProcess(rknn_output *outputs, float ratio, int startX, int startY)
{
    std::vector<float> filterBoxes;
    std::vector<float> objProbs;
    std::vector<int> classId;
    int validCount = 0;
    int stride = 0;
    int grid_h = 0;
    int grid_w = 0;
    int model_in_w = ModelInfo.model_width;
    int model_in_h = ModelInfo.model_height;

    memset(&detectResultGroup, 0, sizeof(object_detect_result_list));

    // default 3 branch
    int dfl_len = ModelInfo.output_attrs[0].dims[1] / 4;
    int output_per_branch = ModelInfo.io_num.n_output / 3;
    for (int i = 0; i < 3; i++)
    {

        void *score_sum = nullptr;
        int32_t score_sum_zp = 0;
        float score_sum_scale = 1.0;
        if (output_per_branch == 3)
        {
            score_sum = outputs[i * output_per_branch + 2].buf;
            score_sum_zp = ModelInfo.output_attrs[i * output_per_branch + 2].zp;
            score_sum_scale = ModelInfo.output_attrs[i * output_per_branch + 2].scale;
        }
        int box_idx = i * output_per_branch;
        int score_idx = i * output_per_branch + 1;

        grid_h = ModelInfo.output_attrs[box_idx].dims[2];
        grid_w = ModelInfo.output_attrs[box_idx].dims[3];
        stride = model_in_h / grid_h;

        if (ModelInfo.is_quant)
        {
            validCount += ProcessI8((int8_t *)outputs[box_idx].buf, ModelInfo.output_attrs[box_idx].zp, ModelInfo.output_attrs[box_idx].scale,
                                    (int8_t *)outputs[score_idx].buf, ModelInfo.output_attrs[score_idx].zp, ModelInfo.output_attrs[score_idx].scale,
                                    (int8_t *)score_sum, score_sum_zp, score_sum_scale,
                                    grid_h, grid_w, stride, dfl_len,
                                    filterBoxes, objProbs, classId, confThreshold);
        }
        else
        {
            validCount += ProcessFp32((float *)outputs[box_idx].buf, (float *)outputs[score_idx].buf, (float *)score_sum,
                                      grid_h, grid_w, stride, dfl_len,
                                      filterBoxes, objProbs, classId, confThreshold);
        }
    }

    // no object detect
    if (validCount <= 0)
    {
        return 0;
    }
    std::vector<int> indexArray;
    for (int i = 0; i < validCount; ++i)
    {
        indexArray.push_back(i);
    }
    QuickSortIndiceInverse(objProbs, 0, validCount - 1, indexArray);

    std::set<int> class_set(std::begin(classId), std::end(classId));

    for (auto c : class_set)
    {
        NMS(validCount, filterBoxes, classId, indexArray, c, nmsThreshold);
    }

    int last_count = 0;
    detectResultGroup.count = 0;

    /* box valid detect target */
    for (int i = 0; i < validCount; ++i)
    {
        if (indexArray[i] == -1 || last_count >= OBJ_NUMB_MAX_SIZE)
        {
            continue;
        }
        int n = indexArray[i];

        float x1 = filterBoxes[n * 4 + 0] - startX;
        float y1 = filterBoxes[n * 4 + 1] - startY;
        float w = filterBoxes[n * 4 + 2];
        float h = filterBoxes[n * 4 + 3];
        int id = classId[n];
        float obj_conf = objProbs[i];
        // std::cout << "x1,x2,w,h: " << x1 << " " << y1 << " " << w << " " << h << std::endl;
        // std::cout << "model_in_w " << model_in_w << " " << model_in_h << std::endl;

        detectResultGroup.results[last_count].x = (int)(Clamp(x1, 0, model_in_w) * ratio);
        detectResultGroup.results[last_count].y = (int)(Clamp(y1, 0, model_in_h) * ratio);
        detectResultGroup.results[last_count].w = w * ratio;
        detectResultGroup.results[last_count].h = h * ratio;
        detectResultGroup.results[last_count].prop = obj_conf;
        detectResultGroup.results[last_count].obj_id = id;
        last_count++;
    }
    detectResultGroup.count = last_count;
    return 0;
}

int CModelInference::ProcessI8(int8_t *box_tensor, int32_t box_zp, float box_scale,
                               int8_t *score_tensor, int32_t score_zp, float score_scale,
                               int8_t *score_sum_tensor, int32_t score_sum_zp, float score_sum_scale,
                               int grid_h, int grid_w, int stride, int dfl_len,
                               std::vector<float> &boxes,
                               std::vector<float> &objProbs,
                               std::vector<int> &classId,
                               float threshold)
{
    int validCount = 0;
    int grid_len = grid_h * grid_w;
    int8_t score_thres_i8 = QntF32ToAffine(threshold, score_zp, score_scale);
    int8_t score_sum_thres_i8 = QntF32ToAffine(threshold, score_sum_zp, score_sum_scale);

    for (int i = 0; i < grid_h; i++)
    {
        for (int j = 0; j < grid_w; j++)
        {
            int offset = i * grid_w + j;
            int max_class_id = -1;

            // 通过 score sum 起到快速过滤的作用
            if (score_sum_tensor != nullptr)
            {
                if (score_sum_tensor[offset] < score_sum_thres_i8)
                {
                    continue;
                }
            }

            int8_t max_score = -score_zp;
            for (int c = 0; c < objClassNum; c++)
            {
                if ((score_tensor[offset] > score_thres_i8) && (score_tensor[offset] > max_score))
                {
                    max_score = score_tensor[offset];
                    max_class_id = c;
                }
                offset += grid_len;
            }

            // compute box
            if (max_score > score_thres_i8)
            {
                offset = i * grid_w + j;
                float box[4];
                float before_dfl[dfl_len * 4];
                for (int k = 0; k < dfl_len * 4; k++)
                {
                    before_dfl[k] = DeqntAffineToF32(box_tensor[offset], box_zp, box_scale);
                    offset += grid_len;
                }
                ComputeDfl(before_dfl, dfl_len, box);

                float x1, y1, x2, y2, w, h;
                x1 = (-box[0] + j + 0.5) * stride;
                y1 = (-box[1] + i + 0.5) * stride;
                x2 = (box[2] + j + 0.5) * stride;
                y2 = (box[3] + i + 0.5) * stride;
                w = x2 - x1;
                h = y2 - y1;
                boxes.push_back(x1);
                boxes.push_back(y1);
                boxes.push_back(w);
                boxes.push_back(h);

                objProbs.push_back(DeqntAffineToF32(max_score, score_zp, score_scale));
                classId.push_back(max_class_id);
                validCount++;
            }
        }
    }
    return validCount;
}

int CModelInference::ProcessFp32(float *box_tensor, float *score_tensor, float *score_sum_tensor,
                                 int grid_h, int grid_w, int stride, int dfl_len,
                                 std::vector<float> &boxes,
                                 std::vector<float> &objProbs,
                                 std::vector<int> &classId,
                                 float threshold)
{
    int validCount = 0;
    int grid_len = grid_h * grid_w;
    for (int i = 0; i < grid_h; i++)
    {
        for (int j = 0; j < grid_w; j++)
        {
            int offset = i * grid_w + j;
            int max_class_id = -1;

            // 通过 score sum 起到快速过滤的作用
            if (score_sum_tensor != nullptr)
            {
                if (score_sum_tensor[offset] < threshold)
                {
                    continue;
                }
            }

            float max_score = 0;
            for (int c = 0; c < objClassNum; c++)
            {
                if ((score_tensor[offset] > threshold) && (score_tensor[offset] > max_score))
                {
                    max_score = score_tensor[offset];
                    max_class_id = c;
                }
                offset += grid_len;
            }

            // compute box
            if (max_score > threshold)
            {
                offset = i * grid_w + j;
                float box[4];
                float before_dfl[dfl_len * 4];
                for (int k = 0; k < dfl_len * 4; k++)
                {
                    before_dfl[k] = box_tensor[offset];
                    offset += grid_len;
                }
                ComputeDfl(before_dfl, dfl_len, box);

                float x1, y1, x2, y2, w, h;
                x1 = (-box[0] + j + 0.5) * stride;
                y1 = (-box[1] + i + 0.5) * stride;
                x2 = (box[2] + j + 0.5) * stride;
                y2 = (box[3] + i + 0.5) * stride;
                w = x2 - x1;
                h = y2 - y1;
                boxes.push_back(x1);
                boxes.push_back(y1);
                boxes.push_back(w);
                boxes.push_back(h);

                objProbs.push_back(max_score);
                classId.push_back(max_class_id);
                validCount++;
            }
        }
    }
    return validCount;
}

CModelInference::~CModelInference()
{

    if (ModelInfo.rknn_ctx != 0)
    {
        rknn_destroy(ModelInfo.rknn_ctx);
        ModelInfo.rknn_ctx = 0;
    }
    if (ModelInfo.input_attrs != NULL)
    {
        free(ModelInfo.input_attrs);
        ModelInfo.input_attrs = NULL;
    }
    if (ModelInfo.output_attrs != NULL)
    {
        free(ModelInfo.output_attrs);
        ModelInfo.output_attrs = NULL;
    }
    std::cout << "destroy modelInference!" << std::endl;
}