#include "idetector.h"

#include "model.h"
#include "utils.h"
#include "preprocess.h"
#include "postprocess.h"
#include "cuda_utils.h"
#include "logging.h"
#include <fstream>
const int kOutputSize = kMaxNumOutputBbox * sizeof(Detection) / sizeof(float) + 1;
Logger gLogger;

void deserialize_engine(std::string &engine_name, IRuntime **runtime, ICudaEngine **engine, IExecutionContext **context) {
    std::ifstream file(engine_name, std::ios::binary);
    if (!file.good()) {
        std::cerr << "read " << engine_name << " error!" << std::endl;
        assert(false);
    }
    size_t size = 0;
    file.seekg(0, file.end);
    size = file.tellg();
    file.seekg(0, file.beg);
    char *serialized_engine = new char[size];
    assert(serialized_engine);
    file.read(serialized_engine, size);
    file.close();

    *runtime = createInferRuntime(gLogger);
    assert(*runtime);
    *engine = (*runtime)->deserializeCudaEngine(serialized_engine, size);
    assert(*engine);
    *context = (*engine)->createExecutionContext();
    assert(*context);
    delete[] serialized_engine;
}

void prepare_buffer(ICudaEngine *engine, float **input_buffer_device, float **output_buffer_device,
                    float **output_buffer_host, float **decode_ptr_host, float **decode_ptr_device, std::string cuda_post_process) {
    assert(engine->getNbBindings() == 2);
    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    const int inputIndex = engine->getBindingIndex(kInputTensorName);
    const int outputIndex = engine->getBindingIndex(kOutputTensorName);
    assert(inputIndex == 0);
    assert(outputIndex == 1);
    // Create GPU buffers on device
    CUDA_CHECK(cudaMalloc((void **) input_buffer_device, kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
    CUDA_CHECK(cudaMalloc((void **) output_buffer_device, kBatchSize * kOutputSize * sizeof(float)));
    if (cuda_post_process == "c") {
        *output_buffer_host = new float[kBatchSize * kOutputSize];
    } else if (cuda_post_process == "g") {
        if (kBatchSize > 1) {
            std::cerr << "Do not yet support GPU post processing for multiple batches" << std::endl;
            exit(0);
        }
        // Allocate memory for decode_ptr_host and copy to device
        *decode_ptr_host = new float[1 + kMaxNumOutputBbox * bbox_element];
        CUDA_CHECK(cudaMalloc((void **)decode_ptr_device, sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element)));
    }
}

void infer(IExecutionContext &context, cudaStream_t &stream, void **buffers, float *output, int batchsize, float* decode_ptr_host, float* decode_ptr_device, int model_bboxes, std::string cuda_post_process) {
    // infer on the batch asynchronously, and DMA output back to host
    auto start = std::chrono::system_clock::now();
    context.enqueue(batchsize, buffers, stream, nullptr);
    if (cuda_post_process == "c") {
        CUDA_CHECK(cudaMemcpyAsync(output, buffers[1], batchsize * kOutputSize * sizeof(float), cudaMemcpyDeviceToHost,stream));
        auto end = std::chrono::system_clock::now();
        std::cout << "inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
    } else if (cuda_post_process == "g") {
        CUDA_CHECK(cudaMemsetAsync(decode_ptr_device, 0, sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element), stream));
        cuda_decode((float *)buffers[1], model_bboxes, kConfThresh, decode_ptr_device, kMaxNumOutputBbox, stream);
        cuda_nms(decode_ptr_device, kNmsThresh, kMaxNumOutputBbox, stream);//cuda nms
        CUDA_CHECK(cudaMemcpyAsync(decode_ptr_host, decode_ptr_device, sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element), cudaMemcpyDeviceToHost, stream));
        auto end = std::chrono::system_clock::now();
        std::cout << "inference and gpu postprocess time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
    }

    CUDA_CHECK(cudaStreamSynchronize(stream));
}

idetector::idetector(const std::string& enginepath)
{
    runtime = nullptr;
    engine = nullptr;
    context = nullptr;

    output_buffer_host = nullptr;
    decode_ptr_host=nullptr;
    decode_ptr_device=nullptr;

    engine_name = enginepath;
    cuda_post_process = "g"; //must use g, c is bad
}

idetector::~idetector()
{
    // Release stream and buffers
    cudaStreamDestroy(stream);
    CUDA_CHECK(cudaFree(device_buffers[0]));
    CUDA_CHECK(cudaFree(device_buffers[1]));
    CUDA_CHECK(cudaFree(decode_ptr_device));
    delete[] decode_ptr_host;
    delete[] output_buffer_host;
    cuda_preprocess_destroy();
    // Destroy the engine
    delete context;
    delete engine;
    delete runtime;
}

void idetector::init()
{
    cudaSetDevice(kGpuId);
    // Deserialize the engine from file
    deserialize_engine(engine_name, &runtime, &engine, &context);
    CUDA_CHECK(cudaStreamCreate(&stream));
    cuda_preprocess_init(kMaxInputImageSize);
    auto out_dims = engine->getBindingDimensions(1);
    model_bboxes = out_dims.d[0];

    prepare_buffer(engine, &device_buffers[0], &device_buffers[1], &output_buffer_host, &decode_ptr_host, &decode_ptr_device, cuda_post_process);

}

void idetector::process(cv::Mat& img)
{
    std::vector<cv::Mat> img_batch;
    img_batch.push_back(img);

    // Preprocess
    cuda_batch_preprocess(img_batch, device_buffers[0], kInputW, kInputH, stream);
    // Run inference
    infer(*context, stream, (void **)device_buffers, output_buffer_host, kBatchSize, decode_ptr_host, decode_ptr_device, model_bboxes, cuda_post_process);
    std::vector<std::vector<Detection>> res_batch;
    if (cuda_post_process == "c") {
        // NMS
        batch_nms(res_batch, output_buffer_host, img_batch.size(), kOutputSize, kConfThresh, kNmsThresh);
    } else if (cuda_post_process == "g") {
        //Process gpu decode and nms results
        batch_process(res_batch, decode_ptr_host, img_batch.size(), bbox_element, img_batch);
    }
    // Draw bounding boxes
    draw_bbox(img_batch, res_batch);

}

void idetector::process(cv::Mat& img, std::vector<bbox_t>& boxs)
{
    std::vector<cv::Mat> img_batch;
    img_batch.push_back(img);

    // Preprocess
    cuda_batch_preprocess(img_batch, device_buffers[0], kInputW, kInputH, stream);
    // Run inference
    infer(*context, stream, (void **)device_buffers, output_buffer_host, kBatchSize, decode_ptr_host, decode_ptr_device, model_bboxes, cuda_post_process);
    std::vector<std::vector<Detection>> res_batch;
    if (cuda_post_process == "c") {
        // NMS
        batch_nms(res_batch, output_buffer_host, img_batch.size(), kOutputSize, kConfThresh, kNmsThresh);
    } else if (cuda_post_process == "g") {
        //Process gpu decode and nms results
        batch_process(res_batch, decode_ptr_host, img_batch.size(), bbox_element, img_batch);
    }
    // Draw bounding boxes
    draw_bbox(img_batch, res_batch);

    boxs.clear();
    for(auto& res:res_batch[0])
    {
        cv::Rect r = get_rect(img, res.bbox);
        bbox_t box;
        box.x = r.x;
        box.y = r.y;
        box.w = r.width;
        box.h = r.height;
        box.prob = res.conf;
        box.obj_id = res.class_id;

        // printf("idetector res-->res.bbox[0]:%f,res.bbox[1]:%f,res.bbox[2]:%f,res.bbox[3]:%f\n", res.bbox[0],res.bbox[1],res.bbox[2],res.bbox[3]);
        // printf("idetector box-->box.x:%d,box.y:%d,box.w:%d,box.h:%d\n", box.x,box.y,box.w,box.h);

        boxs.emplace_back(box);
    }

}
void idetector::processNoDraw(cv::Mat& img, std::vector<bbox_t>& boxs)
{
    std::vector<cv::Mat> img_batch;
    img_batch.push_back(img);

    // Preprocess
    cuda_batch_preprocess(img_batch, device_buffers[0], kInputW, kInputH, stream);
    // Run inference
    infer(*context, stream, (void **)device_buffers, output_buffer_host, kBatchSize, decode_ptr_host, decode_ptr_device, model_bboxes, cuda_post_process);
    std::vector<std::vector<Detection>> res_batch;
    if (cuda_post_process == "c") {
        // NMS
        batch_nms(res_batch, output_buffer_host, img_batch.size(), kOutputSize, kConfThresh, kNmsThresh);
    } else if (cuda_post_process == "g") {
        //Process gpu decode and nms results
        batch_process(res_batch, decode_ptr_host, img_batch.size(), bbox_element, img_batch);
    }
    // Draw bounding boxes
    // draw_bbox(img_batch, res_batch);

    boxs.clear();
    for(auto& res:res_batch[0])
    {
        cv::Rect r = get_rect(img, res.bbox);
        bbox_t box;
        box.x = r.x;
        box.y = r.y;
        box.w = r.width;
        box.h = r.height;
        box.prob = res.conf;
        box.obj_id = res.class_id;

        // printf("idetector res-->res.bbox[0]:%f,res.bbox[1]:%f,res.bbox[2]:%f,res.bbox[3]:%f\n", res.bbox[0],res.bbox[1],res.bbox[2],res.bbox[3]);
        // printf("idetector box-->box.x:%d,box.y:%d,box.w:%d,box.h:%d\n", box.x,box.y,box.w,box.h);

        boxs.emplace_back(box);
    }

}