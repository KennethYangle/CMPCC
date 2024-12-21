#include <opencv2/opencv.hpp>
#include <iostream>
#include "cuda_utils.h"
#include "logging.h"
#include "utils.h"
#include "preprocess.h"
#include "postprocess.h"
#include "model.h"
#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include <typeinfo>
#include <ctime>
#include <swarm_msgs/BoundingBoxes.h>
#include <swarm_msgs/BoundingBox.h>

using namespace nvinfer1;

static Logger gLogger;
const static int kOutputSize = kMaxNumOutputBbox * sizeof(Detection) / sizeof(float) + 1;
std::string engine_name = "/home/nvidia/yolo_weights/ballon.engine";

IRuntime* runtime = nullptr;
ICudaEngine* engine = nullptr;
IExecutionContext* context = nullptr;
cudaStream_t stream;
float* gpu_buffers[2];  
float* cpu_output_buffer = nullptr;

void prepare_buffers(ICudaEngine* engine, float** gpu_input_buffer, float** gpu_output_buffer, float** cpu_output_buffer) {
    assert(engine->getNbBindings() == 2);
    const int inputIndex = engine->getBindingIndex(kInputTensorName);
    const int outputIndex = engine->getBindingIndex(kOutputTensorName);
    assert(inputIndex == 0);
    assert(outputIndex == 1);
    CUDA_CHECK(cudaMalloc((void**)gpu_input_buffer, kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
    CUDA_CHECK(cudaMalloc((void**)gpu_output_buffer, kBatchSize * kOutputSize * sizeof(float)));
    *cpu_output_buffer = new float[kBatchSize * kOutputSize];
}

void infer(IExecutionContext& context, cudaStream_t& stream, void** gpu_buffers, float* output, int batchsize) {
    context.enqueue(batchsize, gpu_buffers, stream, nullptr);
    CUDA_CHECK(cudaMemcpyAsync(output, gpu_buffers[1], batchsize * kOutputSize * sizeof(float), cudaMemcpyDeviceToHost, stream));
    cudaStreamSynchronize(stream);
}

void deserialize_engine(std::string& engine_name, IRuntime** runtime, ICudaEngine** engine, IExecutionContext** context) {
    std::ifstream file(engine_name, std::ios::binary);
    if (!file.good()) {
        std::cerr << "read " << engine_name << " error!" << std::endl;
        assert(false);
    }
    size_t size = 0;
    file.seekg(0, file.end);
    size = file.tellg();
    file.seekg(0, file.beg);
    char* serialized_engine = new char[size];
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

std::pair<std::vector<cv::Mat>, cv::Mat> split_image(const cv::Mat& img, int horizontal_overlap[4], int vertical_overlap) {
    std::vector<cv::Mat> split_images;

    // 图像的宽度和高度
    int img_width = img.cols;
    int img_height = img.rows;

    // 横向分割：根据给定的重叠信息计算每个区域的宽度和位置
    int horizontal_splits = 640;

    // 根据重叠的要求调整每个分割的起始位置
    int x_start[4];
    x_start[0] = 0;
    x_start[1] = horizontal_splits - horizontal_overlap[0];  // 重叠 213 像素
    x_start[2] = x_start[1] + horizontal_splits - horizontal_overlap[1]; // 重叠 213 像素
    x_start[3] = x_start[2] + horizontal_splits - horizontal_overlap[2]; // 重叠 214 像素

    int height = 640;

    // 纵向分割：每个分割区域的高度
    int y_start[2];
    y_start[0] = 0;
    y_start[1] = height - vertical_overlap;  // 第二部分的起始位置

    // 初始化 pos_start 矩阵，存储每个分割的起始坐标
    cv::Mat pos_start = cv::Mat_<cv::Vec2i>(2, 4);  // 2行4列的矩阵，每个元素是一个 Vec2i

    // 进行分割操作
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 4; ++j) {
            // 存储每个分割区域的左上角坐标 (x_start[j], y_start[i])
            pos_start.at<cv::Vec2i>(i, j) = cv::Vec2i(x_start[j], y_start[i]);  // 存储 (x, y) 坐标

            // 计算每个分割区域的矩形框
            cv::Rect roi(x_start[j], y_start[i], horizontal_splits, height);
            cv::Mat cropped_img = img(roi);  // 从原始图像中截取出对应区域
            split_images.push_back(cropped_img);  // 存储分割的图像
        }
    }

    // 返回结果：包含分割图像和 pos_start 矩阵
    return {split_images, pos_start};
}


int main(int argc, char** argv) {

    // Set device
    cudaSetDevice(kGpuId);

    // Deserialize the engine
    deserialize_engine(engine_name, &runtime, &engine, &context);
    CUDA_CHECK(cudaStreamCreate(&stream));

    // Init CUDA preprocessing
    cuda_preprocess_init(kMaxInputImageSize);

    // Prepare buffers
    prepare_buffers(engine, &gpu_buffers[0], &gpu_buffers[1], &cpu_output_buffer);

    // Load input image
    // std::string input_image_path = "/home/nvidia/record/extracted_images/split_image_1734536070.29.jpg";
    std::string input_image_path = "/home/nvidia/record/extracted_images/frame0071.jpg";  //这个是分割前的图像
    cv::Mat img = cv::imread(input_image_path);
    if (img.empty()) {
        std::cerr << "Error: Unable to load image: " << input_image_path << std::endl;
        return -1;
    }

    // 定义横向分割的重叠值
    int horizontal_overlap[4] = {213, 213, 214, 0};
    // 定义纵向的重叠值
    int vertical_overlap = 200;  // 纵向重叠 200 像素

    // // 声明并初始化split_images为一个包含cv::Mat类型元素的vector
    std::vector<cv::Mat> split_images;
    cv::Mat pos_start;
    
    // 调用split_image函数，获得分割的图像和起始位置矩阵
    std::tie(split_images, pos_start) = split_image(img, horizontal_overlap, vertical_overlap);

    // 显示分割后的图像
    for (size_t i = 0; i < split_images.size(); ++i) {
        cv::Mat imgi = split_images[i];
        //////////////////////////////////////////////////////////////
        // Preprocess image
        std::vector<cv::Mat> img_batch;
        img_batch.clear();

        std::cout << "Image size: " << imgi.cols << "x" << imgi.rows << std::endl;
        std::cout << "Image type: " << imgi.type() << std::endl;

        img_batch.push_back(imgi);  // 使用当前分割图像
        cuda_batch_preprocess(img_batch, gpu_buffers[0], kInputW, kInputH, stream);

        // Run inference
        infer(*context, stream, (void**)gpu_buffers, cpu_output_buffer, kBatchSize);

        // NMS
        std::vector<std::vector<Detection>> res_batch;
        batch_nms(res_batch, cpu_output_buffer, img_batch.size(), kOutputSize, kConfThresh, kNmsThresh);

        // Draw bounding boxes
        auto& res = res_batch[0];
        
        for (size_t j = 0; j < res.size(); j++) {
            // 获取目标框
            cv::Rect r = get_rect(imgi, res[j].bbox);
            std::cout<<j<<":"<<res[j].conf<<std::endl;
            // 在分割图像上绘制矩形框
            cv::rectangle(imgi, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
            cv::putText(imgi, "Target: " + std::to_string(res[j].conf), cv::Point(r.x, r.y - 1), 
                        cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
        }
        //////////////////////////////////////////////////////////////
        
        // 显示每个分割后的图像
        std::string window_name = "Split Image " + std::to_string(i);
        cv::imshow(window_name, imgi);
        // 等待用户按键
        cv::waitKey(20000);
        std::cout << "Displaying: " << window_name << std::endl;
    }

    

    // Cleanup
    CUDA_CHECK(cudaFree(gpu_buffers[0]));
    CUDA_CHECK(cudaFree(gpu_buffers[1]));
    delete[] cpu_output_buffer;
    cudaStreamDestroy(stream);
    context->destroy();
    engine->destroy();
    runtime->destroy();

    return 0;
}