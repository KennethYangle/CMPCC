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
    // std::string input_image_path = "/home/nvidia/record/extracted_images/split_image_1734536070.29.jpg";  //这个是我保存的分割后的图像
    std::string input_image_path = "/home/nvidia/record/extracted_images/frame0071.jpg";
    cv::Mat img = cv::imread(input_image_path);
    if (img.empty()) {
        std::cerr << "Error: Unable to load image: " << input_image_path << std::endl;
        return -1;
    }

    // std::cout << "Image size: " << img.cols << "x" << img.rows << std::endl;
    // std::cout << "Image type: " << img.type() << std::endl;

    // switch (img.type()) {
    //     case CV_8UC1: std::cout << "Type: CV_8UC1 (8-bit single channel)" << std::endl; break;
    //     case CV_8UC3: std::cout << "Type: CV_8UC3 (8-bit 3 channels)" << std::endl; break;
    //     case CV_32FC1: std::cout << "Type: CV_32FC1 (32-bit floating point single channel)" << std::endl; break;
    //     // 添加其他类型的检查
    //     default: std::cout << "Unknown type" << std::endl; break;
    // }

    // Preprocess image
    std::vector<cv::Mat> img_batch;
    img_batch.push_back(img);

        // std::cout << "Imageba size: " << img_batch[0].cols << "x" << img_batch[0].rows << std::endl;
        // std::cout << "Imageba type: " << img_batch[0].type() << std::endl;

        // switch (img_batch[0].type()) {
        //     case CV_8UC1: std::cout << "Type: CV_8UC1 (8-bit single channel)" << std::endl; break;
        //     case CV_8UC3: std::cout << "Type: CV_8UC3 (8-bit 3 channels)" << std::endl; break;
        //     case CV_32FC1: std::cout << "Type: CV_32FC1 (32-bit floating point single channel)" << std::endl; break;
        // // 添加其他类型的检查
        //     default: std::cout << "Unknown type" << std::endl; break;
        // }

    // cv::imshow("Pre Image", img_batch[0]);
    // std::cout << "buffers: " << gpu_buffers[0]<< std::endl;
    // std::cout << "kInputW: " << kInputW<< std::endl;
    // std::cout << "kInputH: " << kInputH<< std::endl;

    cuda_batch_preprocess(img_batch, gpu_buffers[0], kInputW, kInputH, stream);

    // // 假设 kInputW 和 kInputH 是预处理后的图像宽度和高度，3 是通道数
    // int input_width = kInputW;
    // int input_height = kInputH;
    // int input_channels = 3;  // 通道数，假设为 RGB

    // // 数据大小计算（浮点数存储，每像素 3 通道）
    // int input_size = input_width * input_height * input_channels;

    // // 创建主机缓冲区
    // std::vector<float> host_input(input_size);

    // // 从 GPU 缓冲区拷贝数据到主机
    // cudaMemcpy(host_input.data(), gpu_buffers[0], input_size * sizeof(float), cudaMemcpyDeviceToHost);

    // // 输出预处理数据的大小和类型
    // std::cout << "Preprocessed data size: " << input_width << " x " << input_height << " x " << input_channels << std::endl;
    // std::cout << "Data type: float32" << std::endl;

    // // 将预处理数据转换为 OpenCV 图像
    // cv::Mat preprocessed_image(input_height, input_width, CV_32FC3, host_input.data());

    // // 转换为 8 位图像以便显示
    // cv::Mat display_image;
    // preprocessed_image.convertTo(display_image, CV_8UC3, 255.0);

    // // 显示图像
    // cv::imshow("Preprocessed Image", display_image);
    // cv::waitKey(0);


    // Run inference
    infer(*context, stream, (void**)gpu_buffers, cpu_output_buffer, kBatchSize);

    // NMS
    std::vector<std::vector<Detection>> res_batch;
    batch_nms(res_batch, cpu_output_buffer, img_batch.size(), kOutputSize, kConfThresh, kNmsThresh);

    // Draw bounding boxes
    auto& res = res_batch[0];
    // 输出结果
    for (const auto& det : res) {
        std::cout << "Class: " << det.class_id << ", Conf: " << det.conf << ", BBox: ["
                << det.bbox[0] << ", " << det.bbox[1] << ", "
                << det.bbox[2] << ", " << det.bbox[3] << "]\n";
    }

    // for (size_t j = 0; j < res.size(); j++) {
    //     cv::Rect r = get_rect(img, res[j].bbox);
    //     cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
    //     cv::putText(img, "Target: " + std::to_string(res[j].conf), cv::Point(r.x, r.y - 1), 
    //                 cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
    // }

    // // Display output image
    // cv::imshow("Detection Results", img);

    // // 等待用户按键
    // cv::waitKey(0);


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

