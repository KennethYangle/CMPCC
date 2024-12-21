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

void publish_image(const cv::Mat& img, image_transport::Publisher& pub, ros::Time timestamp) {
    try {
        // 将 OpenCV 图像转换为 ROS 图像消息
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        msg->header.stamp = timestamp;  // 设置时间戳
        pub.publish(msg);  // 发布图像消息
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {

    // // 初始化 ROS
    // ros::init(argc, argv, "image_publisher_node");
    // ros::NodeHandle nh;
    // image_transport::ImageTransport it(nh);

    // // 创建一个发布者，话题名称为 "/split_images"
    // image_transport::Publisher pub = it.advertise("/split_images", 1);
    
    // Load input image
    // std::string input_image_path = "/home/nvidia/record/extracted_images/split_image_1734536070.29.jpg";
    std::string input_image_path = "/home/nvidia/record/extracted_images/frame0071.jpg";
    cv::Mat img = cv::imread(input_image_path);
    if (img.empty()) {
        std::cerr << "Error: Unable to load image: " << input_image_path << std::endl;
        return -1;
    }

    // // Preprocess image
    // std::vector<cv::Mat> img_batch;
    // img_batch.push_back(img);
    // cuda_batch_preprocess(img_batch, gpu_buffers[0], kInputW, kInputH, stream);

    // // Run inference
    // infer(*context, stream, (void**)gpu_buffers, cpu_output_buffer, kBatchSize);

    // // NMS
    // std::vector<std::vector<Detection>> res_batch;
    // batch_nms(res_batch, cpu_output_buffer, img_batch.size(), kOutputSize, kConfThresh, kNmsThresh);

    // // Draw bounding boxes
    // auto& res = res_batch[0];
    // for (size_t j = 0; j < res.size(); j++) {
    //     cv::Rect r = get_rect(img, res[j].bbox);
    //     cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
    //     cv::putText(img, "Target: " + std::to_string(res[j].conf), cv::Point(r.x, r.y - 1), 
    //                 cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
    // }

    // // Display output image
    // cv::imshow("Detection Results", img);

    // 定义横向分割的重叠值
    int horizontal_overlap[4] = {213, 213, 214, 0};
    // 定义纵向的重叠值
    int vertical_overlap = 200;  // 纵向重叠 200 像素

    // // 声明并初始化split_images为一个包含cv::Mat类型元素的vector
    std::vector<cv::Mat> split_images;
    cv::Mat pos_start;
    
    // 调用split_image函数，获得分割的图像和起始位置矩阵
    std::tie(split_images, pos_start) = split_image(img, horizontal_overlap, vertical_overlap);

    // // 调用分割函数
    // auto [split_images, pos_start] = split_image(img, horizontal_overlap, vertical_overlap);

    // Set device
    cudaSetDevice(kGpuId);

    // Deserialize the engine
    deserialize_engine(engine_name, &runtime, &engine, &context);
    CUDA_CHECK(cudaStreamCreate(&stream));

    // Init CUDA preprocessing
    cuda_preprocess_init(kMaxInputImageSize);

    // Prepare buffers
    prepare_buffers(engine, &gpu_buffers[0], &gpu_buffers[1], &cpu_output_buffer);

    // 显示分割后的图像
    // for (size_t i = 1; i < split_images.size(); ++i) {
        int i = 1;

        // cv::Mat img_i = split_images[i];
        // cv::imshow("Pre0 Image", img_i);

        cv::Mat imgi = split_images[i];
        cv::imshow("Pre0 Image", imgi);

        // // 假设你希望将图像的像素数据存储为一个字节数组
        // std::vector<uchar> img_data(img_i.total() * img_i.elemSize());  // 创建一个缓冲区

        // // 获取图像的总像素数和每个像素的字节大小
        // size_t total_pixels = img_i.total();  // 图像总像素数
        // size_t element_size = img_i.elemSize();  // 每个像素的字节大小

        // // 输出结果
        // std::cout << "Total pixels: " << total_pixels << std::endl;
        // std::cout << "Element size (bytes per pixel): " << element_size << std::endl;

        // // 输出总字节数 (total pixels * element size)
        // std::cout << "Total memory size (in bytes): " << total_pixels * element_size << std::endl;

        // // 将图像的数据拷贝到 img_data 中
        // std::memcpy(img_data.data(), img_i.data, img_data.size());

        // // 如果需要，可以重新从缓冲区创建 Mat
        // cv::Mat imgi = cv::Mat(img_i.size(), img_i.type(), img_i.data);

        // cv::imshow("Pre1 Image", imgi);

	    // printf("A.data = %p\nB.data = %p\nC.data = %p\n", split_images[i], imgi.data, img_i.data);
        printf("A.data = %p\nB.data = %p\n", split_images[i], imgi.data);


        //////////////////////////////////////////////////////////////
        // Preprocess image
        std::vector<cv::Mat> img_batch;
        img_batch.clear();

        std::cout << "Image size: " << imgi.cols << "x" << imgi.rows << std::endl;
        std::cout << "Image type: " << imgi.type() << std::endl;

        switch (imgi.type()) {
            case CV_8UC1: std::cout << "Type: CV_8UC1 (8-bit single channel)" << std::endl; break;
            case CV_8UC3: std::cout << "Type: CV_8UC3 (8-bit 3 channels)" << std::endl; break;
            case CV_32FC1: std::cout << "Type: CV_32FC1 (32-bit floating point single channel)" << std::endl; break;
        // 添加其他类型的检查
            default: std::cout << "Unknown type" << std::endl; break;
        }

        img_batch.push_back(imgi);  // 使用当前分割图像

        std::cout << "Imageba size: " << img_batch[0].cols << "x" << img_batch[0].rows << std::endl;
        std::cout << "Imageba type: " << img_batch[0].type() << std::endl;

        switch (img_batch[0].type()) {
            case CV_8UC1: std::cout << "Type: CV_8UC1 (8-bit single channel)" << std::endl; break;
            case CV_8UC3: std::cout << "Type: CV_8UC3 (8-bit 3 channels)" << std::endl; break;
            case CV_32FC1: std::cout << "Type: CV_32FC1 (32-bit floating point single channel)" << std::endl; break;
        // 添加其他类型的检查
            default: std::cout << "Unknown type" << std::endl; break;
        }

        cv::imshow("Pre Image", img_batch[0]);
        
        std::cout << "buffers: " << gpu_buffers[0]<< std::endl;
        std::cout << "kInputW: " << kInputW<< std::endl;
        std::cout << "kInputH: " << kInputH<< std::endl;

        cuda_batch_preprocess(img_batch, gpu_buffers[0], kInputW, kInputH, stream);

        // 假设 kInputW 和 kInputH 是预处理后的图像宽度和高度，3 是通道数
        int input_width = kInputW;
        int input_height = kInputH;
        int input_channels = 3;  // 通道数，假设为 RGB

        // 数据大小计算（浮点数存储，每像素 3 通道）
        int input_size = input_width * input_height * input_channels;

        // 创建主机缓冲区
        std::vector<float> host_input(input_size);

        // 从 GPU 缓冲区拷贝数据到主机
        cudaMemcpy(host_input.data(), gpu_buffers[0], input_size * sizeof(float), cudaMemcpyDeviceToHost);

        // 输出预处理数据的大小和类型
        std::cout << "Preprocessed data size: " << input_width << " x " << input_height << " x " << input_channels << std::endl;
        std::cout << "Data type: float32" << std::endl;

        // 将预处理数据转换为 OpenCV 图像
        cv::Mat preprocessed_image(input_height, input_width, CV_32FC3, host_input.data());

        // 转换为 8 位图像以便显示
        cv::Mat display_image;
        preprocessed_image.convertTo(display_image, CV_8UC3, 255.0);

        // 显示图像
        cv::imshow("Preprocessed Image", display_image);
        cv::waitKey(0);

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
    // }


    // Cleanup
    CUDA_CHECK(cudaFree(gpu_buffers[0]));
    CUDA_CHECK(cudaFree(gpu_buffers[1]));
    delete[] cpu_output_buffer;
    cudaStreamDestroy(stream);
    context->destroy();
    engine->destroy();
    runtime->destroy();

    //  // 获取当前时间戳
    // ros::Time timestamp = ros::Time::now();

    // // 发布每个分割后的图像
    // for (size_t i = 0; i < split_images.size(); ++i) {
    //     // 发布图像
    //     publish_image(split_images[i], pub, timestamp);
    //     std::cout << "Publishing split image " << i << std::endl;
    // }

    // // ROS 主循环
    // ros::spin();

    return 0;
}





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

std::tuple<cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat> split_image(const cv::Mat& img, int horizontal_overlap[4], int vertical_overlap) {
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

    // 进行分割操作，直接返回8个cv::Mat
    cv::Mat img1, img2, img3, img4, img5, img6, img7, img8;
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 4; ++j) {
            // 计算每个分割区域的矩形框
            cv::Rect roi(x_start[j], y_start[i], horizontal_splits, height);
            cv::Mat cropped_img = img(roi);  // 从原始图像中截取出对应区域

            // 将每个分割区域分配给对应的变量
            if (i == 0 && j == 0) img1 = cropped_img;
            if (i == 0 && j == 1) img2 = cropped_img;
            if (i == 0 && j == 2) img3 = cropped_img;
            if (i == 0 && j == 3) img4 = cropped_img;
            if (i == 1 && j == 0) img5 = cropped_img;
            if (i == 1 && j == 1) img6 = cropped_img;
            if (i == 1 && j == 2) img7 = cropped_img;
            if (i == 1 && j == 3) img8 = cropped_img;
        }
    }

    // 返回结果，包含8个cv::Mat图像和一个pos_start矩阵
    return {img1, img2, img3, img4, img5, img6, img7, img8};
}


int main(int argc, char** argv) {

    // // 初始化 ROS
    // ros::init(argc, argv, "image_publisher_node");
    // ros::NodeHandle nh;
    // image_transport::ImageTransport it(nh);

    // // 创建一个发布者，话题名称为 "/split_images"
    // image_transport::Publisher pub = it.advertise("/split_images", 1);
    
    // Load input image
    // std::string input_image_path = "/home/nvidia/record/extracted_images/split_image_1734536070.29.jpg";
    std::string input_image_path = "/home/nvidia/record/extracted_images/frame0071.jpg";
    cv::Mat img = cv::imread(input_image_path);
    if (img.empty()) {
        std::cerr << "Error: Unable to load image: " << input_image_path << std::endl;
        return -1;
    }

    // 定义横向分割的重叠值
    int horizontal_overlap[4] = {213, 213, 214, 0};
    // 定义纵向的重叠值
    int vertical_overlap = 200;  // 纵向重叠 200 像素

    auto [img1, imgi, img3, img4, img5, img6, img7, img8] = split_image(img, horizontal_overlap, vertical_overlap);

    // Set device
    cudaSetDevice(kGpuId);

    // Deserialize the engine
    deserialize_engine(engine_name, &runtime, &engine, &context);
    CUDA_CHECK(cudaStreamCreate(&stream));

    // Init CUDA preprocessing
    cuda_preprocess_init(kMaxInputImageSize);

    // Prepare buffers
    prepare_buffers(engine, &gpu_buffers[0], &gpu_buffers[1], &cpu_output_buffer);

    // 显示分割后的图像
    // for (size_t i = 1; i < split_images.size(); ++i) {
        // int i = 1;

        // cv::Mat img_i = split_images[i];
        // cv::imshow("Pre0 Image", img_i);

        // cv::Mat imgi = split_images[i];
        cv::imshow("Pre0 Image", imgi);

        // // 假设你希望将图像的像素数据存储为一个字节数组
        // std::vector<uchar> img_data(img_i.total() * img_i.elemSize());  // 创建一个缓冲区

        // // 获取图像的总像素数和每个像素的字节大小
        // size_t total_pixels = img_i.total();  // 图像总像素数
        // size_t element_size = img_i.elemSize();  // 每个像素的字节大小

        // // 输出结果
        // std::cout << "Total pixels: " << total_pixels << std::endl;
        // std::cout << "Element size (bytes per pixel): " << element_size << std::endl;

        // // 输出总字节数 (total pixels * element size)
        // std::cout << "Total memory size (in bytes): " << total_pixels * element_size << std::endl;

        // // 将图像的数据拷贝到 img_data 中
        // std::memcpy(img_data.data(), img_i.data, img_data.size());

        // // 如果需要，可以重新从缓冲区创建 Mat
        // cv::Mat imgi = cv::Mat(img_i.size(), img_i.type(), img_i.data);

        // cv::imshow("Pre1 Image", imgi);

	    // printf("A.data = %p\nB.data = %p\nC.data = %p\n", split_images[i], imgi.data, img_i.data);
        // printf("A.data = %p\nB.data = %p\n", split_images[i], imgi.data);
        printf("A.data = %p\n",imgi.data);


        //////////////////////////////////////////////////////////////
        // Preprocess image
        std::vector<cv::Mat> img_batch;
        img_batch.clear();

        std::cout << "Image size: " << imgi.cols << "x" << imgi.rows << std::endl;
        std::cout << "Image type: " << imgi.type() << std::endl;

        switch (imgi.type()) {
            case CV_8UC1: std::cout << "Type: CV_8UC1 (8-bit single channel)" << std::endl; break;
            case CV_8UC3: std::cout << "Type: CV_8UC3 (8-bit 3 channels)" << std::endl; break;
            case CV_32FC1: std::cout << "Type: CV_32FC1 (32-bit floating point single channel)" << std::endl; break;
        // 添加其他类型的检查
            default: std::cout << "Unknown type" << std::endl; break;
        }

        img_batch.push_back(imgi);  // 使用当前分割图像

        std::cout << "Imageba size: " << img_batch[0].cols << "x" << img_batch[0].rows << std::endl;
        std::cout << "Imageba type: " << img_batch[0].type() << std::endl;

        switch (img_batch[0].type()) {
            case CV_8UC1: std::cout << "Type: CV_8UC1 (8-bit single channel)" << std::endl; break;
            case CV_8UC3: std::cout << "Type: CV_8UC3 (8-bit 3 channels)" << std::endl; break;
            case CV_32FC1: std::cout << "Type: CV_32FC1 (32-bit floating point single channel)" << std::endl; break;
        // 添加其他类型的检查
            default: std::cout << "Unknown type" << std::endl; break;
        }

        cv::imshow("Pre Image", img_batch[0]);

        std::cout << "buffers: " << gpu_buffers[0]<< std::endl;
        std::cout << "kInputW: " << kInputW<< std::endl;
        std::cout << "kInputH: " << kInputH<< std::endl;

        cuda_batch_preprocess(img_batch, gpu_buffers[0], kInputW, kInputH, stream);

        // 假设 kInputW 和 kInputH 是预处理后的图像宽度和高度，3 是通道数
        int input_width = kInputW;
        int input_height = kInputH;
        int input_channels = 3;  // 通道数，假设为 RGB

        // 数据大小计算（浮点数存储，每像素 3 通道）
        int input_size = input_width * input_height * input_channels;

        // 创建主机缓冲区
        std::vector<float> host_input(input_size);

        // 从 GPU 缓冲区拷贝数据到主机
        cudaMemcpy(host_input.data(), gpu_buffers[0], input_size * sizeof(float), cudaMemcpyDeviceToHost);

        // 输出预处理数据的大小和类型
        std::cout << "Preprocessed data size: " << input_width << " x " << input_height << " x " << input_channels << std::endl;
        std::cout << "Data type: float32" << std::endl;

        // 将预处理数据转换为 OpenCV 图像
        cv::Mat preprocessed_image(input_height, input_width, CV_32FC3, host_input.data());

        // 转换为 8 位图像以便显示
        cv::Mat display_image;
        preprocessed_image.convertTo(display_image, CV_8UC3, 255.0);

        // 显示图像
        cv::imshow("Preprocessed Image", display_image);
        cv::waitKey(0);

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
        std::string window_name = "Split Image " + std::to_string(1);
        cv::imshow(window_name, imgi);
            // 等待用户按键
        cv::waitKey(20000);
        std::cout << "Displaying: " << window_name << std::endl;
    // }


    // Cleanup
    CUDA_CHECK(cudaFree(gpu_buffers[0]));
    CUDA_CHECK(cudaFree(gpu_buffers[1]));
    delete[] cpu_output_buffer;
    cudaStreamDestroy(stream);
    context->destroy();
    engine->destroy();
    runtime->destroy();

    //  // 获取当前时间戳
    // ros::Time timestamp = ros::Time::now();

    // // 发布每个分割后的图像
    // for (size_t i = 0; i < split_images.size(); ++i) {
    //     // 发布图像
    //     publish_image(split_images[i], pub, timestamp);
    //     std::cout << "Publishing split image " << i << std::endl;
    // }

    // // ROS 主循环
    // ros::spin();

    return 0;
}