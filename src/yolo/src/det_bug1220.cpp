/*
 * Bian 2023.08.17
 * target det with yolo v5 7.0, engine must be generated with yolov5s.pt
 * param: KNum=2
 *        Pixel of resized image: (640,640)
 * input: engine file (engine_name in) 
 *        ros topic of image (it.subscribe)
 * output: <swarm_msgs::BoundingBoxes>pub_circle
*/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include "cuda_utils.h"
#include "logging.h"
#include "utils.h"
#include "preprocess.h"
#include "postprocess.h"
#include "model.h"
#include <chrono>
#include <typeinfo>
#include <ctime>
#include <string>
#include <swarm_msgs/BoundingBoxes.h>
#include <swarm_msgs/BoundingBox.h>

using namespace nvinfer1;
// 是否显示检测图像
bool is_show_bbx = false;

static Logger gLogger;
const static int kOutputSize = kMaxNumOutputBbox * sizeof(Detection) / sizeof(float) + 1;
std::string engine_name = "/home/nvidia/yolo_weights/ballon.engine";
// std::string engine_name = "/home/nvidia/yolo_weights/zy.engine";

IRuntime* runtime = nullptr;
ICudaEngine* engine = nullptr;
IExecutionContext* context = nullptr;
cudaStream_t stream;
float* gpu_buffers[2];  
float* cpu_output_buffer = nullptr;

ros::Publisher pub_circle;
int num = 0;
void prepare_buffers(ICudaEngine* engine, float** gpu_input_buffer, float** gpu_output_buffer, float** cpu_output_buffer) {
  assert(engine->getNbBindings() == 2);
  // In order to bind the buffers, we need to know the names of the input and output tensors.
  // Note that indices are guaranteed to be less than IEngine::getNbBindings()
  const int inputIndex = engine->getBindingIndex(kInputTensorName);
  const int outputIndex = engine->getBindingIndex(kOutputTensorName);
  assert(inputIndex == 0);
  assert(outputIndex == 1);
  // Create GPU buffers on device
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

cv::Mat rotateImage(cv::Mat& src_img, int degree) {
    if (degree == 90) {
        cv::Mat srcCopy = cv::Mat(src_img.rows, src_img.cols, src_img.depth());
        cv::transpose(src_img, srcCopy);
        cv::flip(srcCopy, srcCopy, 1);
        return srcCopy;
    } else if (degree == 180) {
        cv::Mat srcCopy = cv::Mat(src_img.rows, src_img.cols, src_img.depth());
        cv::flip(src_img, srcCopy, -1);
        return srcCopy;
    } else if (degree == 270) {
        cv::Mat srcCopy = cv::Mat(src_img.rows, src_img.cols, src_img.depth());
        cv::transpose(src_img, srcCopy);
        cv::flip(srcCopy, srcCopy, 0);
        return srcCopy;
    } else {
        return src_img;
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
 num ++;
 if (num%1==0){
 try
  { // Get image
    auto start = std::chrono::system_clock::now();
    auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    std::vector<cv::Mat> img_batch;
    cv::Mat img = cv_ptr->image;
    // img = rotateImage(img , 180);
    img_batch.push_back(img);
    
    //Preprocess
    cuda_batch_preprocess(img_batch, gpu_buffers[0], kInputW, kInputH, stream);

    // Run inference
    
//  std::cout<<"ready run"<<std::endl;
    infer(*context, stream, (void**)gpu_buffers, cpu_output_buffer, kBatchSize);
    
    // NMS
    std::vector<std::vector<Detection>> res_batch;
    batch_nms(res_batch, cpu_output_buffer, img_batch.size(), kOutputSize, kConfThresh, kNmsThresh);
    //draw fuzhuxian
    // std::cout<<img.size().width<<","<<img.size().height<<std::endl;
    double real_image_center_x = img.size().width/2;
    double real_image_center_y = img.size().height/2;
    
    // Draw bounding boxes
    auto& res = res_batch[0];
    swarm_msgs::BoundingBoxes circle_boxes;
    for (size_t j = 0; j < res.size(); j++) {
      cv::Rect r = get_rect(img, res[j].bbox);
      swarm_msgs::BoundingBox msg_BoundingBox;
      msg_BoundingBox.probability=res[j].conf;
      msg_BoundingBox.xmin=(r.x);
      msg_BoundingBox.xmax=(r.x+r.width);
      msg_BoundingBox.ymin=(r.y);
      msg_BoundingBox.ymax=(r.y+r.height); 

      switch((int)res[j].class_id)
      {
        case 0: 
          if (is_show_bbx) {
            cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
            cv::putText(img, "ballon"+std::to_string(res[j].conf), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
          }
          if(res[j].conf>.40){
            circle_boxes.bounding_boxes.push_back(msg_BoundingBox);
          } 
          break;
        
      }
    }

    if (is_show_bbx) {
      cv::imshow("yolo", img);
      cv::waitKey(1);
    }

    pub_circle.publish(circle_boxes);
    // auto end = std::chrono::system_clock::now();
    // std::cout << "run time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
}

int main(int argc, char **argv)
{
  cudaSetDevice(kGpuId);
  // Deserialize the engine from file
  deserialize_engine(engine_name, &runtime, &engine, &context);
  CUDA_CHECK(cudaStreamCreate(&stream));
  // Init CUDA preprocessing
  cuda_preprocess_init(kMaxInputImageSize);
  // Prepare cpu and gpu buffers
  prepare_buffers(engine, &gpu_buffers[0], &gpu_buffers[1], &cpu_output_buffer);

  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  nh.param("/Debug/is_show_bbx", is_show_bbx, false);
  ROS_INFO("is_show_bbx: %d", is_show_bbx);

  image_transport::ImageTransport it(nh);
  pub_circle = nh.advertise<swarm_msgs::BoundingBoxes>("/tracker/pos_image", 1);
  image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 1, imageCallback);
  
  ros::spin();
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
    std::string input_image_path = "/home/nvidia/record/extracted_images/frame0066.jpg";
    cv::Mat img = cv::imread(input_image_path);
    if (img.empty()) {
        std::cerr << "Error: Unable to load image: " << input_image_path << std::endl;
        return -1;
    }

    // Preprocess image
    std::vector<cv::Mat> img_batch;
    img_batch.push_back(img);
    cuda_batch_preprocess(img_batch, gpu_buffers[0], kInputW, kInputH, stream);

    // Run inference
    infer(*context, stream, (void**)gpu_buffers, cpu_output_buffer, kBatchSize);

    // NMS
    std::vector<std::vector<Detection>> res_batch;
    batch_nms(res_batch, cpu_output_buffer, img_batch.size(), kOutputSize, kConfThresh, kNmsThresh);

    // Draw bounding boxes
    auto& res = res_batch[0];
    for (size_t j = 0; j < res.size(); j++) {
        cv::Rect r = get_rect(img, res[j].bbox);
        cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
        cv::putText(img, "Target: " + std::to_string(res[j].conf), cv::Point(r.x, r.y - 1), 
                    cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
    }

    // Display output image
    cv::imshow("Detection Results", img);
    cv::waitKey(0);

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

#include <opencv2/opencv.hpp>
#include <iostream>
#include "cuda_utils.h"
#include "logging.h"
#include "utils.h"
#include "preprocess.h"
#include "postprocess.h"
#include "model.h"
#include <string>

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
    std::string input_image_path = "/home/nvidia/record/extracted_images/frame0066.jpg";
    cv::Mat img = cv::imread(input_image_path);
    if (img.empty()) {
        std::cerr << "Error: Unable to load image: " << input_image_path << std::endl;
        return -1;
    }

    // Preprocess image
    std::vector<cv::Mat> img_batch;
    img_batch.push_back(img);
    cuda_batch_preprocess(img_batch, gpu_buffers[0], kInputW, kInputH, stream);

    // Run inference
    infer(*context, stream, (void**)gpu_buffers, cpu_output_buffer, kBatchSize);

    // NMS
    std::vector<std::vector<Detection>> res_batch;
    batch_nms(res_batch, cpu_output_buffer, img_batch.size(), kOutputSize, kConfThresh, kNmsThresh);

    // Draw bounding boxes
    auto& res = res_batch[0];
    for (size_t j = 0; j < res.size(); j++) {
        cv::Rect r = get_rect(img, res[j].bbox);
        cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
        cv::putText(img, "Target: " + std::to_string(res[j].conf), cv::Point(r.x, r.y - 1), 
                    cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
    }

    // Display output image
    cv::imshow("Detection Results", img);
    cv::waitKey(0);

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


#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include "cuda_utils.h"
#include "logging.h"
#include "utils.h"
#include "preprocess.h"
#include "postprocess.h"
#include "model.h"
#include <string>

std::vector<cv::Mat> split_image(const cv::Mat& img, int horizontal_overlap[4], int vertical_overlap) {
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

    // 进行分割操作
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 4; ++j) {
            // 计算每个分割区域的矩形框
            cv::Rect roi(x_start[j], y_start[i], horizontal_splits, height);
            cv::Mat cropped_img = img(roi);  // 从原始图像中截取出对应区域
            split_images.push_back(cropped_img);  // 存储分割的图像
        }
    }

    return split_images;
}

int main() {
    std::cout << "0" << std::endl;
    // 读取原始图像
    std::string input_image_path = "/home/nvidia/record/extracted_images/frame0066.jpg"; // 替换为图像路径
    std::cout << "0.5" << std::endl;
    cv::Mat img = cv::imread(input_image_path);
    std::cout << "1" << std::endl;

    if (img.empty()) {
        std::cerr << "Error: Unable to load image!" << std::endl;
        return -1;
    }

    // 定义横向分割的重叠值
    int horizontal_overlap[4] = {213, 213, 214, 0};  // 重叠值：前三个之间213像素，最后一个与前面没有重叠
    std::cout << "2" << std::endl;

    // 定义纵向的重叠值
    int vertical_overlap = 200;  // 纵向重叠 200 像素
    std::cout << "3" << std::endl;

    // 调用分割函数
    std::vector<cv::Mat> split_images = split_image(img, horizontal_overlap, vertical_overlap);
    std::cout << "4" << std::endl;

    // 显示分割后的图像
    for (size_t i = 0; i < split_images.size(); ++i) {
        // 显示每个分割后的图像
        std::string window_name = "Split Image " + std::to_string(i);
        cv::imshow(window_name, split_images[i]);
        std::cout << "Displaying: " << window_name << std::endl;
    }

    // 等待按键退出
    cv::waitKey(0);

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

// void merge_results(std::vector<Detection>& all_detections, const std::vector<std::vector<Detection>>& split_results, const cv::Mat& pos_start) {
//     // 遍历每个分割区域的检测结果
//     for (int i = 0; i < split_results.size(); ++i) {
//         int split_idx = i / 4;  // 计算当前分割区域的纵向位置
//         int x_offset = pos_start.at<cv::Vec2i>(split_idx, i % 4)[0];  // 横向偏移量
//         int y_offset = pos_start.at<cv::Vec2i>(split_idx, i % 4)[1];  // 纵向偏移量

//         for (const auto& det : split_results[i]) {
//             // 通过 pos_start 中的偏移量调整每个检测框的坐标
//             Detection merged_det = det;
//             merged_det.bbox[0] += x_offset;  // 横向偏移
//             merged_det.bbox[1] += y_offset;  // 纵向偏移
//             all_detections.push_back(merged_det);
//         }
//     }
// }

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
    std::string input_image_path = "/home/nvidia/record/extracted_images/frame0066.jpg";
    cv::Mat img = cv::imread(input_image_path);
    if (img.empty()) {
        std::cerr << "Error: Unable to load image: " << input_image_path << std::endl;
        return -1;
    }

    // Preprocess image
    std::vector<cv::Mat> img_batch;
    img_batch.push_back(img);
    cuda_batch_preprocess(img_batch, gpu_buffers[0], kInputW, kInputH, stream);

    // Run inference
    infer(*context, stream, (void**)gpu_buffers, cpu_output_buffer, kBatchSize);

    // NMS
    std::vector<std::vector<Detection>> res_batch;
    batch_nms(res_batch, cpu_output_buffer, img_batch.size(), kOutputSize, kConfThresh, kNmsThresh);

    cv::imshow("Detection Results", img);

    // 定义横向分割的重叠值
    int horizontal_overlap[4] = {213, 213, 214, 0};
    // 定义纵向的重叠值
    int vertical_overlap = 200;  // 纵向重叠 200 像素
    // 调用分割函数
    {std::vector<cv::Mat> split_images, cv::Mat pos_start} = split_image(img, horizontal_overlap, vertical_overlap);
    // 显示分割后的图像
    for (size_t i = 0; i < split_images.size(); ++i) {
        // 显示每个分割后的图像
        std::string window_name = "Split Image " + std::to_string(i);
        cv::imshow(window_name, split_images[i]);
        std::cout << "Displaying: " << window_name << std::endl;
    }

    cv::waitKey(0);

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


