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
    img = rotateImage(img , 180);
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
          cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
          cv::putText(img, "ballon"+std::to_string(res[j].conf), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
          if(res[j].conf>.40){
            circle_boxes.bounding_boxes.push_back(msg_BoundingBox);
          } 
          break;
        
      }
    }

    pub_circle.publish(circle_boxes);
auto end = std::chrono::system_clock::now();
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

  image_transport::ImageTransport it(nh);
  pub_circle = nh.advertise<swarm_msgs::BoundingBoxes>("/tracker/pos_image", 1);
  image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 1, imageCallback);
  
  ros::spin();
}

