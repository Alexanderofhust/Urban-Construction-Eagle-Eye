// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <std_msgs/msg/string.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
// #include <image_transport/image_transport.hpp>
// #include <iomanip>
// #include <sstream>
// #include <vector>
// #include <string>
// #include <chrono>

// // RKNN API
// #include "rknn_api.h"

// // 检测和分割结果结构体
// struct DetectionResult {
//     float x_center, y_center, width, height;
//     int class_id;
//     float confidence;
//     cv::Mat mask;  // 分割掩码
// };

// class ImageInferenceNode : public rclcpp::Node
// {
// public:
//     ImageInferenceNode() : Node("image_inference_node"), frame_count_(0)
//     {
//         // 初始化类别名称
//         initializeClassNames();
        
//         // 声明参数
//         this->declare_parameter("model_path", "segmentation_model.rknn");
//         this->declare_parameter("confidence_threshold", 0.5);
//         this->declare_parameter("input_topic", "/hikrobot_camera/rgb");
//         this->declare_parameter("output_topic", "/inference/results");
//         this->declare_parameter("input_size", 480);
//         this->declare_parameter("num_classes", 3);
        
//         // 获取参数
//         std::string model_path = this->get_parameter("model_path").as_string();
//         confidence_threshold_ = this->get_parameter("confidence_threshold").as_double();
//         std::string input_topic = this->get_parameter("input_topic").as_string();
//         std::string output_topic = this->get_parameter("output_topic").as_string();
//         input_size_ = this->get_parameter("input_size").as_int();
//         num_classes_ = this->get_parameter("num_classes").as_int();
        
//         RCLCPP_INFO(this->get_logger(), "启动RKNN分割推理节点");
//         RCLCPP_INFO(this->get_logger(), "模型路径: %s", model_path.c_str());
//         RCLCPP_INFO(this->get_logger(), "输入尺寸: %dx%d", input_size_, input_size_);
//         RCLCPP_INFO(this->get_logger(), "分割类别数: %d", num_classes_);
//         RCLCPP_INFO(this->get_logger(), "订阅话题: %s", input_topic.c_str());
//         RCLCPP_INFO(this->get_logger(), "发布话题: %s", output_topic.c_str());
        
//         // 初始化RKNN模型
//         if (initRKNNModel(model_path)) {
//             RCLCPP_INFO(this->get_logger(), "RKNN模型初始化成功");
//         } else {
//             RCLCPP_ERROR(this->get_logger(), "RKNN模型初始化失败");
//         }
        
//         // 创建图像订阅器
//         image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
//             input_topic, 10,
//             std::bind(&ImageInferenceNode::imageCallback, this, std::placeholders::_1));
        
//         // 创建推理结果发布器
//         result_pub_ = this->create_publisher<std_msgs::msg::String>(output_topic, 10);
        
//         RCLCPP_INFO(this->get_logger(), "RKNN分割推理节点初始化完成，等待图像数据...");
//     }
    
//     ~ImageInferenceNode() {
//         // 清理RKNN资源
//         if (input_attrs_) {
//             delete[] input_attrs_;
//         }
//         if (output_attrs_) {
//             delete[] output_attrs_;
//         }
//         if (rknn_ctx_) {
//             rknn_destroy(rknn_ctx_);
//         }
//     }
    
//     void initializeClassNames()
//     {
//         // UNet分割类别名称 (根据您的具体模型调整)
//         class_names_ = {
//             "background",    // 类别0: 背景
//             "class_1",       // 类别1: 第一类目标
//             "class_2"        // 类别2: 第二类目标
//         };
//     }
    
//     std::string getClassName(int class_id)
//     {
//         if (class_id >= 0 && class_id < static_cast<int>(class_names_.size())) {
//             return class_names_[class_id];
//         }
//         return "unknown";
//     }
    
//     void initializeImageTransport()
//     {
//         // 在对象完全构造后初始化image_transport
//         if (!vis_pub_initialized_) {
//             image_transport::ImageTransport it(shared_from_this());
//             vis_pub_ = it.advertise("/inference/visualization", 1);
//             vis_pub_initialized_ = true;
//         }
//     }

// private:
//     void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
//     {
//         try {
//             // 确保image_transport已初始化
//             initializeImageTransport();
            
//             // 转换ROS图像消息为OpenCV格式
//             cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//             cv::Mat image = cv_ptr->image;
            
//             RCLCPP_INFO(this->get_logger(), "收到图像: %dx%d, 时间戳: %f", 
//                        image.cols, image.rows, rclcpp::Time(msg->header.stamp).seconds());
            
//             // 执行神经网络推理
//             std::string inference_result = performInference(image);
            
//             // 发布推理结果
//             auto result_msg = std_msgs::msg::String();
//             result_msg.data = inference_result;
//             result_pub_->publish(result_msg);
            
//             // 创建可视化图像（可选）
//             cv::Mat vis_image = createVisualization(image, inference_result);
            
//             // 发布可视化图像
//             if (vis_pub_.getNumSubscribers() > 0) {
//                 sensor_msgs::msg::Image::SharedPtr vis_msg = cv_bridge::CvImage(
//                     msg->header, "bgr8", vis_image).toImageMsg();
//                 vis_pub_.publish(vis_msg);
//             }
            
//         } catch (cv_bridge::Exception& e) {
//             RCLCPP_ERROR(this->get_logger(), "cv_bridge异常: %s", e.what());
//         } catch (const std::exception& e) {
//             RCLCPP_ERROR(this->get_logger(), "推理过程中发生错误: %s", e.what());
//         }
//     }
    
//     // RKNN相关方法
//     bool initRKNNModel(const std::string& model_path) {
//         // 读取RKNN模型文件
//         FILE* fp = fopen(model_path.c_str(), "rb");
//         if (!fp) {
//             RCLCPP_ERROR(this->get_logger(), "无法打开RKNN模型文件: %s", model_path.c_str());
//             return false;
//         }
        
//         fseek(fp, 0, SEEK_END);
//         size_t model_size = ftell(fp);
//         fseek(fp, 0, SEEK_SET);
        
//         void* model_data = malloc(model_size);
//         if (!model_data) {
//             fclose(fp);
//             RCLCPP_ERROR(this->get_logger(), "内存分配失败");
//             return false;
//         }
        
//         fread(model_data, 1, model_size, fp);
//         fclose(fp);
        
//         // 初始化RKNN context
//         int ret = rknn_init(&rknn_ctx_, model_data, model_size, 0, nullptr);
//         free(model_data);
        
//         if (ret < 0) {
//             RCLCPP_ERROR(this->get_logger(), "RKNN初始化失败: %d", ret);
//             return false;
//         }
        
//         // 获取模型信息
//         ret = rknn_query(rknn_ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num_, sizeof(io_num_));
//         if (ret < 0) {
//             RCLCPP_ERROR(this->get_logger(), "获取输入输出数量失败: %d", ret);
//             return false;
//         }
        
//         RCLCPP_INFO(this->get_logger(), "模型输入数量: %d, 输出数量: %d", io_num_.n_input, io_num_.n_output);
        
//         // 获取输入属性
//         input_attrs_ = new rknn_tensor_attr[io_num_.n_input];
//         memset(input_attrs_, 0, sizeof(rknn_tensor_attr) * io_num_.n_input);
//         for (uint32_t i = 0; i < io_num_.n_input; i++) {
//             input_attrs_[i].index = i;
//             ret = rknn_query(rknn_ctx_, RKNN_QUERY_INPUT_ATTR, &(input_attrs_[i]), sizeof(rknn_tensor_attr));
//             if (ret < 0) {
//                 RCLCPP_ERROR(this->get_logger(), "获取输入属性失败: %d", ret);
//                 return false;
//             }
//         }
        
//         // 获取输出属性
//         output_attrs_ = new rknn_tensor_attr[io_num_.n_output];
//         memset(output_attrs_, 0, sizeof(rknn_tensor_attr) * io_num_.n_output);
//         for (uint32_t i = 0; i < io_num_.n_output; i++) {
//             output_attrs_[i].index = i;
//             ret = rknn_query(rknn_ctx_, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs_[i]), sizeof(rknn_tensor_attr));
//             if (ret < 0) {
//                 RCLCPP_ERROR(this->get_logger(), "获取输出属性失败: %d", ret);
//                 return false;
//             }
//         }
        
//         return true;
//     }
    
//     cv::Mat preprocessImage(const cv::Mat& image) {
//         cv::Mat processed;
        
//         // 调整图像大小到模型输入尺寸
//         cv::resize(image, processed, cv::Size(input_size_, input_size_));
        
//         // 转换为RGB格式 (RKNN通常期望RGB格式)
//         cv::cvtColor(processed, processed, cv::COLOR_BGR2RGB);
        
//         return processed;
//     }
    
//     std::string performInference(const cv::Mat& image)
//     {
//         if (rknn_ctx_ == 0) {
//             return "RKNN模型未初始化";
//         }
        
//         try {
//             // 预处理图像
//             cv::Mat processed_image = preprocessImage(image);
            
//             // 准备输入数据
//             rknn_input inputs[1];
//             memset(inputs, 0, sizeof(inputs));
//             inputs[0].index = 0;
//             inputs[0].type = RKNN_TENSOR_UINT8;
//             inputs[0].size = processed_image.total() * processed_image.elemSize();
//             inputs[0].fmt = RKNN_TENSOR_NHWC;
//             inputs[0].buf = processed_image.data;
            
//             // 设置输入
//             int ret = rknn_inputs_set(rknn_ctx_, io_num_.n_input, inputs);
//             if (ret < 0) {
//                 return "设置RKNN输入失败";
//             }
            
//             // 执行推理
//             std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
//             ret = rknn_run(rknn_ctx_, nullptr);
//             if (ret < 0) {
//                 return "RKNN推理执行失败";
//             }
            
//             // 获取输出
//             rknn_output outputs[1];
//             memset(outputs, 0, sizeof(outputs));
//             outputs[0].want_float = 1;
            
//             ret = rknn_outputs_get(rknn_ctx_, io_num_.n_output, outputs, nullptr);
//             if (ret < 0) {
//                 return "获取RKNN输出失败";
//             }
            
//             auto end = std::chrono::steady_clock::now();
//             std::chrono::duration<double> duration = end - start;
//             RCLCPP_INFO(this->get_logger(), "RKNN推理耗时: %.2f ms", duration.count() * 1000.0);
            
//             // 处理输出结果
//             std::string result = processRKNNOutput(outputs, image.size());
            
//             // 释放输出
//             rknn_outputs_release(rknn_ctx_, io_num_.n_output, outputs);
            
//             return result;
            
//         } catch (const std::exception& e) {
//             RCLCPP_ERROR(this->get_logger(), "RKNN推理执行失败: %s", e.what());
//             return "推理失败";
//         }
//     }
    
//     std::string processRKNNOutput(rknn_output* outputs, const cv::Size& original_size)
//     {
//         detection_results_.clear();
//         std::stringstream result;
//         result << "RKNN分割结果: ";

//         if (!outputs || !outputs[0].buf) {
//             return result.str() + " - 无输出";
//         }

//         // 获取输出数据
//         float* output_data = (float*)outputs[0].buf;
        
//         // 输出应该是 [1, 3, 480, 480] 格式
//         // 基于RKNN输出属性计算维度
//         int channels = num_classes_;  // 3
//         int height = input_size_;     // 480
//         int width = input_size_;      // 480
        
//         result << ", 输出形状=[1," << channels << "," << height << "," << width << "]";
        
//         // 处理分割结果
//         cv::Mat final_mask = processRKNNSegmentationMask(output_data, channels, height, width, original_size);
        
//         // 分析分割结果
//         std::map<int, int> class_pixel_counts;
//         for (int y = 0; y < final_mask.rows; ++y) {
//             for (int x = 0; x < final_mask.cols; ++x) {
//                 int class_id = static_cast<int>(final_mask.at<uchar>(y, x));
//                 class_pixel_counts[class_id]++;
//             }
//         }
        
//         // 创建伪检测结果用于可视化
//         for (const auto& pair : class_pixel_counts) {
//             int class_id = pair.first;
//             int pixel_count = pair.second;
            
//             if (class_id > 0 && pixel_count > 100) { // 忽略背景类和太小的区域
//                 DetectionResult det;
//                 det.class_id = class_id;
//                 det.confidence = static_cast<float>(pixel_count) / (original_size.width * original_size.height);
                
//                 // 创建该类别的二值掩码
//                 cv::Mat class_mask = cv::Mat::zeros(original_size, CV_32F);
//                 for (int y = 0; y < final_mask.rows; ++y) {
//                     for (int x = 0; x < final_mask.cols; ++x) {
//                         if (static_cast<int>(final_mask.at<uchar>(y, x)) == class_id) {
//                             class_mask.at<float>(y, x) = 1.0f;
//                         }
//                     }
//                 }
//                 det.mask = class_mask;
                
//                 // 计算该类别的边界框
//                 cv::Mat binary_mask;
//                 class_mask.convertTo(binary_mask, CV_8U, 255);
//                 std::vector<std::vector<cv::Point>> contours;
//                 cv::findContours(binary_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                
//                 if (!contours.empty()) {
//                     cv::Rect bbox = cv::boundingRect(contours[0]);
//                     for (size_t i = 1; i < contours.size(); ++i) {
//                         bbox |= cv::boundingRect(contours[i]);
//                     }
                    
//                     det.x_center = bbox.x + bbox.width / 2.0f;
//                     det.y_center = bbox.y + bbox.height / 2.0f;
//                     det.width = bbox.width;
//                     det.height = bbox.height;
//                 } else {
//                     det.x_center = original_size.width / 2.0f;
//                     det.y_center = original_size.height / 2.0f;
//                     det.width = original_size.width / 4.0f;
//                     det.height = original_size.height / 4.0f;
//                 }
                
//                 detection_results_.push_back(det);
//             }
//         }
        
//         result << ", 检测到类别数=" << detection_results_.size();
        
//         // 添加各类别的像素统计
//         for (const auto& pair : class_pixel_counts) {
//             if (pair.first > 0) { // 忽略背景
//                 float percentage = 100.0f * pair.second / (original_size.width * original_size.height);
//                 result << ", " << getClassName(pair.first) << "=" << std::fixed << std::setprecision(1) << percentage << "%";
//             }
//         }
        
//         return result.str();
//     }
    
//     cv::Mat processRKNNSegmentationMask(float* output_data, int channels, int height, int width, const cv::Size& original_size)
//     {
//         // 创建类别掩码
//         cv::Mat class_mask = cv::Mat::zeros(height, width, CV_8UC1);
        
//         // 对每个像素找到最大概率的类别
//         for (int y = 0; y < height; ++y) {
//             for (int x = 0; x < width; ++x) {
//                 float max_prob = -1.0f;
//                 int best_class = 0;
                
//                 for (int c = 0; c < channels; ++c) {
//                     // 计算索引 [batch, channel, height, width] 或 [channel, height, width]
//                     int idx = c * height * width + y * width + x;
//                     float prob = output_data[idx];
                    
//                     // 可选：应用softmax或sigmoid激活
//                     // prob = 1.0f / (1.0f + std::exp(-prob)); // sigmoid
                    
//                     if (prob > max_prob) {
//                         max_prob = prob;
//                         best_class = c;
//                     }
//                 }
                
//                 class_mask.at<uchar>(y, x) = static_cast<uchar>(best_class);
//             }
//         }
        
//         // 将掩码从480x480缩放到原始图像尺寸
//         cv::Mat resized_mask;
//         cv::resize(class_mask, resized_mask, original_size, 0, 0, cv::INTER_NEAREST);
        
//         return resized_mask;
//     }
    
//     cv::Mat createVisualization(const cv::Mat& image, const std::string& inference_result)
//     {
//         cv::Mat vis_image = image.clone();
        
//         // 创建分割可视化图像
//         cv::Mat segmentation_overlay = cv::Mat::zeros(image.size(), CV_8UC3);
        
//         // 绘制每个检测到的分割区域
//         for (size_t i = 0; i < detection_results_.size(); ++i) {
//             const auto& det = detection_results_[i];
            
//             // 获取类别颜色
//             cv::Scalar color = getClassColor(det.class_id);
            
//             // 处理分割掩码
//             if (!det.mask.empty() && det.mask.type() == CV_32F) {
//                 // 将浮点掩码转换为二值图像
//                 cv::Mat binary_mask;
//                 cv::threshold(det.mask, binary_mask, 0.5, 255, cv::THRESH_BINARY);
//                 binary_mask.convertTo(binary_mask, CV_8U);
                
//                 // 创建彩色掩码
//                 cv::Mat colored_mask = cv::Mat::zeros(image.size(), CV_8UC3);
//                 colored_mask.setTo(color, binary_mask);
                
//                 // 将彩色掩码叠加到分割图像上
//                 cv::addWeighted(segmentation_overlay, 1.0, colored_mask, 0.8, 0, segmentation_overlay);
//             }
//         }
        
//         // 将分割掩码叠加到原图上
//         cv::addWeighted(vis_image, 0.6, segmentation_overlay, 0.4, 0, vis_image);
        
//         // // 在图像顶部添加总体信息
//         // std::string info = "分割数量: " + std::to_string(detection_results_.size());
//         // cv::putText(vis_image, info, cv::Point(10, 30), 
//         //            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
        
//         // // 添加推理结果信息的前80个字符
//         // std::string short_result = inference_result.length() > 80 ? 
//         //                           inference_result.substr(0, 80) + "..." : inference_result;
//         // cv::putText(vis_image, short_result, cv::Point(10, 60), 
//         //            cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 0), 1);
        
//         // 添加时间戳
//         // auto now = this->get_clock()->now();
//         // std::string timestamp = "帧: " + std::to_string(frame_count_);
//         // cv::putText(vis_image, timestamp, cv::Point(10, 90), 
//         //            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
        
//         // // 保存图像（每10帧保存一次）
//         // if (frame_count_ % 10 == 0) {
//         //     // 保存混合图像
//         //     std::string filename = "/home/elf/slam/MVS_ROS2/segmentation_result_" + 
//         //                          std::to_string(frame_count_) + ".jpg";
//         //     cv::imwrite(filename, vis_image);
            
//         //     // 单独保存纯分割图像
//         //     std::string seg_filename = "/home/elf/slam/MVS_ROS2/segmentation_mask_" + 
//         //                              std::to_string(frame_count_) + ".jpg";
//         //     cv::imwrite(seg_filename, segmentation_overlay);
            
//         //     RCLCPP_INFO(this->get_logger(), "保存分割结果图像: %s", filename.c_str());
//         //     RCLCPP_INFO(this->get_logger(), "保存分割掩码图像: %s", seg_filename.c_str());
//         // }
//         frame_count_++;
        
//         return vis_image;
//     }
    
//     cv::Scalar getClassColor(int class_id)
//     {
//         // 为不同类别生成不同颜色
//         std::vector<cv::Scalar> colors = {
//             cv::Scalar(255, 0, 0),    // 红色
//             cv::Scalar(0, 255, 0),    // 绿色
//             cv::Scalar(0, 0, 255),    // 蓝色
//             cv::Scalar(255, 255, 0),  // 青色
//             cv::Scalar(255, 0, 255),  // 洋红色
//             cv::Scalar(0, 255, 255),  // 黄色
//             cv::Scalar(128, 0, 128),  // 紫色
//             cv::Scalar(255, 165, 0),  // 橙色
//             cv::Scalar(255, 192, 203), // 粉色
//             cv::Scalar(0, 128, 128)   // 青绿色
//         };
        
//         return colors[class_id % colors.size()];
//     }

//     // 成员变量
//     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
//     image_transport::Publisher vis_pub_;
//     bool vis_pub_initialized_ = false;
    
//     // RKNN相关成员变量
//     rknn_context rknn_ctx_ = 0;
//     rknn_input_output_num io_num_;
//     rknn_tensor_attr* input_attrs_ = nullptr;
//     rknn_tensor_attr* output_attrs_ = nullptr;
    
//     double confidence_threshold_;
//     int input_size_;       // RKNN输入尺寸 (480)
//     int num_classes_;      // 分割类别数 (3)
    
//     // 检测结果和可视化相关
//     std::vector<DetectionResult> detection_results_;
//     std::vector<std::string> class_names_;
//     int frame_count_;
// };

// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<ImageInferenceNode>();
    
//     RCLCPP_INFO(node->get_logger(), "开始运行RKNN图像推理节点...");
//     rclcpp::spin(node);
    
//     rclcpp::shutdown();
//     return 0;
// }
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <iomanip>
#include <sstream>
#include <vector>
#include <string>
#include <chrono>
#include <memory>

// RKNN API
#include "rknn_api.h"

// 检测和分割结果结构体
struct DetectionResult {
    float x_center, y_center, width, height;
    int class_id;
    float confidence;
    cv::Mat mask;  // 分割掩码
};

class ImageInferenceNode : public rclcpp::Node
{
public:
    ImageInferenceNode() : Node("image_inference_node"), frame_count_(0), rknn_ctx_(0), 
                          input_attrs_(nullptr), output_attrs_(nullptr), vis_pub_initialized_(false)
    {
        // 初始化类别名称
        initializeClassNames();
        
        // 声明参数
        this->declare_parameter("model_path", "segmentation_model.rknn");
        this->declare_parameter("confidence_threshold", 0.5);
        this->declare_parameter("input_topic", "/hikrobot_camera/rgb");
        this->declare_parameter("output_topic", "/inference/results");
        this->declare_parameter("input_size", 480);
        this->declare_parameter("num_classes", 3);
        
        // 获取参数
        std::string model_path = this->get_parameter("model_path").as_string();
        confidence_threshold_ = this->get_parameter("confidence_threshold").as_double();
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        input_size_ = this->get_parameter("input_size").as_int();
        num_classes_ = this->get_parameter("num_classes").as_int();
        
        RCLCPP_INFO(this->get_logger(), "启动RKNN分割推理节点");
        RCLCPP_INFO(this->get_logger(), "模型路径: %s", model_path.c_str());
        RCLCPP_INFO(this->get_logger(), "输入尺寸: %dx%d", input_size_, input_size_);
        RCLCPP_INFO(this->get_logger(), "分割类别数: %d", num_classes_);
        RCLCPP_INFO(this->get_logger(), "订阅话题: %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "发布话题: %s", output_topic.c_str());
        
        // 初始化RKNN模型
        if (initRKNNModel(model_path)) {
            RCLCPP_INFO(this->get_logger(), "RKNN模型初始化成功");
        } else {
            RCLCPP_ERROR(this->get_logger(), "RKNN模型初始化失败");
        }
        
        // 创建图像订阅器
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic, 10,
            std::bind(&ImageInferenceNode::imageCallback, this, std::placeholders::_1));
        
        // 创建推理结果发布器
        result_pub_ = this->create_publisher<std_msgs::msg::String>(output_topic, 10);
        
        RCLCPP_INFO(this->get_logger(), "RKNN分割推理节点初始化完成，等待图像数据...");
    }
    
    ~ImageInferenceNode() {
        // 清理RKNN资源
        cleanupRKNNResources();
    }
    
    void initializeClassNames()
    {
        // UNet分割类别名称 (根据您的具体模型调整)
        class_names_ = {
            "background",    // 类别0: 背景
            "class_1",       // 类别1: 第一类目标
            "class_2"        // 类别2: 第二类目标
        };
    }
    
    std::string getClassName(int class_id)
    {
        if (class_id >= 0 && class_id < static_cast<int>(class_names_.size())) {
            return class_names_[class_id];
        }
        return "unknown";
    }
    
    void initializeImageTransport()
    {
        // 在对象完全构造后初始化image_transport
        if (!vis_pub_initialized_) {
            image_transport::ImageTransport it(shared_from_this());
            vis_pub_ = it.advertise("/inference/visualization", 1);
            vis_pub_initialized_ = true;
        }
    }

private:
    void cleanupRKNNResources() {
        // 清理输入属性数组
        if (input_attrs_) {
            delete[] input_attrs_;
            input_attrs_ = nullptr;
        }
        
        // 清理输出属性数组
        if (output_attrs_) {
            delete[] output_attrs_;
            output_attrs_ = nullptr;
        }
        
        // 销毁RKNN上下文
        if (rknn_ctx_) {
            int ret = rknn_destroy(rknn_ctx_);
            if (ret < 0) {
                RCLCPP_WARN(this->get_logger(), "RKNN上下文销毁失败: %d", ret);
            }
            rknn_ctx_ = 0;
        }
        
        RCLCPP_INFO(this->get_logger(), "RKNN资源清理完成");
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // 确保image_transport已初始化
            initializeImageTransport();
            
            // 转换ROS图像消息为OpenCV格式
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat image = cv_ptr->image;
            
            RCLCPP_INFO(this->get_logger(), "收到图像: %dx%d, 时间戳: %f", 
                       image.cols, image.rows, rclcpp::Time(msg->header.stamp).seconds());
            
            // 执行神经网络推理
            std::string inference_result = performInference(image);
            
            // 发布推理结果
            auto result_msg = std_msgs::msg::String();
            result_msg.data = inference_result;
            result_pub_->publish(result_msg);
            
            // 创建可视化图像（可选）
            cv::Mat vis_image = createVisualization(image, inference_result);
            
            // 发布可视化图像，使用与输入图像相同的时间戳
            if (vis_pub_.getNumSubscribers() > 0) {
                // 创建可视化图像消息，确保时间戳一致
                cv_bridge::CvImage cv_vis_image;
                cv_vis_image.header = msg->header;  // 直接复制header确保时间戳一致
                cv_vis_image.encoding = "bgr8";
                cv_vis_image.image = vis_image;
                
                sensor_msgs::msg::Image::SharedPtr vis_msg = cv_vis_image.toImageMsg();
                vis_pub_.publish(vis_msg);
                
                // 调试信息：比较时间戳
                RCLCPP_DEBUG(this->get_logger(), "原始图像时间戳: %f, 可视化图像时间戳: %f", 
                            rclcpp::Time(msg->header.stamp).seconds(),
                            rclcpp::Time(vis_msg->header.stamp).seconds());
            }
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge异常: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "推理过程中发生错误: %s", e.what());
        }
    }
    
    // RKNN相关方法
    bool initRKNNModel(const std::string& model_path) {
        // 读取RKNN模型文件
        FILE* fp = fopen(model_path.c_str(), "rb");
        if (!fp) {
            RCLCPP_ERROR(this->get_logger(), "无法打开RKNN模型文件: %s", model_path.c_str());
            return false;
        }
        
        fseek(fp, 0, SEEK_END);
        size_t model_size = ftell(fp);
        fseek(fp, 0, SEEK_SET);
        
        // 使用智能指针管理模型数据内存
        std::unique_ptr<uint8_t[]> model_data(new uint8_t[model_size]);
        if (!model_data) {
            fclose(fp);
            RCLCPP_ERROR(this->get_logger(), "内存分配失败");
            return false;
        }
        
        size_t read_size = fread(model_data.get(), 1, model_size, fp);
        fclose(fp);
        
        if (read_size != model_size) {
            RCLCPP_ERROR(this->get_logger(), "模型文件读取不完整");
            return false;
        }
        
        // 初始化RKNN context
        int ret = rknn_init(&rknn_ctx_, model_data.get(), model_size, 0, nullptr);
        // model_data会在函数结束时自动释放
        
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "RKNN初始化失败: %d", ret);
            return false;
        }
        
        // 获取模型信息
        ret = rknn_query(rknn_ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num_, sizeof(io_num_));
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "获取输入输出数量失败: %d", ret);
            cleanupRKNNResources();
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "模型输入数量: %d, 输出数量: %d", io_num_.n_input, io_num_.n_output);
        
        // 获取输入属性
        input_attrs_ = new rknn_tensor_attr[io_num_.n_input];
        memset(input_attrs_, 0, sizeof(rknn_tensor_attr) * io_num_.n_input);
        for (uint32_t i = 0; i < io_num_.n_input; i++) {
            input_attrs_[i].index = i;
            ret = rknn_query(rknn_ctx_, RKNN_QUERY_INPUT_ATTR, &(input_attrs_[i]), sizeof(rknn_tensor_attr));
            if (ret < 0) {
                RCLCPP_ERROR(this->get_logger(), "获取输入属性失败: %d", ret);
                cleanupRKNNResources();
                return false;
            }
        }
        
        // 获取输出属性
        output_attrs_ = new rknn_tensor_attr[io_num_.n_output];
        memset(output_attrs_, 0, sizeof(rknn_tensor_attr) * io_num_.n_output);
        for (uint32_t i = 0; i < io_num_.n_output; i++) {
            output_attrs_[i].index = i;
            ret = rknn_query(rknn_ctx_, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs_[i]), sizeof(rknn_tensor_attr));
            if (ret < 0) {
                RCLCPP_ERROR(this->get_logger(), "获取输出属性失败: %d", ret);
                cleanupRKNNResources();
                return false;
            }
        }
        
        return true;
    }
    
    cv::Mat preprocessImage(const cv::Mat& image) {
        cv::Mat processed;
        
        // 调整图像大小到模型输入尺寸
        cv::resize(image, processed, cv::Size(input_size_, input_size_));
        
        // 转换为RGB格式 (RKNN通常期望RGB格式)
        cv::cvtColor(processed, processed, cv::COLOR_BGR2RGB);
        
        return processed;
    }
    
    std::string performInference(const cv::Mat& image)
    {
        if (rknn_ctx_ == 0) {
            return "RKNN模型未初始化";
        }
        
        try {
            // 预处理图像
            cv::Mat processed_image = preprocessImage(image);
            
            // 准备输入数据
            rknn_input inputs[1];
            memset(inputs, 0, sizeof(inputs));
            inputs[0].index = 0;
            inputs[0].type = RKNN_TENSOR_UINT8;
            inputs[0].size = processed_image.total() * processed_image.elemSize();
            inputs[0].fmt = RKNN_TENSOR_NHWC;
            inputs[0].buf = processed_image.data;
            
            // 设置输入
            int ret = rknn_inputs_set(rknn_ctx_, io_num_.n_input, inputs);
            if (ret < 0) {
                return "设置RKNN输入失败: " + std::to_string(ret);
            }
            
            // 执行推理
            std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
            ret = rknn_run(rknn_ctx_, nullptr);
            if (ret < 0) {
                return "RKNN推理执行失败: " + std::to_string(ret);
            }
            
            // 获取输出
            rknn_output outputs[1];
            memset(outputs, 0, sizeof(outputs));
            outputs[0].want_float = 1;
            
            ret = rknn_outputs_get(rknn_ctx_, io_num_.n_output, outputs, nullptr);
            if (ret < 0) {
                return "获取RKNN输出失败: " + std::to_string(ret);
            }
            
            auto end = std::chrono::steady_clock::now();
            std::chrono::duration<double> duration = end - start;
            RCLCPP_INFO(this->get_logger(), "RKNN推理耗时: %.2f ms", duration.count() * 1000.0);
            
            // 处理输出结果
            std::string result = processRKNNOutput(outputs, image.size());
            
            // 释放输出 - 重要！防止内存泄漏
            ret = rknn_outputs_release(rknn_ctx_, io_num_.n_output, outputs);
            if (ret < 0) {
                RCLCPP_WARN(this->get_logger(), "释放RKNN输出失败: %d", ret);
            }
            
            return result;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "RKNN推理执行失败: %s", e.what());
            return "推理失败: " + std::string(e.what());
        }
    }
    
    std::string processRKNNOutput(rknn_output* outputs, const cv::Size& original_size)
    {
        detection_results_.clear();
        std::stringstream result;
        result << "RKNN分割结果: ";

        if (!outputs || !outputs[0].buf) {
            return result.str() + " - 无输出";
        }

        // 获取输出数据
        float* output_data = (float*)outputs[0].buf;
        
        // 输出应该是 [1, 3, 480, 480] 格式
        // 基于RKNN输出属性计算维度
        int channels = num_classes_;  // 3
        int height = input_size_;     // 480
        int width = input_size_;      // 480
        
        result << ", 输出形状=[1," << channels << "," << height << "," << width << "]";
        
        // 处理分割结果
        cv::Mat final_mask = processRKNNSegmentationMask(output_data, channels, height, width, original_size);
        
        // 分析分割结果
        std::map<int, int> class_pixel_counts;
        for (int y = 0; y < final_mask.rows; ++y) {
            for (int x = 0; x < final_mask.cols; ++x) {
                int class_id = static_cast<int>(final_mask.at<uchar>(y, x));
                class_pixel_counts[class_id]++;
            }
        }
        
        // 创建伪检测结果用于可视化
        for (const auto& pair : class_pixel_counts) {
            int class_id = pair.first;
            int pixel_count = pair.second;
            
            if (class_id > 0 && pixel_count > 100) { // 忽略背景类和太小的区域
                DetectionResult det;
                det.class_id = class_id;
                det.confidence = static_cast<float>(pixel_count) / (original_size.width * original_size.height);
                
                // 创建该类别的二值掩码
                cv::Mat class_mask = cv::Mat::zeros(original_size, CV_32F);
                for (int y = 0; y < final_mask.rows; ++y) {
                    for (int x = 0; x < final_mask.cols; ++x) {
                        if (static_cast<int>(final_mask.at<uchar>(y, x)) == class_id) {
                            class_mask.at<float>(y, x) = 1.0f;
                        }
                    }
                }
                det.mask = class_mask;
                
                // 计算该类别的边界框
                cv::Mat binary_mask;
                class_mask.convertTo(binary_mask, CV_8U, 255);
                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(binary_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                
                if (!contours.empty()) {
                    cv::Rect bbox = cv::boundingRect(contours[0]);
                    for (size_t i = 1; i < contours.size(); ++i) {
                        bbox |= cv::boundingRect(contours[i]);
                    }
                    
                    det.x_center = bbox.x + bbox.width / 2.0f;
                    det.y_center = bbox.y + bbox.height / 2.0f;
                    det.width = bbox.width;
                    det.height = bbox.height;
                } else {
                    det.x_center = original_size.width / 2.0f;
                    det.y_center = original_size.height / 2.0f;
                    det.width = original_size.width / 4.0f;
                    det.height = original_size.height / 4.0f;
                }
                
                detection_results_.push_back(det);
            }
        }
        
        result << ", 检测到类别数=" << detection_results_.size();
        
        // 添加各类别的像素统计
        for (const auto& pair : class_pixel_counts) {
            if (pair.first > 0) { // 忽略背景
                float percentage = 100.0f * pair.second / (original_size.width * original_size.height);
                result << ", " << getClassName(pair.first) << "=" << std::fixed << std::setprecision(1) << percentage << "%";
            }
        }
        
        return result.str();
    }
    
    cv::Mat processRKNNSegmentationMask(float* output_data, int channels, int height, int width, const cv::Size& original_size)
    {
        // 创建类别掩码
        cv::Mat class_mask = cv::Mat::zeros(height, width, CV_8UC1);
        
        // 对每个像素找到最大概率的类别
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                float max_prob = -1.0f;
                int best_class = 0;
                
                for (int c = 0; c < channels; ++c) {
                    // 计算索引 [batch, channel, height, width] 或 [channel, height, width]
                    int idx = c * height * width + y * width + x;
                    float prob = output_data[idx];
                    
                    // 可选：应用softmax或sigmoid激活
                    // prob = 1.0f / (1.0f + std::exp(-prob)); // sigmoid
                    
                    if (prob > max_prob) {
                        max_prob = prob;
                        best_class = c;
                    }
                }
                
                class_mask.at<uchar>(y, x) = static_cast<uchar>(best_class);
            }
        }
        
        // 将掩码从480x480缩放到原始图像尺寸
        cv::Mat resized_mask;
        cv::resize(class_mask, resized_mask, original_size, 0, 0, cv::INTER_NEAREST);
        
        return resized_mask;
    }
    
    cv::Mat createVisualization(const cv::Mat& image, const std::string& inference_result)
    {
        cv::Mat vis_image = image.clone();
        
        // 创建分割可视化图像
        cv::Mat segmentation_overlay = cv::Mat::zeros(image.size(), CV_8UC3);
        
        // 绘制每个检测到的分割区域
        for (size_t i = 0; i < detection_results_.size(); ++i) {
            const auto& det = detection_results_[i];
            
            // 获取类别颜色
            cv::Scalar color = getClassColor(det.class_id);
            
            // 处理分割掩码
            if (!det.mask.empty() && det.mask.type() == CV_32F) {
                // 将浮点掩码转换为二值图像
                cv::Mat binary_mask;
                cv::threshold(det.mask, binary_mask, 0.5, 255, cv::THRESH_BINARY);
                binary_mask.convertTo(binary_mask, CV_8U);
                
                // 创建彩色掩码
                cv::Mat colored_mask = cv::Mat::zeros(image.size(), CV_8UC3);
                colored_mask.setTo(color, binary_mask);
                
                // 将彩色掩码叠加到分割图像上
                cv::addWeighted(segmentation_overlay, 1.0, colored_mask, 0.8, 0, segmentation_overlay);
            }
        }
        
        // 将分割掩码叠加到原图上
        cv::addWeighted(vis_image, 0.6, segmentation_overlay, 0.4, 0, vis_image);
        
        frame_count_++;
        
        return vis_image;
    }
    
    cv::Scalar getClassColor(int class_id)
    {
        // 为不同类别生成不同颜色
        std::vector<cv::Scalar> colors = {
            cv::Scalar(255, 0, 0),    // 红色
            cv::Scalar(0, 255, 0),    // 绿色
            cv::Scalar(0, 0, 255),    // 蓝色
            cv::Scalar(255, 255, 0),  // 青色
            cv::Scalar(255, 0, 255),  // 洋红色
            cv::Scalar(0, 255, 255),  // 黄色
            cv::Scalar(128, 0, 128),  // 紫色
            cv::Scalar(255, 165, 0),  // 橙色
            cv::Scalar(255, 192, 203), // 粉色
            cv::Scalar(0, 128, 128)   // 青绿色
        };
        
        return colors[class_id % colors.size()];
    }

    // 成员变量
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
    image_transport::Publisher vis_pub_;
    bool vis_pub_initialized_;
    
    // RKNN相关成员变量
    rknn_context rknn_ctx_;
    rknn_input_output_num io_num_;
    rknn_tensor_attr* input_attrs_;
    rknn_tensor_attr* output_attrs_;
    
    double confidence_threshold_;
    int input_size_;       // RKNN输入尺寸 (480)
    int num_classes_;      // 分割类别数 (3)
    
    // 检测结果和可视化相关
    std::vector<DetectionResult> detection_results_;
    std::vector<std::string> class_names_;
    int frame_count_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageInferenceNode>();
    
    RCLCPP_INFO(node->get_logger(), "开始运行RKNN图像推理节点...");
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}