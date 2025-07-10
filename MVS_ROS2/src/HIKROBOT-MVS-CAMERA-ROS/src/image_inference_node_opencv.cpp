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
    ImageInferenceNode() : Node("image_inference_node"), frame_count_(0)
    {
        // 初始化类别名称
        initializeClassNames();
        
        // 声明参数
        this->declare_parameter("model_path", "best.onnx");
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
        
        RCLCPP_INFO(this->get_logger(), "启动UNet分割推理节点");
        RCLCPP_INFO(this->get_logger(), "模型路径: %s", model_path.c_str());
        RCLCPP_INFO(this->get_logger(), "输入尺寸: %dx%d", input_size_, input_size_);
        RCLCPP_INFO(this->get_logger(), "分割类别数: %d", num_classes_);
        RCLCPP_INFO(this->get_logger(), "订阅话题: %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "发布话题: %s", output_topic.c_str());
        
        // 初始化神经网络模型（这里使用OpenCV DNN作为示例）
        try {
            // 根据模型文件扩展名选择加载方式
            if (model_path.find(".onnx") != std::string::npos) {
                net_ = cv::dnn::readNetFromONNX(model_path);
                RCLCPP_INFO(this->get_logger(), "成功加载ONNX模型");
            } else if (model_path.find(".pb") != std::string::npos) {
                net_ = cv::dnn::readNetFromTensorflow(model_path);
                RCLCPP_INFO(this->get_logger(), "成功加载TensorFlow模型");
            } else {
                RCLCPP_WARN(this->get_logger(), "未知模型格式，请确保模型路径正确");
            }
            
            // 设置推理后端（可选：CPU, CUDA, OpenCL等）
            net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU); // 或 DNN_TARGET_OPENCL
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "模型加载失败: %s", e.what());
        }
        
        // 创建图像订阅器
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic, 10,
            std::bind(&ImageInferenceNode::imageCallback, this, std::placeholders::_1));
        
        // 创建推理结果发布器
        result_pub_ = this->create_publisher<std_msgs::msg::String>(output_topic, 10);
        
        RCLCPP_INFO(this->get_logger(), "UNet分割推理节点初始化完成，等待图像数据...");
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
            
            // 发布可视化图像
            if (vis_pub_.getNumSubscribers() > 0) {
                sensor_msgs::msg::Image::SharedPtr vis_msg = cv_bridge::CvImage(
                    msg->header, "bgr8", vis_image).toImageMsg();
                vis_pub_.publish(vis_msg);
            }
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge异常: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "推理过程中发生错误: %s", e.what());
        }
    }
    
    std::string performInference(const cv::Mat& image)
    {
        if (net_.empty()) {
            return "模型未加载";
        }
        
        try {
            // 预处理图像 - UNet通常需要480x480输入
            cv::Mat blob;
            cv::dnn::blobFromImage(image, blob, 1.0/255.0, cv::Size(input_size_, input_size_), cv::Scalar(0,0,0), true, false);
            
            // 设置网络输入
            net_.setInput(blob);
            
            // 执行前向传播
            std::vector<cv::Mat> outputs;
            //calc time
            std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
            net_.forward(outputs, net_.getUnconnectedOutLayersNames());
            auto end = std::chrono::steady_clock::now();
            std::chrono::duration<double> duration = end - start;
            std::cout << "ms: " << duration.count() * 1000.0 << std::endl;
            
            // 处理UNet输出结果
            std::string result = processUNetOutput(outputs, image.size());
            
            return result;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "UNet推理执行失败: %s", e.what());
            return "推理失败";
        }
    }
    
    std::string processUNetOutput(const std::vector<cv::Mat>& outputs, const cv::Size& original_size)
    {
        detection_results_.clear();
        std::stringstream result;
        result << "UNet分割结果: ";

        if (outputs.empty()) {
            return result.str() + " - 无输出";
        }

        result << "输出层数=" << outputs.size();

        // UNet通常只有一个输出层
        cv::Mat segmentation_output = outputs[0];
        
        // 输出应该是 [1, 3, 480, 480] 格式
        if (segmentation_output.dims == 4 && segmentation_output.size[0] == 1) {
            int channels = segmentation_output.size[1];  // 应该是3
            int height = segmentation_output.size[2];    // 应该是480
            int width = segmentation_output.size[3];     // 应该是480
            
            result << ", 输出形状=[" << segmentation_output.size[0] << "," 
                   << channels << "," << height << "," << width << "]";
            
            if (channels == num_classes_ && height == input_size_ && width == input_size_) {
                // 处理分割结果
                cv::Mat final_mask = processSegmentationMask(segmentation_output, original_size);
                
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
                
            } else {
                result << " - 输出维度不匹配预期";
            }
        } else {
            result << " - 输出格式不正确";
        }
        
        return result.str();
    }
    
    cv::Mat processSegmentationMask(const cv::Mat& output, const cv::Size& original_size)
    {
        // output shape: [1, 3, 480, 480]
        // 需要将输出转换为类别掩码
        
        int channels = output.size[1];  // 3
        int height = output.size[2];    // 480
        int width = output.size[3];     // 480
        
        // 创建类别掩码
        cv::Mat class_mask = cv::Mat::zeros(height, width, CV_8UC1);
        
        // 获取数据指针
        float* data = (float*)output.data;
        
        // 对每个像素找到最大概率的类别
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                float max_prob = -1.0f;
                int best_class = 0;
                
                for (int c = 0; c < channels; ++c) {
                    // 计算4D数据的索引 [batch, channel, height, width]
                    int idx = c * height * width + y * width + x;
                    float prob = data[idx];
                    
                    // 可选：应用softmax或sigmoid激活
                    // prob = 1.0f / (1.0f + std::exp(-prob)); // sigmoid
                    
                    if (prob > max_prob) {
                        max_prob = prob;
                        best_class = c;
                        //std::cout << "Pixel (" << x << ", " << y << ") - Class: " 
                        //          << best_class << ", Probability: " << prob << std::endl; // 调试输出
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
            std::cout << color << det.class_id << std::endl; // 调试输出
            
            // 处理分割掩码
            if (!det.mask.empty() && det.mask.type() == CV_32F) {
                // 将浮点掩码转换为二值图像
                cv::Mat binary_mask;
                std::cout<<det.mask.at<float>(0, 0) << std::endl; // 调试输出
                cv::threshold(det.mask, binary_mask, 0.5, 255, cv::THRESH_BINARY);
                binary_mask.convertTo(binary_mask, CV_8U);
                
                // 创建彩色掩码
                cv::Mat colored_mask = cv::Mat::zeros(image.size(), CV_8UC3);
                colored_mask.setTo(color, binary_mask);
                
                // 将彩色掩码叠加到分割图像上
                cv::addWeighted(segmentation_overlay, 1.0, colored_mask, 0.8, 0, segmentation_overlay);
            }
            
            // 计算边界框坐标
            int x1 = static_cast<int>(det.x_center - det.width / 2);
            int y1 = static_cast<int>(det.y_center - det.height / 2);
            int x2 = static_cast<int>(det.x_center + det.width / 2);
            int y2 = static_cast<int>(det.y_center + det.height / 2);
            
            // 确保坐标在图像范围内
            x1 = std::max(0, std::min(x1, image.cols - 1));
            y1 = std::max(0, std::min(y1, image.rows - 1));
            x2 = std::max(0, std::min(x2, image.cols - 1));
            y2 = std::max(0, std::min(y2, image.rows - 1));
            
            // 绘制边界框
            cv::rectangle(vis_image, cv::Point(x1, y1), cv::Point(x2, y2), color, 2);
            
            // 准备标签文本
            std::string class_name = getClassName(det.class_id);
            std::stringstream label_ss;
            label_ss << class_name << " " << std::fixed << std::setprecision(2) << det.confidence;
            std::string label = label_ss.str();
            
            // 计算文本大小
            int baseline = 0;
            cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
            
            // 绘制文本背景
            if (y1 - text_size.height - 10 > 0) {
                cv::rectangle(vis_image, 
                             cv::Point(x1, y1 - text_size.height - 10),
                             cv::Point(x1 + text_size.width, y1),
                             color, -1);
                
                // 绘制文本
                cv::putText(vis_image, label, cv::Point(x1, y1 - 5), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
            }
        }
        
        // 将分割掩码叠加到原图上
        cv::addWeighted(vis_image, 0.6, segmentation_overlay, 0.4, 0, vis_image);
        
        // 在图像顶部添加总体信息
        std::string info = "分割数量: " + std::to_string(detection_results_.size());
        cv::putText(vis_image, info, cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
        
        // 添加推理结果信息的前80个字符
        std::string short_result = inference_result.length() > 80 ? 
                                  inference_result.substr(0, 80) + "..." : inference_result;
        cv::putText(vis_image, short_result, cv::Point(10, 60), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 0), 1);
        
        // 添加时间戳
        auto now = this->get_clock()->now();
        std::string timestamp = "帧: " + std::to_string(frame_count_);
        cv::putText(vis_image, timestamp, cv::Point(10, 90), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
        
        // 保存图像（每10帧保存一次）
        if (frame_count_ % 10 == 0) {
            // 保存混合图像
            std::string filename = "/home/elf/slam/MVS_ROS2/segmentation_result_" + 
                                 std::to_string(frame_count_) + ".jpg";
            cv::imwrite(filename, vis_image);
            
            // 单独保存纯分割图像
            std::string seg_filename = "/home/elf/slam/MVS_ROS2/segmentation_mask_" + 
                                     std::to_string(frame_count_) + ".jpg";
            cv::imwrite(seg_filename, segmentation_overlay);
            
            RCLCPP_INFO(this->get_logger(), "保存分割结果图像: %s", filename.c_str());
            RCLCPP_INFO(this->get_logger(), "保存分割掩码图像: %s", seg_filename.c_str());
        }
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
    bool vis_pub_initialized_ = false;
    
    cv::dnn::Net net_;
    double confidence_threshold_;
    int input_size_;       // UNet输入尺寸 (480)
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
    
    RCLCPP_INFO(node->get_logger(), "开始运行图像推理节点...");
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
