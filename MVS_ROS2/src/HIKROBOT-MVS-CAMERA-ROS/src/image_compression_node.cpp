#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>

class ImageCompressionNode : public rclcpp::Node
{
public:
    ImageCompressionNode() : Node("image_compression_node")
    {
        // 声明参数
        this->declare_parameter("input_topic", "/hikrobot_camera/rgb");
        this->declare_parameter("output_topic", "/compressed_image");
        this->declare_parameter("compression_quality", 80);
        this->declare_parameter("compression_format", "jpg");
        
        // 获取参数
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        compression_quality_ = this->get_parameter("compression_quality").as_int();
        compression_format_ = this->get_parameter("compression_format").as_string();
        
        RCLCPP_INFO(this->get_logger(), "启动图像压缩节点");
        RCLCPP_INFO(this->get_logger(), "输入话题: %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "输出话题: %s", output_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "压缩质量: %d", compression_quality_);
        RCLCPP_INFO(this->get_logger(), "压缩格式: %s", compression_format_.c_str());
        
        // 设置压缩参数
        if (compression_format_ == "jpg" || compression_format_ == "jpeg") {
            compression_params_.push_back(cv::IMWRITE_JPEG_QUALITY);
            compression_params_.push_back(compression_quality_);
            format_extension_ = ".jpg";
        } else if (compression_format_ == "png") {
            compression_params_.push_back(cv::IMWRITE_PNG_COMPRESSION);
            compression_params_.push_back(9 - (compression_quality_ / 10)); // PNG压缩级别是0-9，质量越高压缩级别越低
            format_extension_ = ".png";
        } else {
            RCLCPP_WARN(this->get_logger(), "不支持的压缩格式: %s，使用默认JPEG", compression_format_.c_str());
            compression_params_.push_back(cv::IMWRITE_JPEG_QUALITY);
            compression_params_.push_back(compression_quality_);
            format_extension_ = ".jpg";
            compression_format_ = "jpg";
        }
        
        // 创建订阅器
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic, 10,
            std::bind(&ImageCompressionNode::imageCallback, this, std::placeholders::_1));
        
        // 创建发布器 - 发布压缩图像
        compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            output_topic + "/compressed", 10);
        
        // 创建发布器 - 也发布普通图像话题（可选）
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            output_topic, 10);
        
        frame_count_ = 0;
        total_original_size_ = 0;
        total_compressed_size_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "图像压缩节点初始化完成，等待图像数据...");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // 转换ROS图像消息为OpenCV格式
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat image = cv_ptr->image;
            
            // 计算原始图像大小
            size_t original_size = image.total() * image.elemSize();
            total_original_size_ += original_size;
            
            // 压缩图像
            std::vector<uchar> compressed_data;
            bool success = cv::imencode(format_extension_, image, compressed_data, compression_params_);
            
            if (!success) {
                RCLCPP_ERROR(this->get_logger(), "图像压缩失败");
                return;
            }
            
            // 计算压缩后大小
            size_t compressed_size = compressed_data.size();
            total_compressed_size_ += compressed_size;
            
            // 创建压缩图像消息
            auto compressed_msg = sensor_msgs::msg::CompressedImage();
            compressed_msg.header = msg->header;
            compressed_msg.format = compression_format_;
            compressed_msg.data = compressed_data;
            
            // 发布压缩图像
            compressed_pub_->publish(compressed_msg);
            
            // 可选：同时发布解压缩后的图像（用于验证）
            if (image_pub_->get_subscription_count() > 0) {
                // 解压缩验证
                cv::Mat decompressed_image = cv::imdecode(compressed_data, cv::IMREAD_COLOR);
                
                // 转换回ROS消息
                sensor_msgs::msg::Image::SharedPtr decompressed_msg = cv_bridge::CvImage(
                    msg->header, sensor_msgs::image_encodings::BGR8, decompressed_image).toImageMsg();
                
                image_pub_->publish(*decompressed_msg);
            }
            
            frame_count_++;
            
            // 每100帧输出一次统计信息
            if (frame_count_ % 100 == 0) {
                double compression_ratio = 100.0 * (1.0 - static_cast<double>(total_compressed_size_) / static_cast<double>(total_original_size_));
                double avg_original_size = static_cast<double>(total_original_size_) / frame_count_ / 1024.0; // KB
                double avg_compressed_size = static_cast<double>(total_compressed_size_) / frame_count_ / 1024.0; // KB
                
                RCLCPP_INFO(this->get_logger(), 
                    "帧数: %d, 压缩比: %.1f%%, 平均原始大小: %.1f KB, 平均压缩大小: %.1f KB", 
                    frame_count_, compression_ratio, avg_original_size, avg_compressed_size);
            }
            
            // 每10帧输出一次详细信息
            if (frame_count_ % 10 == 0) {
                double frame_compression_ratio = 100.0 * (1.0 - static_cast<double>(compressed_size) / static_cast<double>(original_size));
                RCLCPP_DEBUG(this->get_logger(), 
                    "帧 %d: %dx%d, 原始: %.1f KB, 压缩: %.1f KB (%.1f%%)", 
                    frame_count_, image.cols, image.rows,
                    original_size / 1024.0, compressed_size / 1024.0, frame_compression_ratio);
            }
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge异常: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "图像压缩过程中发生错误: %s", e.what());
        }
    }
    
    // 成员变量
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    
    int compression_quality_;
    std::string compression_format_;
    std::string format_extension_;
    std::vector<int> compression_params_;
    
    // 统计信息
    int frame_count_;
    size_t total_original_size_;
    size_t total_compressed_size_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageCompressionNode>();
    
    RCLCPP_INFO(node->get_logger(), "开始运行图像压缩节点...");
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
