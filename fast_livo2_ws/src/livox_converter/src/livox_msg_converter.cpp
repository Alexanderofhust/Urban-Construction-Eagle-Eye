#include <rclcpp/rclcpp.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_interfaces/msg/custom_msg.hpp>

class LivoxMsgConverter : public rclcpp::Node
{
public:
  LivoxMsgConverter() : Node("livox_msg_converter")
  {
    // 声明参数
    this->declare_parameter<std::string>("input_topic", "/livox/lidar");
    this->declare_parameter<std::string>("output_topic", "/livox/lidar_converted");
    
    // 获取参数
    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    
    // 创建订阅者和发布者
    subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      input_topic, 10, std::bind(&LivoxMsgConverter::topic_callback, this, std::placeholders::_1));
    
    publisher_ = this->create_publisher<livox_interfaces::msg::CustomMsg>(
      output_topic, 10);
    
    RCLCPP_INFO(this->get_logger(), "启动消息转换器: %s -> %s", 
                input_topic.c_str(), output_topic.c_str());
  }

private:
  void topic_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr src_msg)
  {
    // 创建目标消息
    auto dst_msg = std::make_shared<livox_interfaces::msg::CustomMsg>();
    
    // 复制头部信息
    dst_msg->header = src_msg->header;
    
    // 复制点云元数据
    dst_msg->timebase = src_msg->timebase;
    dst_msg->point_num = src_msg->point_num;
    dst_msg->lidar_id = src_msg->lidar_id;
    
    // 复制保留字段
    for (size_t i = 0; i < 3; ++i) {
      dst_msg->rsvd[i] = src_msg->rsvd[i];
    }
    
    // 复制点云数据
    dst_msg->points.resize(src_msg->points.size());
    
    for (size_t i = 0; i < src_msg->points.size(); ++i) {
      const auto& src_point = src_msg->points[i];
      auto& dst_point = dst_msg->points[i];
      
      // 精确映射所有字段
      dst_point.offset_time = src_point.offset_time;
      dst_point.x = src_point.x;
      dst_point.y = src_point.y;
      dst_point.z = src_point.z;
      dst_point.reflectivity = src_point.reflectivity;
      dst_point.tag = src_point.tag;
      dst_point.line = src_point.line;
    }
    
    // 发布转换后的消息
    publisher_->publish(*dst_msg);
    // 打印时间戳信息
    // double timestamp_seconds = rclcpp::Time(dst_msg->header.stamp).seconds();
    // RCLCPP_INFO(this->get_logger(), "已发布点云消息，时间戳: %.6f秒，点数: %zu", 
    //           timestamp_seconds, dst_msg->points.size());

    //RCLCPP_INFO(this->get_logger(), "已转换并发布 %zu 个点", dst_msg->points.size());
    // auto current_time = this->get_clock()->now();
    // // 打印当前时间
    // RCLCPP_INFO(this->get_logger(), "point cloud transtime: %f秒", current_time.seconds());
  }
  
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
  rclcpp::Publisher<livox_interfaces::msg::CustomMsg>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LivoxMsgConverter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
