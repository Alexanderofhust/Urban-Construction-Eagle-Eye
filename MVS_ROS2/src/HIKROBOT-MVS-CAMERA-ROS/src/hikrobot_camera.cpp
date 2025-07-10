#include <iostream>
#include "opencv2/opencv.hpp"
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <math.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include "hikrobot_camera.hpp"

// 剪裁掉照片和雷达没有重合的视角，去除多余像素可以使rosbag包变小
#define FIT_LIDAR_CUT_IMAGE false
#if FIT_LIDAR_CUT_IMAGE
    #define FIT_min_x 420
    #define FIT_max_x 2450
    #define FIT_min_y 70
    #define FIT_max_y 2000
#endif 

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    //********** variables    **********/
    cv::Mat src;
    //********** rosnode init **********/
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("hikrobot_camera");
    camera::Camera MVS_cap(node);
    
    
    //********** rosnode init **********/
    //auto image_pub = image_transport::create_camera_publisher(node.get(), "/hikrobot_camera/rgb", rmw_qos_profile_sensor_data);
    auto image_pub = image_transport::create_camera_publisher(node.get(), "/hikrobot_camera/rgb");

    sensor_msgs::msg::Image::SharedPtr image_msg(new sensor_msgs::msg::Image);
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg(new sensor_msgs::msg::CameraInfo);
    cv_bridge::CvImage cv_image;
    cv_image.encoding = sensor_msgs::image_encodings::BGR8;  // 就是rgb格式 

    std::string path_for_time_stamp = "/home/elf/timeshare";
    const char *shared_file_name = path_for_time_stamp.c_str();
    int fd = open(shared_file_name, O_RDWR);

    pointt = (time_stamp *)mmap(NULL, sizeof(time_stamp), PROT_READ | PROT_WRITE,
                                MAP_SHARED, fd, 0);
    
    //********** 10 Hz        **********/
    rclcpp::Rate loop_rate(10);

    while (rclcpp::ok())
    {
        loop_rate.sleep();

        MVS_cap.ReadImg(src);
        if (src.empty())
        {
            continue;
        }
        else 
        {
            RCLCPP_INFO(node->get_logger(), "get img success");
            // auto current_time = node->get_clock()->now();
            // // 打印当前时间
            // RCLCPP_INFO(node->get_logger(), "当前系统时间: %f秒", current_time.seconds());
        }
#if FIT_LIDAR_CUT_IMAGE
        cv::Rect area(FIT_min_x,FIT_min_y,FIT_max_x-FIT_min_x,FIT_max_y-FIT_min_y); // cut区域：从左上角像素坐标x，y，宽，高
        cv::Mat src_new = src(area);
        cv_image.image = src_new;
#else
        cv_image.image = src;
#endif
        int64_t low = pointt->low;
        //RCLCPP_INFO(node->get_logger(), "pointt->low : %ld",low);
        double time_pc = low / 1000000000.0;
        //RCLCPP_INFO(node->get_logger(), "timde_pc:%f",time_pc);
        //rclcpp::Time rcv_time(time_pc, 0, RCL_ROS_TIME);
        // 用完整的纳秒数来构造时间对象
        rclcpp::Time rcv_time(low, RCL_ROS_TIME);

        cv_image.toImageMsg(*image_msg);
        image_msg->header.stamp = node->get_clock()->now();  // ros发出的时间不是快门时间
        image_msg->header.frame_id = "hikrobot_camera";

        if (rcv_time.seconds() == 0)
        {
            image_msg->header.stamp = node->get_clock()->now();
        }
        else
        {
            image_msg->header.stamp = rcv_time; // use the lidar's time
        }

        camera_info_msg->header.frame_id = image_msg->header.frame_id;
	    camera_info_msg->header.stamp = image_msg->header.stamp;
        rclcpp::Time stamp_time(image_msg->header.stamp);
        //RCLCPP_INFO(node->get_logger(), "get imt at %lf", stamp_time.seconds());
        image_pub.publish(image_msg, camera_info_msg);

        //*******************************************************************************************************************/
    }
    // 清理资源
    if (pointt) {
        munmap(pointt, sizeof(time_stamp));
    }
    return 0;
}