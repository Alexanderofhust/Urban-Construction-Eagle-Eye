/*
 * output_helper.cpp
 *
 *  Created on: Jan 20, 2013
 *      Author: chrigi
 */

 #include <vikit/output_helper.h>
 #include <visualization_msgs/msg/marker.hpp>
 #include <geometry_msgs/msg/point.hpp>
 #include <tf2_ros/transform_broadcaster.h>
 #include <tf2/LinearMath/Quaternion.h>
 
 namespace vk {
 namespace output_helper {
 
 void
 publishTfTransform(const Sophus::SE3& T, const rclcpp::Time& stamp,
                    const std::string& frame_id, const std::string& child_frame_id,
                    tf2_ros::TransformBroadcaster& br)
 {
   geometry_msgs::msg::TransformStamped transform_msg;
   Eigen::Quaterniond q(T.rotation_matrix());
   transform_msg.transform.translation.x = T.translation().x();
   transform_msg.transform.translation.y = T.translation().y();
   transform_msg.transform.translation.z = T.translation().z();
   
   tf2::Quaternion tf_q;
   tf_q.setX(q.x());
   tf_q.setY(q.y());
   tf_q.setZ(q.z());
   tf_q.setW(q.w());
   transform_msg.transform.rotation.x = tf_q.x();
   transform_msg.transform.rotation.y = tf_q.y();
   transform_msg.transform.rotation.z = tf_q.z();
   transform_msg.transform.rotation.w = tf_q.w();
   
   transform_msg.header.stamp = stamp;
   transform_msg.header.frame_id = frame_id;
   transform_msg.child_frame_id = child_frame_id;
   
   br.sendTransform(transform_msg);
 }
 
 void
 publishPointMarker(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                    const Eigen::Vector3d& pos,
                    const std::string& ns,
                    const rclcpp::Time& timestamp,
                    int id,
                    int action,
                    double marker_scale,
                    const Eigen::Vector3d& color,
                    rclcpp::Duration lifetime)
 {
   visualization_msgs::msg::Marker msg;
   msg.header.frame_id = "/world";
   msg.header.stamp = timestamp;
   msg.ns = ns;
   msg.id = id;
   msg.type = visualization_msgs::msg::Marker::CUBE;
   msg.action = action; // 0 = add/modify
   msg.scale.x = marker_scale;
   msg.scale.y = marker_scale;
   msg.scale.z = marker_scale;
   msg.color.a = 1.0;
   msg.color.r = color[0];
   msg.color.g = color[1];
   msg.color.b = color[2];
   msg.lifetime = lifetime;
   msg.pose.position.x = pos[0];
   msg.pose.position.y = pos[1];
   msg.pose.position.z = pos[2];
   pub->publish(msg);
 }
 
 void
 publishLineMarker(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                   const Eigen::Vector3d& start,
                   const Eigen::Vector3d& end,
                   const std::string& ns,
                   const rclcpp::Time& timestamp,
                   int id,
                   int action,
                   double marker_scale,
                   const Eigen::Vector3d& color,
                   rclcpp::Duration lifetime)
 {
   visualization_msgs::msg::Marker msg;
   msg.header.frame_id = "/world";
   msg.header.stamp = timestamp;
   msg.ns = ns;
   msg.id = id;
   msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
   msg.action = action; // 0 = add/modify
   msg.scale.x = marker_scale;
   msg.color.a = 1.0;
   msg.color.r = color[0];
   msg.color.g = color[1];
   msg.color.b = color[2];
   msg.points.resize(2);
   msg.lifetime = lifetime;
   msg.points[0].x = start[0];
   msg.points[0].y = start[1];
   msg.points[0].z = start[2];
   msg.points[1].x = end[0];
   msg.points[1].y = end[1];
   msg.points[1].z = end[2];
   pub->publish(msg);
 }
 
 
 void
 publishArrowMarker(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                    const Eigen::Vector3d& pos,
                    const Eigen::Vector3d& dir,
                    double scale,
                    const std::string& ns,
                    const rclcpp::Time& timestamp,
                    int id,
                    int action,
                    double marker_scale,
                    const Eigen::Vector3d& color)
 {
   visualization_msgs::msg::Marker msg;
   msg.header.frame_id = "/world";
   msg.header.stamp = timestamp;
   msg.ns = ns;
   msg.id = id;
   msg.type = visualization_msgs::msg::Marker::ARROW;
   msg.action = action; // 0 = add/modify
   msg.scale.x = marker_scale;
   msg.scale.y = marker_scale*0.35;
   msg.scale.z = 0.0;
   msg.color.a = 1.0;
   msg.color.r = color[0];
   msg.color.g = color[1];
   msg.color.b = color[2];
   msg.points.resize(2);
   msg.points[0].x = pos[0];
   msg.points[0].y = pos[1];
   msg.points[0].z = pos[2];
   msg.points[1].x = pos[0] + scale*dir[0];
   msg.points[1].y = pos[1] + scale*dir[1];
   msg.points[1].z = pos[2] + scale*dir[2];
   pub->publish(msg);
 }
 
 void
 publishHexacopterMarker(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                         const std::string& frame_id,
                         const std::string& ns,
                         const rclcpp::Time& timestamp,
                         int id,
                         int action,
                         double marker_scale,
                         const Eigen::Vector3d& color)
 {
   const double sqrt2_2 = sqrt(2) / 2;
 
   visualization_msgs::msg::Marker marker;
 
   // the marker will be displayed in frame_id
   marker.header.frame_id = frame_id;
   marker.header.stamp = timestamp;
   marker.ns = ns;
   marker.action = 0;
   marker.id = id;
 
   // make rotors
   marker.type = visualization_msgs::msg::Marker::CYLINDER;
   marker.scale.x = 0.2 * marker_scale;
   marker.scale.y = 0.2 * marker_scale;
   marker.scale.z = 0.01 * marker_scale;
   marker.color.r = 0.4;
   marker.color.g = 0.4;
   marker.color.b = 0.4;
   marker.color.a = 0.8;
   marker.pose.position.z = 0;
 
   // front left/right
   marker.pose.position.x = 0.19 * marker_scale;
   marker.pose.position.y = 0.11 * marker_scale;
   marker.id--;
   pub->publish(marker);
 
   marker.pose.position.x = 0.19 * marker_scale;
   marker.pose.position.y = -0.11 * marker_scale;
   marker.id--;
   pub->publish(marker);
 
   // left/right
   marker.pose.position.x = 0;
   marker.pose.position.y = 0.22 * marker_scale;
   marker.id--;
   pub->publish(marker);
 
   marker.pose.position.x = 0;
   marker.pose.position.y = -0.22 * marker_scale;
   marker.id--;
   pub->publish(marker);
 
   // back left/right
   marker.pose.position.x = -0.19 * marker_scale;
   marker.pose.position.y = 0.11 * marker_scale;
   marker.id--;
   pub->publish(marker);
 
   marker.pose.position.x = -0.19 * marker_scale;
   marker.pose.position.y = -0.11 * marker_scale;
   marker.id--;
   pub->publish(marker);
 
   // make arms
   marker.type = visualization_msgs::msg::Marker::CUBE;
   marker.scale.x = 0.44 * marker_scale;
   marker.scale.y = 0.02 * marker_scale;
   marker.scale.z = 0.01 * marker_scale;
   marker.color.r = color[0];
   marker.color.g = color[1];
   marker.color.b = color[2];
   marker.color.a = 1;
 
   marker.pose.position.x = 0;
   marker.pose.position.y = 0;
   marker.pose.position.z = -0.015 * marker_scale;
   marker.pose.orientation.x = 0;
   marker.pose.orientation.y = 0;
 
   marker.pose.orientation.w = sqrt2_2;
   marker.pose.orientation.z = sqrt2_2;
   marker.id--;
   pub->publish(marker);
 
   // 30 deg rotation
   marker.pose.orientation.w = 0.9659;
   marker.pose.orientation.z = 0.2588;
   marker.id--;
   pub->publish(marker);
 
   marker.pose.orientation.w = 0.9659;
   marker.pose.orientation.z = -0.2588;
   marker.id--;
   pub->publish(marker);
 }
 
 void
 publishCameraMarker(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                     const std::string& frame_id,
                     const std::string& ns,
                     const rclcpp::Time& timestamp,
                     int id,
                     double marker_scale,
                     const Eigen::Vector3d& color)
 {
   const double sqrt2_2 = sqrt(2) / 2;
 
   visualization_msgs::msg::Marker marker;
 
   // the marker will be displayed in frame_id
   marker.header.frame_id = frame_id;
   marker.header.stamp = timestamp;
   marker.ns = ns;
   marker.action = 0;
   marker.id = id;
 
   // make rectangles as frame
   double r_w = 1.0;
   double z_plane = (r_w / 2.0) * marker_scale;
   marker.pose.position.x = 0;
   marker.pose.position.y = (r_w / 4.0) * marker_scale;
   marker.pose.position.z = z_plane;
 
   marker.type = visualization_msgs::msg::Marker::CUBE;
   marker.scale.x = r_w * marker_scale;
   marker.scale.y = 0.04 * marker_scale;
   marker.scale.z = 0.04 * marker_scale;
   marker.color.r = color[0];
   marker.color.g = color[1];
   marker.color.b = color[2];
   marker.color.a = 1;
 
   marker.pose.orientation.x = 0;
   marker.pose.orientation.y = 0;
   marker.pose.orientation.z = 0;
   marker.pose.orientation.w = 1;
   marker.id--;
   pub->publish(marker);
 
   marker.pose.position.y = -(r_w / 4.0) * marker_scale;
   marker.id--;
   pub->publish(marker);
 
   marker.scale.x = (r_w / 2.0) * marker_scale;
   marker.pose.position.x = (r_w / 2.0) * marker_scale;
   marker.pose.position.y = 0;
   marker.pose.orientation.w = sqrt2_2;
   marker.pose.orientation.z = sqrt2_2;
   marker.id--;
   pub->publish(marker);
 
   marker.pose.position.x = -(r_w / 2.0) * marker_scale;
   marker.id--;
   pub->publish(marker);
 
   // make pyramid edges
   marker.scale.x = (3.0 * r_w / 4.0) * marker_scale;
   marker.pose.position.z = 0.5 * z_plane;
 
   marker.pose.position.x = (r_w / 4.0) * marker_scale;
   marker.pose.position.y = (r_w / 8.0) * marker_scale;
   marker.pose.orientation.x = 0.08198092;
   marker.pose.orientation.y = -0.34727674;
   marker.pose.orientation.z = 0.21462883;
   marker.pose.orientation.w = 0.9091823;
   marker.id--;
   pub->publish(marker);
 
   marker.pose.position.x = -(r_w / 4.0) * marker_scale;
   marker.pose.position.y = (r_w / 8.0) * marker_scale;
   marker.pose.orientation.x = 0.08198092;
   marker.pose.orientation.y = 0.34727674;
   marker.pose.orientation.z = -0.21462883;
   marker.pose.orientation.w = 0.9091823;
   marker.id--;
   pub->publish(marker);
 
   marker.pose.position.x = -(r_w / 4.0) * marker_scale;
   marker.pose.position.y = -(r_w / 8.0) * marker_scale;
   marker.pose.orientation.x = -0.08198092;
   marker.pose.orientation.y = 0.34727674;
   marker.pose.orientation.z = 0.21462883;
   marker.pose.orientation.w = 0.9091823;
   marker.id--;
   pub->publish(marker);
 
   marker.pose.position.x = (r_w / 4.0) * marker_scale;
   marker.pose.position.y = -(r_w / 8.0) * marker_scale;
   marker.pose.orientation.x = -0.08198092;
   marker.pose.orientation.y = -0.34727674;
   marker.pose.orientation.z = -0.21462883;
   marker.pose.orientation.w = 0.9091823;
   marker.id--;
   pub->publish(marker);
 }
 
 void publishFrameMarker(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                         const Eigen::Matrix3d& rot,
                         const Eigen::Vector3d& pos,
                         const std::string& ns,
                         const rclcpp::Time& timestamp,
                         int id,
                         int action,
                         double marker_scale,
                         rclcpp::Duration lifetime)
 {
   visualization_msgs::msg::Marker marker;
   marker.header.frame_id = "/world";
   marker.header.stamp = timestamp;
   marker.ns = ns;
   marker.id = id++;
   marker.type = visualization_msgs::msg::Marker::ARROW;
   marker.action = action; // 0 = add/modify
   marker.points.reserve(2);
   geometry_msgs::msg::Point point;
   point.x = static_cast<float>(pos.x());
   point.y = static_cast<float>(pos.y());
   point.z = static_cast<float>(pos.z());
   marker.points.push_back(point);
   point.x = static_cast<float>(pos.x() + marker_scale * rot(0, 2)); // Draw arrow in z-direction
   point.y = static_cast<float>(pos.y() + marker_scale * rot(1, 2)); // Draw arrow in z-direction
   point.z = static_cast<float>(pos.z() + marker_scale * rot(2, 2)); // Draw arrow in z-direction
   marker.points.push_back(point);
   marker.scale.x = 0.5 * marker_scale;
   marker.scale.y = 0.5 * marker_scale;
   marker.color.a = 1.0;
   marker.color.r = 0.0;
   marker.color.g = 0.0;
   marker.color.b = 1.0;
   marker.lifetime = lifetime;
   pub->publish(marker);
 
   marker.id = id++;
   marker.points.clear();
   point.x = static_cast<float>(pos.x());
   point.y = static_cast<float>(pos.y());
   point.z = static_cast<float>(pos.z());
   marker.points.push_back(point);
   point.x = static_cast<float>(pos.x() + marker_scale * rot(0, 0)); // Draw arrow in x-direction
   point.y = static_cast<float>(pos.y() + marker_scale * rot(1, 0)); // Draw arrow in x-direction
   point.z = static_cast<float>(pos.z() + marker_scale * rot(2, 0)); // Draw arrow in x-direction
   marker.points.push_back(point);
   marker.color.r = 1.0;
   marker.color.g = 0.0;
   marker.color.b = 0.0;
   marker.lifetime = lifetime;
   pub->publish(marker);
 
   marker.id = id++;
   marker.points.clear();
   point.x = static_cast<float>(pos.x());
   point.y = static_cast<float>(pos.y());
   point.z = static_cast<float>(pos.z());
   marker.points.push_back(point);
   point.x = static_cast<float>(pos.x() + marker_scale * rot(0, 1)); // Draw arrow in y-direction
   point.y = static_cast<float>(pos.y() + marker_scale * rot(1, 1)); // Draw arrow in y-direction
   point.z = static_cast<float>(pos.z() + marker_scale * rot(2, 1)); // Draw arrow in y-direction
   marker.points.push_back(point);
   marker.color.r = 0.0;
   marker.color.g = 1.0;
   marker.color.b = 0.0;
   marker.lifetime = lifetime;
   pub->publish(marker);
 }
 
 
 } // namespace output_helper
 } // namespace vk