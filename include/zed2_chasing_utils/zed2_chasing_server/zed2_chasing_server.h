//
// Created by larr-laptop on 23. 8. 21.
//

#ifndef ZED2_CHASING_UTILS_ZED2_CHASING_SERVER_H
#define ZED2_CHASING_UTILS_ZED2_CHASING_SERVER_H

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "zed2_chasing_utils/zed2_chasing_info_manager/zed2_chasing_info_manager.h"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::CompressedImage,
                                                        sensor_msgs::msg::CameraInfo,
                                                        zed_interfaces::msg::ObjectsStamped>
    CompressedImageMaskBoundingBoxSync;

namespace zed2_chasing_utils {

class Zed2ChasingServer : public rclcpp::Node {
public:
  Zed2ChasingServer();

private:
  rclcpp::Node::SharedPtr node_handle_;
  ChasingInfoManager chasing_info_manager_;
  image_transport::ImageTransport image_transporter_;
  rclcpp::Time zed_call_time_;
  Param param_;

  bool is_camera_pose_received_{false};
  bool is_object_pose_received_{false};
  bool is_depth_image_received_{false};
  bool is_pcl_created_{false};

  message_filters::Subscriber<sensor_msgs::msg::CompressedImage>
      *compressed_depth_image_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> *camera_information_subscriber_;
  message_filters::Subscriber<zed_interfaces::msg::ObjectsStamped> *zed_object_subscriber_;
  message_filters::Synchronizer<CompressedImageMaskBoundingBoxSync> *subscription_synchronizer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2_ros::TransformListener *tf_listener_ptr_;
  tf2_ros::TransformBroadcaster *tf_broadcaster_ptr_;

  image_transport::Publisher masked_depth_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr masked_points_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr object_position_publisher_;

  void ZedSyncCallback(const sensor_msgs::msg::CompressedImage &compressed_depth_image,
                       const sensor_msgs::msg::CameraInfo &camera_info,
                       const zed_interfaces::msg::ObjectsStamped &zed_od);

  Pose tfCallBack(const sensor_msgs::msg::CompressedImage &compressed_depth_image);
  Pose tfObjectCallback(const zed_interfaces::msg::ObjectsStamped &object_stamped);
  cv::Mat DecompressDepthPng(const sensor_msgs::msg::CompressedImage &depth_image);
  static std_msgs::msg::Header
  GetDepthImageHeader(const sensor_msgs::msg::CompressedImage &depth_image) {
    return depth_image.header;
  }

  Pose GetPoseFromGeometryMsgs(const geometry_msgs::msg::PoseStamped &pose_stamped);
  Pose GetPoseFromTfMsgs(
      const geometry_msgs::msg::TransformStamped &tf_stamped); // TODO: How to read TF?
  geometry_msgs::msg::PoseStamped GetGeometryPoseMsgsFromPose(const Pose &pose);
  geometry_msgs::msg::PointStamped GetGeometryPointMsgsFromPose(const Pose &pose);
  sensor_msgs::msg::Image GetRosMsgsFromImage(const cv::Mat &img, const std::string &encoding_type,
                                              const std::string &frame_id, rclcpp::Time t);
};

}; // namespace zed2_chasing_utils

#endif // ZED2_CHASING_UTILS_ZED2_CHASING_SERVER_H
