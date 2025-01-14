//
// Created by larr-laptop on 23. 8. 21.
//

#ifndef ZED2_CHASING_UTILS_ZED2_CHASING_INFO_MANAGER_H
#define ZED2_CHASING_UTILS_ZED2_CHASING_INFO_MANAGER_H

#include "image_geometry/pinhole_camera_model.h"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

// #include "pcl_ros/point_cloud.hpp"
// #include "pcl_ros/transforms.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
// #include "pcl_conversions/pcl_conversions.h"
// #include "pcl_ros/point_cloud.hpp"
// #include "pcl_ros/transforms.hpp"
// #include "pcl
// #include "pcl-1.10/pcl/point_cloud.h"
#include "compressed_depth_image_transport/codec.h"
#include "compressed_depth_image_transport/compression_common.h"
#include "compressed_image_transport/compression_common.h"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "zed_msgs/msg/objects_stamped.hpp"

#include "algorithm"
#include "cv_bridge/cv_bridge.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Eigenvalues"
#include "numeric"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"

namespace zed2_chasing_utils {

struct Point {
  float x;
  float y;
  float z;

  Point() {
    x = 0.0;
    y = 0.0, z = 0.0;
  };

  Point(float x_, float y_, float z_) : x(x_), y(y_), z(z_){};
  Point(const Eigen::Vector3f &pnt) : x(pnt(0)), y(pnt(1)), z(pnt(2)){};
  Eigen::Vector3f toEigen() const { return Eigen::Vector3f(x, y, z); }
  Eigen::Vector3d toEigend() const { return Eigen::Vector3d(x, y, z); }
  Point operator+(const Point &pnt) const { return Point(pnt.x + x, pnt.y + y, pnt.z + z); }
  Point operator-(const Point &pnt) const { return Point(x - pnt.x, y - pnt.y, z - pnt.z); }
  Point operator*(float s) const { return Point(s * x, s * y, s * z); }
  Point operator/(float s) const { return *this * (float)(1.0 / s); }
  float distTo(Point p) const {
    return (float)sqrt(pow(p.x - x, 2) + pow(p.y - y, 2) + pow(p.z - z, 2));
  }
  float dot(Point p) const { return x * p.x + y * p.y + z * p.z; }
  float norm() const { return (float)sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2)); }
  Point normalized() const { return *this * (float)(1.0 / this->norm()); }
};

struct Pose {
  Eigen::Transform<float, 3, Eigen::Affine> poseMat;
  void inverse() { poseMat = poseMat.inverse(); };
  Pose() { poseMat.setIdentity(); }
  void setTranslation(const float &x, const float &y, const float &z) {
    poseMat.translate(Eigen::Vector3f(x, y, z));
  };
  void setTranslation(const Point &pnt) { poseMat.translate(pnt.toEigen()); };
  void setRotation(const Eigen::Quaternionf &quat) { poseMat.rotate(quat); };
  Point getTranslation() const {
    return Point(poseMat.translation().x(), poseMat.translation().y(), poseMat.translation().z());
  }
  Eigen::Quaternionf getQuaternion() const { return Eigen::Quaternionf(poseMat.rotation()); }
  void rotate(const Eigen::Vector3f &axis, const float &angle) {
    poseMat.rotate(Eigen::AngleAxisf(angle, axis));
  }
  void applyTransform(const Pose &pose) { poseMat = pose.poseMat * poseMat; }
};

struct Param {
  std::string global_frame_id;
  int pcl_stride;
  int mask_padding_x;
  int mask_padding_y;
};
struct State {
  bool is_target_tracked = false;
  Pose T_cw; // world to cam (optical)
  Pose T_cd; // zed cam to rear view frame
  Pose T_wc;
  Pose T_wo; // world to object (x-forwarding)
  sensor_msgs::msg::PointCloud pcl_objects_removed;
  int bbox_2d[4]; // rMin, cMin, rMax, cMax
  rclcpp::Time zed_last_call_time;
  rclcpp::Time server_last_call_time;
};

class ChasingInfoManager {
private:
  Param param_;
  State drone_state_;
  cv::Mat decomp_depth_image_;
  cv::Mat depth_image_masked_;
  std::string depth_image_frame_id_;
  u_int64_t depth_image_time_stamp_;
  std_msgs::msg::Header depth_image_header_;

public:
  ChasingInfoManager();

  void SetParameter(const std::string &global_frame_id, const int &pcl_stride,
                    const int &mask_padding_x, const int &mask_padding_y);
  void DepthCallback(const sensor_msgs::msg::CameraInfo &camera_info,
                     const zed_msgs::msg::ObjectsStamped &zed_od);
  void SetPose(const Pose &pose);
  void SetObjectPose(const Pose &pose);
  void SetDecompressedDepth(const cv::Mat &decompressed_depth);
  void SetDepthFrameId(const std::string &frame_id) { depth_image_frame_id_ = frame_id; }
  void SetDepthImageHeader(const std_msgs::msg::Header &header) { depth_image_header_ = header; }
  void SetDepthTimestamp(const u_int64_t &time_stamp) { depth_image_time_stamp_ = time_stamp; }
  cv::Mat GetMaskedImage() { return decomp_depth_image_; }
  Pose GetCameraPose() { return drone_state_.T_wc; }
  Pose GetObjectPose() { return drone_state_.T_wo; }
  sensor_msgs::msg::PointCloud GetMaskedPointCloud() { return drone_state_.pcl_objects_removed; }
};
} // namespace zed2_chasing_utils

#endif // ZED2_CHASING_UTILS_ZED2_CHASING_INFO_MANAGER_H
