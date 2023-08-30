//
// Created by larr-laptop on 23. 8. 21.
//
#include "zed2_chasing_utils/zed2_chasing_info_manager/zed2_chasing_info_manager.h"

zed2_chasing_utils::ChasingInfoManager::ChasingInfoManager() {}

void zed2_chasing_utils::ChasingInfoManager::SetParameter(const std::string &global_frame_id,
                                                          const int &pcl_stride,
                                                          const int &mask_padding_x,
                                                          const int &mask_padding_y) {
  param_.global_frame_id = global_frame_id;
  param_.pcl_stride = pcl_stride;
  param_.mask_padding_x = mask_padding_x;
  param_.mask_padding_y = mask_padding_y;
}
void zed2_chasing_utils::ChasingInfoManager::DepthCallback(
    const sensor_msgs::msg::CameraInfo &camera_info,
    const zed_interfaces::msg::ObjectsStamped &zed_od) {
  image_geometry::PinholeCameraModel model;
  model.fromCameraInfo(camera_info);
  double camera_cx = model.cx();
  double camera_cy = model.cy();
  double camera_fx = model.fx();
  double camera_fy = model.fy();
  double camera_factor = 1;
  drone_state_.pcl_objects_removed.points.clear();
  drone_state_.pcl_objects_removed.header.frame_id = param_.global_frame_id;
  drone_state_.pcl_objects_removed.header.stamp = depth_image_header_.stamp;

  int rGlobalMin = 0;
  int cGlobalMin = 0;
  int rGlobalMax = decomp_depth_image_.rows - 1;
  int cGlobalMax = decomp_depth_image_.cols - 1;
  int rMin = rGlobalMin;
  int rMax = rGlobalMax;
  int cMin = cGlobalMin;
  int cMax = cGlobalMax;
  for (int idx = 0; idx < zed_od.objects.size(); idx++) {
    rMin =
        std::max(int(zed_od.objects[idx].bounding_box_2d.corners[0].kp[1]) - param_.mask_padding_y,
                 rGlobalMin);
    cMin =
        std::max(int(zed_od.objects[idx].bounding_box_2d.corners[0].kp[0]) - param_.mask_padding_x,
                 cGlobalMin);
    rMax =
        std::min(int(zed_od.objects[idx].bounding_box_2d.corners[2].kp[1]) + param_.mask_padding_y,
                 rGlobalMax);
    cMax =
        std::min(int(zed_od.objects[idx].bounding_box_2d.corners[2].kp[0]) + param_.mask_padding_x,
                 cGlobalMax);

    const float NaN = std::numeric_limits<float>::quiet_NaN();
    // Depth Image Masking
    for (int r = rMin; r < rMax; r++) {
      for (int c = cMin; c < cMax; c++) {
        decomp_depth_image_.ptr<float>(r)[c] = NaN;
      }
    }
    drone_state_.bbox_2d[0] = rMin;
    drone_state_.bbox_2d[1] = cMin;
    drone_state_.bbox_2d[2] = rMax;
    drone_state_.bbox_2d[3] = cMax;
  }
  if (zed_od.objects.empty()) {
    const float NaN = std::numeric_limits<float>::quiet_NaN();
    // Depth Image Masking
    for (int r = drone_state_.bbox_2d[0]; r < drone_state_.bbox_2d[2]; r++) {
      for (int c = drone_state_.bbox_2d[1]; c < drone_state_.bbox_2d[3]; c++) {
        decomp_depth_image_.ptr<float>(r)[c] = NaN;
      }
    }
  }
  // Point-cloud Generation
  float pcl_pts[3];
  for (int r = 0; r < decomp_depth_image_.rows; r += param_.pcl_stride) {
    for (int c = 0; c < decomp_depth_image_.cols; c += param_.pcl_stride) {
      float d = decomp_depth_image_.ptr<float>(r)[c];
      if (not isnan(d)) {
        pcl_pts[2] = (float)(d / camera_factor);
        pcl_pts[0] = (float)((c - camera_cx) * pcl_pts[2] / camera_fx);
        pcl_pts[1] = (float)((r - camera_cy) * pcl_pts[2] / camera_fy);
        // point cloud transformation
        Point p_w = drone_state_.T_wc.poseMat * Point(pcl_pts[0], pcl_pts[1], pcl_pts[2]).toEigen();
        geometry_msgs::msg::Point32 p;
        p.x = p_w.x;
        p.y = p_w.y;
        p.z = p_w.z;
        drone_state_.pcl_objects_removed.points.push_back(p);
      }
    }
  }
}
void zed2_chasing_utils::ChasingInfoManager::SetPose(const zed2_chasing_utils::Pose &pose) {
  drone_state_.T_wc = pose;
  drone_state_.T_cw = drone_state_.T_wc;
  drone_state_.T_cw.inverse();
}
void zed2_chasing_utils::ChasingInfoManager::SetObjectPose(const zed2_chasing_utils::Pose &pose) {
  if (not(isnan(pose.getTranslation().x) or isnan(pose.getTranslation().y) or
          isnan(pose.getTranslation().z)))
    drone_state_.T_wo = pose;
}
void zed2_chasing_utils::ChasingInfoManager::SetDecompressedDepth(
    const cv::Mat &decompressed_depth) {
  decomp_depth_image_ = decompressed_depth;
}
