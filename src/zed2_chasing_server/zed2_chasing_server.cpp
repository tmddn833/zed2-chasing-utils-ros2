//
// Created by larr-laptop on 23. 8. 21.
//

#include "zed2_chasing_utils/zed2_chasing_server/zed2_chasing_server.h"

zed2_chasing_utils::Zed2ChasingServer::Zed2ChasingServer()
    : Node("zed2_chasing_server"),
      node_handle_(std::shared_ptr<Zed2ChasingServer>(this, [](auto *) {})),
      image_transporter_(node_handle_) {
  printf("HELLLOOO\n");
  // Read parameters from yaml
  get_parameter<std::string>("global_frame_id", param_.global_frame_id);
  printf("GLOBAL FRAME ID %s \n",param_.global_frame_id.c_str());
  get_parameter<int>("pcl_stride", param_.pcl_stride);
  get_parameter<int>("mask_padding_x", param_.mask_padding_x);
  get_parameter<int>("mask_padding_y", param_.mask_padding_y);
  // Update parameters of chasing information manager
  chasing_info_manager_.SetParameter(param_.global_frame_id, param_.pcl_stride,
                                     param_.mask_padding_x, param_.mask_padding_y);

  // Subscriber
  compressed_depth_image_subscriber_ =
      new message_filters::Subscriber<sensor_msgs::msg::CompressedImage>(
          node_handle_, "~/depth_compressed_image");
  camera_information_subscriber_ =
      new message_filters::Subscriber<sensor_msgs::msg::CameraInfo>(node_handle_, "~/camera_info");
  zed_object_subscriber_ = new message_filters::Subscriber<zed_interfaces::msg::ObjectsStamped>(
      node_handle_, "~/objects");

  subscription_synchronizer_ =
      new message_filters::Synchronizer<CompressedImageMaskBoundingBoxSync>(
          CompressedImageMaskBoundingBoxSync(10), *this->compressed_depth_image_subscriber_,
          *this->camera_information_subscriber_, *this->zed_object_subscriber_);
  subscription_synchronizer_->registerCallback(&Zed2ChasingServer::ZedSyncCallback, this);

  // Publisher
  masked_points_publisher_ = create_publisher<sensor_msgs::msg::PointCloud>("masked_points", 1);
  masked_depth_image_publisher_ = image_transporter_.advertise("masked_depth_image", 1);
  object_position_publisher_ =
      create_publisher<geometry_msgs::msg::PointStamped>("target_position", 1);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ptr_ = new tf2_ros::TransformListener(*tf_buffer_);
  tf_broadcaster_ptr_ = new tf2_ros::TransformBroadcaster(*this);
}

void zed2_chasing_utils::Zed2ChasingServer::ZedSyncCallback(
    const sensor_msgs::msg::CompressedImage &compressed_depth_image,
    const sensor_msgs::msg::CameraInfo &camera_info,
    const zed_interfaces::msg::ObjectsStamped &zed_object_detection) {
  printf("1111111111111\n");
  chasing_info_manager_.SetPose(this->tfCallBack(compressed_depth_image));
  printf("2222222222222\n");
  chasing_info_manager_.SetObjectPose(this->tfObjectCallback(zed_object_detection));
  printf("33333333333333\n");
  chasing_info_manager_.SetDecompressedDepth(this->DecompressDepthPng(compressed_depth_image));
  printf("444444444444\n");
  chasing_info_manager_.SetDepthImageHeader(this->GetDepthImageHeader(compressed_depth_image));
  printf("55555555555555\n");
  chasing_info_manager_.DepthCallback(camera_info, zed_object_detection);
  printf("666666666666666\n");
  if ((not isnan(chasing_info_manager_.GetObjectPose().getTranslation().x)) and
      (not isnan(chasing_info_manager_.GetObjectPose().getTranslation().y)) and
      (not isnan(chasing_info_manager_.GetObjectPose().getTranslation().z)))
    object_position_publisher_->publish(
        GetGeometryPointMsgsFromPose(chasing_info_manager_.GetObjectPose()));

  if (not chasing_info_manager_.GetMaskedPointCloud().points.empty())
    masked_points_publisher_->publish(chasing_info_manager_.GetMaskedPointCloud());

  masked_depth_image_publisher_.publish(GetRosMsgsFromImage(
      chasing_info_manager_.GetMaskedImage(), sensor_msgs::image_encodings::TYPE_32FC1,
      compressed_depth_image.header.frame_id, compressed_depth_image.header.stamp));
}
zed2_chasing_utils::Pose zed2_chasing_utils::Zed2ChasingServer::tfCallBack(
    const sensor_msgs::msg::CompressedImage &compressed_depth_image) {
  printf("AAAAAAAAAAAAAAAAA \n");
  rclcpp::Time current_sensor_time = compressed_depth_image.header.stamp;
  zed_call_time_ = current_sensor_time;
  geometry_msgs::msg::TransformStamped transfrom_temp;
  printf("BBBBBBBBBBBBBBBBB \n");
  try {
    transfrom_temp = tf_buffer_->lookupTransform(
        param_.global_frame_id, compressed_depth_image.header.frame_id, current_sensor_time);
    is_camera_pose_received_ = true;
    return GetPoseFromTfMsgs(transfrom_temp);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR_STREAM(get_logger(), ex.what());
    RCLCPP_ERROR(get_logger(), "[Zed2 Chasing Server] No transform between map and object header. "
                               "Cannot process further.");
    Pose dummy;
    dummy.setTranslation(0.0, 0.0, 0.0);
    dummy.setRotation(Eigen::Quaternionf(1.0, 0.0, 0.0, 0.0));
    is_camera_pose_received_ = false;
    return dummy;
  }
}
zed2_chasing_utils::Pose zed2_chasing_utils::Zed2ChasingServer::tfObjectCallback(
    const zed_interfaces::msg::ObjectsStamped &object_stamped) {
  float temp_x{0.0}, temp_y{0.0}, temp_z{0.0};
  bool is_nan_head_pos = false;
  for (int idx = 0; idx < object_stamped.objects.size(); idx++) {
    for (int i = 0; i < 3; i++) {
      if (isnan(object_stamped.objects[idx].head_position[i]))
        is_nan_head_pos = true;
    }
    if (not is_nan_head_pos) {
      temp_x = object_stamped.objects[idx].head_position[0];
      temp_y = object_stamped.objects[idx].head_position[1];
      temp_z = object_stamped.objects[idx].head_position[2];
    } else {
      bool is_nan_head_bbox = false;
      for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 3; j++) {
          if (isnan(object_stamped.objects[idx].head_bounding_box_3d.corners[i].kp[j]))
            is_nan_head_bbox = true;
        }
      }
      if (not is_nan_head_bbox) {
        std::vector<Point> head_bbox;
        Point tempCorners{0.0, 0.0, 0.0};
        for (auto elem : object_stamped.objects[idx].head_bounding_box_3d.corners) {
          tempCorners.x = elem.kp[0];
          tempCorners.y = elem.kp[1];
          tempCorners.z = elem.kp[2];
          head_bbox.push_back(tempCorners);
        }
        for (auto &i : head_bbox) {
          temp_x += i.x;
          temp_y += i.y;
          temp_z += i.z;
        }
        temp_x = temp_x / (float)head_bbox.size();
        temp_y = temp_y / (float)head_bbox.size();
        temp_z = temp_z / (float)head_bbox.size();
      } else {
        bool is_nan_body_bbox = false;
        for (int i = 0; i < 8; i++) {
          for (int j = 0; j < 3; j++) {
            if (isnan(object_stamped.objects[idx].bounding_box_3d.corners[i].kp[j]))
              is_nan_body_bbox = true;
          }
        }
        if (not is_nan_body_bbox) {
          std::vector<Point> body_bbox;
          Point tempCorners{0.0, 0.0, 0.0};
          for (auto elem : object_stamped.objects[idx].bounding_box_3d.corners) {
            tempCorners.x = elem.kp[0];
            tempCorners.y = elem.kp[1];
            tempCorners.z = elem.kp[2];
            body_bbox.push_back(tempCorners);
          }
          for (auto &i : body_bbox) {
            temp_x += i.x;
            temp_y += i.y;
            temp_z += i.z;
          }
          temp_x = temp_x / (float)body_bbox.size();
          temp_y = temp_y / (float)body_bbox.size();
          temp_z = temp_z / (float)body_bbox.size();
        } else {
          const float NaN = std::numeric_limits<float>::quiet_NaN();
          temp_x = NaN;
          temp_y = NaN;
          temp_z = NaN;
        }
      }
    }
  }
  if (object_stamped.objects.empty()) {
    const float NaN = std::numeric_limits<float>::quiet_NaN();
    temp_x = NaN;
    temp_y = NaN;
    temp_z = NaN;
  }
  Pose tempPose;
  tempPose.setTranslation(temp_x, temp_y, temp_z);
  tempPose.setRotation(Eigen::Quaternionf(1.0, 0.0, 0.0, 0.0));
  return tempPose;
}
cv::Mat zed2_chasing_utils::Zed2ChasingServer::DecompressDepthPng(
    const sensor_msgs::msg::CompressedImage &depth_image) {
  cv::Mat decompressedTemp;
  cv::Mat decompressed;
  const size_t split_pos = depth_image.format.find(';');
  const std::string image_encoding = depth_image.format.substr(0, split_pos);

  if (depth_image.data.size() > sizeof(compressed_depth_image_transport::ConfigHeader)) {
    compressed_depth_image_transport::ConfigHeader compressionConfig{};
    memcpy(&compressionConfig, &depth_image.data[0], sizeof(compressionConfig));
    const std::vector<uint8_t> imageData(depth_image.data.begin() + sizeof(compressionConfig),
                                         depth_image.data.end());

    if (sensor_msgs::image_encodings::bitDepth(image_encoding) == 32)
      try {
        decompressedTemp = cv::imdecode(imageData, cv::IMREAD_UNCHANGED);
      } catch (cv::Exception &e) {
        RCLCPP_ERROR(get_logger(), "%s", e.what());
        is_depth_image_received_ = false;
        return (cv::Mat(1, 1, CV_32FC1));
      }
    size_t rows = decompressedTemp.rows;
    size_t cols = decompressedTemp.cols;

    if ((rows > 0) && (cols > 0)) {
      decompressed = cv::Mat(rows, cols, CV_32FC1);

      // Depth conversion
      auto itDepthImg = decompressed.begin<float>(), itDepthImg_end = decompressed.end<float>();
      auto itInvDepthImg = decompressedTemp.begin<unsigned short>(),
           itInvDepthImg_end = decompressedTemp.end<unsigned short>();

      float depthQuantA = compressionConfig.depthParam[0];
      float depthQuantB = compressionConfig.depthParam[1];

      for (; (itDepthImg != itDepthImg_end) && (itInvDepthImg != itInvDepthImg_end);
           ++itDepthImg, ++itInvDepthImg) {
        // check for NaN & max depth
        if (*itInvDepthImg) {
          *itDepthImg = depthQuantA / ((float)*itInvDepthImg - depthQuantB);
        } else {
          *itDepthImg = std::numeric_limits<float>::quiet_NaN();
        }
      }
      is_depth_image_received_ = true;
      return decompressed;
    } else {
      is_depth_image_received_ = false;
      return (cv::Mat(1, 1, CV_32FC1));
    }
  } else {
    is_depth_image_received_ = false;
    return (cv::Mat(1, 1, CV_32FC1));
  }
}
zed2_chasing_utils::Pose zed2_chasing_utils::Zed2ChasingServer::GetPoseFromGeometryMsgs(
    const geometry_msgs::msg::PoseStamped &pose_stamped) {
  Pose temp_pose;
  temp_pose.poseMat.setIdentity();
  Eigen::Vector3f loc((float)pose_stamped.pose.position.x, (float)pose_stamped.pose.position.y,
                      (float)pose_stamped.pose.position.z);
  temp_pose.poseMat.translate(loc);
  Eigen::Quaternionf quaternionf;
  quaternionf.setIdentity();
  quaternionf.w() = (float)pose_stamped.pose.orientation.w;
  quaternionf.x() = (float)pose_stamped.pose.orientation.x;
  quaternionf.y() = (float)pose_stamped.pose.orientation.y;
  quaternionf.z() = (float)pose_stamped.pose.orientation.z;
  temp_pose.poseMat.rotate(quaternionf);
  return temp_pose;
}

sensor_msgs::msg::Image zed2_chasing_utils::Zed2ChasingServer::GetRosMsgsFromImage(
    const cv::Mat &img, const std::string &encoding_type, const std::string &frame_id,
    rclcpp::Time t) {}
geometry_msgs::msg::PointStamped
zed2_chasing_utils::Zed2ChasingServer::GetGeometryPointMsgsFromPose(
    const zed2_chasing_utils::Pose &pose) {
  geometry_msgs::msg::PointStamped temp_point;
  temp_point.point.x = pose.getTranslation().x;
  temp_point.point.y = pose.getTranslation().y;
  temp_point.point.z = pose.getTranslation().z;
  temp_point.header.frame_id = param_.global_frame_id;
  return temp_point;
}
zed2_chasing_utils::Pose zed2_chasing_utils::Zed2ChasingServer::GetPoseFromTfMsgs(
    const geometry_msgs::msg::TransformStamped &tf_stamped) {
  Pose temp_pose;
  temp_pose.poseMat.setIdentity();
  Eigen::Vector3f loc((float)tf_stamped.transform.translation.x,
                      (float)tf_stamped.transform.translation.y,
                      (float)tf_stamped.transform.translation.z);
  temp_pose.poseMat.translate(loc);
  Eigen::Quaternionf quaternionf;
  quaternionf.setIdentity();
  quaternionf.w() = (float)tf_stamped.transform.rotation.w;
  quaternionf.x() = (float)tf_stamped.transform.rotation.x;
  quaternionf.y() = (float)tf_stamped.transform.rotation.y;
  quaternionf.z() = (float)tf_stamped.transform.rotation.z;
  temp_pose.poseMat.rotate(quaternionf);
  return temp_pose;
}
geometry_msgs::msg::PoseStamped zed2_chasing_utils::Zed2ChasingServer::GetGeometryPoseMsgsFromPose(
    const zed2_chasing_utils::Pose &pose) {
  geometry_msgs::msg::PoseStamped tempPose;
  tempPose.pose.position.x = pose.getTranslation().x;
  tempPose.pose.position.y = pose.getTranslation().y;
  tempPose.pose.position.z = pose.getTranslation().z;

  tempPose.pose.orientation.w = pose.getQuaternion().w();
  tempPose.pose.orientation.x = pose.getQuaternion().x();
  tempPose.pose.orientation.y = pose.getQuaternion().y();
  tempPose.pose.orientation.z = pose.getQuaternion().z();

  tempPose.header.frame_id = param_.global_frame_id;

  return tempPose;
}
