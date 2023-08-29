//
// Created by larr-laptop on 23. 8. 21.
//

#include "zed2_chasing_utils/zed2_chasing_server/zed2_chasing_server.h"

zed2_chasing_utils::Zed2ChasingServer::Zed2ChasingServer() : Node("zed2_chasing_server"), node_handle_(
        std::shared_ptr<Zed2ChasingServer>(this, [](auto *) {})),
                                                             image_transporter_(node_handle_) {

    // Read parameters from yaml
    get_parameter<std::string>("global_frame_id", param_.global_frame_id);
    get_parameter<int>("pcl_stride", param_.pcl_stride);
    get_parameter<int>("mask_padding_x", param_.mask_padding_x);
    get_parameter<int>("mask_padding_y", param_.mask_padding_y);
    // Update parameters of chasing information manager
    chasing_info_manager_.SetParameter(param_.global_frame_id, param_.mask_padding_x, param_.mask_padding_y);

    // Subscriber
    compressed_depth_image_subscriber_ = new message_filters::Subscriber<sensor_msgs::msg::CompressedImage>(
            node_handle_, "/depth_compressed_image");
    camera_information_subscriber_ = new message_filters::Subscriber<sensor_msgs::msg::CameraInfo>(node_handle_,
                                                                                                   "/camera_info"); // TODO: QoS
    zed_object_subscriber_ = new message_filters::Subscriber<zed_interfaces::msg::ObjectsStamped>(node_handle_,
                                                                                                  "/objects");

    subscription_synchronizer_ = new message_filters::Synchronizer<CompressedImageMaskBoundingBoxSync>(
            CompressedImageMaskBoundingBoxSync(10), *this->compressed_depth_image_subscriber_,
            *this->camera_information_subscriber_, *this->zed_object_subscriber_);
    subscription_synchronizer_->registerCallback(&Zed2ChasingServer::ZedSyncCallback, this); //TODO: bind

    // Publisher
    masked_points_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/masked_points", 1);
    masked_depth_image_publisher_ = image_transporter_.advertise("/masked_depth_image", 1);
    object_position_publisher_ = create_publisher<geometry_msgs::msg::PointStamped>("/target_position", 1);

    tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ptr_ = new tf2_ros::TransformListener(*tf_buffer_);
    tf_broadcaster_ptr_ = new tf2_ros::TransformBroadcaster(*this);

}

void zed2_chasing_utils::Zed2ChasingServer::ZedSyncCallback(const sensor_msgs::msg::CompressedImage &,
                                                            const sensor_msgs::msg::CameraInfo &,
                                                            const zed_interfaces::msg::ObjectsStamped) {

}
