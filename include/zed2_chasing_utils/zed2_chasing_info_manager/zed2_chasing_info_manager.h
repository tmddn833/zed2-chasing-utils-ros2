//
// Created by larr-laptop on 23. 8. 21.
//

#ifndef ZED2_CHASING_UTILS_ZED2_CHASING_INFO_MANAGER_H
#define ZED2_CHASING_UTILS_ZED2_CHASING_INFO_MANAGER_H

#include "image_transport/image_transport.hpp"
#include "image_geometry/pinhole_camera_model.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
//#include "pcl_conversions/pcl_conversions.h"


#include "compressed_depth_image_transport/codec.h"
#include "compressed_depth_image_transport/compression_common.h"
#include "compressed_image_transport/compression_common.h"

#include "zed_interfaces/msg/objects_stamped.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "algorithm"
#include "numeric"

namespace zed2_chasing_utils {
    struct Param {
        std::string global_frame_id;
        int pcl_stride;
        int mask_padding_x;
        int mask_padding_y;
    };
    class ChasingInfoManager {
    private:
        Param param_;
    public:
        ChasingInfoManager();
        void SetParameter(const std::string &global_frame_id, const int &mask_padding_x, const int &mask_padding_y);
    };
}


#endif //ZED2_CHASING_UTILS_ZED2_CHASING_INFO_MANAGER_H
