//
// Created by larr-laptop on 23. 8. 21.
//

#ifndef ZED2_CHASING_UTILS_ZED2_CHASING_SERVER_H
#define ZED2_CHASING_UTILS_ZED2_CHASING_SERVER_H
#include "rclcpp/rclcpp.hpp"
#include "zed2_chasing_utils/zed2_chasing_info_manager/zed2_chasing_info_manager.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"


namespace zed2_chasing_utils  {

    class Zed2ChasingServer: public rclcpp::Node {
    public:
        Zed2ChasingServer();
    private:
//        message_filters::Subscriber

    };

};


#endif //ZED2_CHASING_UTILS_ZED2_CHASING_SERVER_H
