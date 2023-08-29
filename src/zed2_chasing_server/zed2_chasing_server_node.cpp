#include "zed2_chasing_utils/zed2_chasing_server/zed2_chasing_server.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<zed2_chasing_utils::Zed2ChasingServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
