#include "zed2_chasing_utils/zed2_chasing_server/zed2_chasing_server.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    printf("@@@@@@@@@@@@@@@@\n");
    auto node = std::make_shared<zed2_chasing_utils::Zed2ChasingServer>();
    printf("!!!!!!!!!!!!!!\n");
    rclcpp::spin(node);
    printf("????????????\n");
    rclcpp::shutdown();
    return 0;
}
