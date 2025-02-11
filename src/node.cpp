#include <rclcpp/rclcpp.hpp>
#include "cmax_slam.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    // auto node = rclcpp::Node::make_shared("cmax_slam");
    auto slam = std::make_shared<cmax_slam::CMaxSLAM>("cmax_slam");
    rclcpp::spin(slam);
    rclcpp::shutdown();
    return 0;
}
