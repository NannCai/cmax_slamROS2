// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"
#include <glog/logging.h>
#include <gflags/gflags.h>

#include "cmax_slam.h"

int main(int argc, char* argv[])
{

  // ros::init(argc, argv, "cmax_slam");
  // ros::NodeHandle nh;
  // cmax_slam::CMaxSLAM slam(nh);
  // ros::spin();



  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);
  // gflags::AllowCommandLineReparsing();
  // google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;


  rclcpp::init(argc, argv);




  auto node = rclcpp::Node::make_shared("cmax_slam");
  cmax_slam::CMaxSLAM slam(node);
  // cmax_slam::CMaxSLAM slam = std::make_shared<CMaxSLAM>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  google::ShutdownGoogleLogging();
  return 0;
}
