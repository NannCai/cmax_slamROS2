#pragma once

#include <thread>
#include <fstream>

#include "frontend/ang_vel_estimator.h"
#include "backend/pose_graph_optimizer.h"

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
// #include <dvs_msgs/Event.h>
// #include <dvs_msgs/EventArray.h>
#include <event_camera_codecs/decoder_factory.h>
#include <event_camera_msgs/msg/event_packet.hpp>
#include "event_adapter.hpp"

namespace cmax_slam {

class CMaxSLAM
{
public:
    // Constructor
    // CMaxSLAM(ros::NodeHandle& nh);
    CMaxSLAM(rclcpp::Node::SharedPtr node);
    // Deconstructor
    ~CMaxSLAM();

    // Precomputed bearing vectors for each image pixel
    std::vector<cv::Point3d> precomputed_bearing_vectors; // Share with the back-end
    image_geometry::PinholeCameraModel cam;

private:
    // Node handle used to subscribe to ROS topics
    rclcpp::Node::SharedPtr node_;
    // Private node handle for reading parameters
    rclcpp::Node::SharedPtr pnh_;

    // Subscribers and callbacks
    rclcpp::Subscription<event_camera_msgs::msg::EventPacket>::SharedPtr event_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    void eventsCallback(const event_camera_msgs::msg::EventPacket::SharedPtr msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info);
    bool got_camera_info_;

    // Precompute bearing vectors and share with the front-end and the back-end
    void precomputeBearingVectors();

    // Parameters
    AngVelEstParams front_end_params_;
    PoseGraphParams back_end_params_;

    // Multi-thread
    AngVelEstimator* ang_vel_estimator_;       // Front-end
    PoseGraphOptimizer* pose_graph_optimizer_; // Back-end

    std::thread* pose_graph_optim_; // Thread for the back-end


    EventBatchProcessor batch_processor_;
    rclcpp::Subscription<event_camera_msgs::msg::EventPacket>::SharedPtr eventSub_;
    event_camera_codecs::DecoderFactory<event_camera_msgs::msg::EventPacket, EventBatchProcessor> factory;


};
}
