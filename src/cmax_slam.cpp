#include "cmax_slam.h"
#include <rclcpp/rclcpp.hpp>
#include <event_camera_codecs/decoder.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <string>
#include <sstream>
#include <filesystem>
#include <ctime>

namespace cmax_slam {

static const double rad2degFactor = 180.0 * M_1_PI;

CMaxSLAM::CMaxSLAM(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    // Load test configurations
    // Topic info
    const std::string events_topic = node_->declare_parameter<std::string>("events_topic", "/dvs/events");
    const std::string camera_info_topic = node_->declare_parameter<std::string>("camera_info_topic", "/dvs/camera_info");
    RCLCPP_INFO(node_->get_logger(), "Event topic: %s", events_topic.c_str());
    RCLCPP_INFO(node_->get_logger(), "Camera info topic: %s", camera_info_topic.c_str());

    // Set up subscribers
    event_sub_ = node_->create_subscription<event_camera_codecs::EventPacket>(
        events_topic, 10, std::bind(&CMaxSLAM::eventsCallback, this, std::placeholders::_1));
    camera_info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, 10, std::bind(&CMaxSLAM::cameraInfoCallback, this, std::placeholders::_1));
    got_camera_info_ = false;

    // Load processing options
    OptionsProcess process_opt;
    process_opt.contrast_measure = node_->declare_parameter<int>("contrast_measure", 0);

    RCLCPP_INFO(node_->get_logger(), "*************** Processing options ******************");
    RCLCPP_INFO(node_->get_logger(), "Contrast Measure: " << ((process_opt.contrast_measure == 0)? "Variance": "Mean Square"));

    // Load front-end configurations
    front_end_params_.process_opt = process_opt;
    front_end_params_.num_events_per_packet = node_->declare_parameter<int>("num_events_per_packet", 30000);
    front_end_params_.dt_ang_vel = node_->declare_parameter<double>("dt_ang_vel", 0.02);
    front_end_params_.warp_opt.blur_sigma = node_->declare_parameter<double>("frontend_blur_sigma", 1.0);
    front_end_params_.warp_opt.event_batch_size = node_->declare_parameter<int>("event_batch_size", 100);
    front_end_params_.warp_opt.event_sample_rate = node_->declare_parameter<int>("frontend_event_sample_rate", 1);
    front_end_params_.data_opt.show_iwe = node_->declare_parameter<bool>("show_local_iwe", false);

    RCLCPP_INFO(node_->get_logger(), "*************** Front-end params ******************");
    RCLCPP_INFO(node_->get_logger(), "frontend_blur_sigma: " << front_end_params_.warp_opt.blur_sigma);
    RCLCPP_INFO(node_->get_logger(), "event_batch_size: " << front_end_params_.warp_opt.event_batch_size);
    RCLCPP_INFO(node_->get_logger(), "frontend_event_sample_rate: " << front_end_params_.warp_opt.event_sample_rate);

    // Load back-end configurations
    back_end_params_.process_opt = process_opt;
    back_end_params_.sliding_window_opt.time_window_size = node_->declare_parameter<double>("backend_time_window_size", 0.2);
    back_end_params_.sliding_window_opt.sliding_window_stride = node_->declare_parameter<double>("backend_sliding_window_stride", 0.1);
    back_end_params_.warp_opt.blur_sigma = node_->declare_parameter<double>("backend_blur_sigma", 1.0);
    back_end_params_.warp_opt.event_batch_size = front_end_params_.warp_opt.event_batch_size;
    back_end_params_.warp_opt.event_sample_rate = node_->declare_parameter<int>("backend_event_sample_rate", 1);
    back_end_params_.traj_opt.dt_knots = node_->declare_parameter<double>("dt_knots", 0.1);
    back_end_params_.traj_opt.spline_degree = node_->declare_parameter<int>("spline_degree", 1);
    back_end_params_.data_opt.show_iwe = node_->declare_parameter<bool>("show_pano_map", true);
    back_end_params_.map_opt.pano_height = node_->declare_parameter<int>("pano_height", 1024);
    back_end_params_.map_opt.pano_width = 2 * back_end_params_.map_opt.pano_height;
    back_end_params_.map_opt.Y_angle = node_->declare_parameter<double>("Y_angle", 0.0);
    back_end_params_.map_opt.backend_min_ev_rate = node_->declare_parameter<int>("backend_min_ev_rate", 10);
    back_end_params_.map_opt.max_update_times = node_->declare_parameter<int>("max_update_times", 10);
    back_end_params_.draw_FOV = node_->declare_parameter<bool>("draw_FOV", false);
    back_end_params_.gamma = node_->declare_parameter<double>("gamma", 0.75);

    RCLCPP_INFO(node_->get_logger(), "*************** Back-end params ******************");
    RCLCPP_INFO(node_->get_logger(), "time_window_size: " << back_end_params_.sliding_window_opt.time_window_size);
    RCLCPP_INFO(node_->get_logger(), "sliding_window_stride: " << back_end_params_.sliding_window_opt.sliding_window_stride);
    RCLCPP_INFO(node_->get_logger(), "backend_blur_sigma: " << back_end_params_.warp_opt.blur_sigma);
    RCLCPP_INFO(node_->get_logger(), "event_batch_size: " << back_end_params_.warp_opt.event_batch_size);
    RCLCPP_INFO(node_->get_logger(), "backend_event_sample_rate: " << back_end_params_.warp_opt.event_sample_rate);
    RCLCPP_INFO(node_->get_logger(), "dt_knots: " << back_end_params_.traj_opt.dt_knots);
    RCLCPP_INFO(node_->get_logger(), "spline_degree: " << back_end_params_.traj_opt.spline_degree);
    RCLCPP_INFO(node_->get_logger(), "show_pano_map: " << back_end_params_.data_opt.show_iwe);
    RCLCPP_INFO(node_->get_logger(), "pano_height: " << back_end_params_.map_opt.pano_height);
    RCLCPP_INFO(node_->get_logger(), "Y-angle = " << back_end_params_.map_opt.Y_angle);
    RCLCPP_INFO(node_->get_logger(), "draw_FOV = " << back_end_params_.draw_FOV);
    RCLCPP_INFO(node_->get_logger(), "gamma = " << back_end_params_.gamma);

    // New a angular velocity estimator (the front-end runs in the main thread)
    ang_vel_estimator_ = new AngVelEstimator(node_.get());

    // New a pose graph optimizer
    pose_graph_optimizer_ = new PoseGraphOptimizer(node_.get());

    // Initialize the back-end thread and launch
    pose_graph_optim_ = new std::thread(&PoseGraphOptimizer::Run, pose_graph_optimizer_);

    // Set the pointer to each other
    ang_vel_estimator_->setBackend(pose_graph_optimizer_);
    pose_graph_optimizer_->setFrontend(ang_vel_estimator_);
}

CMaxSLAM::~CMaxSLAM()
{
    pose_graph_optim_->detach();
    delete ang_vel_estimator_;
    delete pose_graph_optimizer_;
}

void CMaxSLAM::precomputeBearingVectors()
{
    int sensor_width = cam.fullResolution().width;
    int sensor_height = cam.fullResolution().height;

    for(int y=0; y < sensor_height; y++)
    {
        for(int x=0; x < sensor_width; x++)
        {
            cv::Point2d rectified_point = cam.rectifyPoint(cv::Point2d(x,y));
            cv::Point3d bearing_vec = cam.projectPixelTo3dRay(rectified_point);
            precomputed_bearing_vectors.emplace_back(bearing_vec);
        }
    }
}

void CMaxSLAM::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info)
{
    if(!got_camera_info_)
    {
        RCLCPP_INFO(node_->get_logger(), "Loading camera information");
        cam.fromCameraInfo(camera_info);
        got_camera_info_ = true;

        RCLCPP_INFO(node_->get_logger(), "Camera info got");
        camera_info_sub_.reset(); // no need to listen to this topic any more

        // Initialze the front-end
        precomputeBearingVectors();
        ang_vel_estimator_->initialize(&cam, front_end_params_, precomputed_bearing_vectors);

        // Intialize the back-end
        int camera_width = cam.fullResolution().width;
        int camera_height = cam.fullResolution().height;
        pose_graph_optimizer_->initialize(camera_width, camera_height,
                                          back_end_params_,
                                          &(ang_vel_estimator_->events_),
                                          &precomputed_bearing_vectors);
    }
}

void CMaxSLAM::eventsCallback(const event_camera_codecs::EventPacket::SharedPtr msg)
{
    if(!got_camera_info_)
    {
        RCLCPP_ERROR(node_->get_logger(), "Received events but camera info is still missing");
        return;
    }

    // Decode the event packet
    event_camera_codecs::Decoder decoder;
    auto events = decoder.decode(msg);

    for (auto ev = events.begin(); ev < events.end();
         ev += front_end_params_.warp_opt.event_sample_rate)
    {
        // Push events into the frontend, which will pass them to the backend then.
        ang_vel_estimator_->pushEvent(*ev);
    }
}

}
