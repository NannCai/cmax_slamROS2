#include "cmax_slam.h"
#include <glog/logging.h>
#include <camera_info_manager/camera_info_manager.hpp>
#include <iostream>

#include <string>
#include <sstream>
#include <filesystem>
#include <ctime>

namespace cmax_slam {

static const double rad2degFactor = 180.0 * M_1_PI;


CMaxSLAM::CMaxSLAM(rclcpp::Node::SharedPtr node)
    : node_(node)
    // , node_(node->create_sub_node(""))



{
    // Load test configurations
    // Topic info
    // const std::string events_topic = node_.param<std::string>("events_topic", "/event_camera/events");
    // const std::string camera_info_topic = node_.param<std::string>("camera_info_topic", "/event_camera/camera_info");

    const std::string events_topic = node_->declare_parameter<std::string>("events_topic", "/event_camera/events");
    const std::string camera_info_topic = node_->declare_parameter<std::string>("camera_info_topic", "/event_camera/camera_info");  // TODO


    LOG(INFO) << "Event topic: " << events_topic;
    LOG(INFO) << "Camera info topic: " << camera_info_topic;

    // Set up subscribers
    // event_sub_ = nh_.subscribe(events_topic, 0, &CMaxSLAM::eventsCallback, this);
    const int qsize = 1000;
    auto qos = rclcpp::QoS(rclcpp::KeepLast(qsize))
                .best_effort()
                .durability_volatile();
    event_sub_ = node_->create_subscription<event_camera_msgs::msg::EventPacket>(
      "/event_camera/events", qos,
      std::bind(
        &CMaxSLAM::eventsCallback, this, std::placeholders::_1));

    // camera_info_sub_ = nh_.subscribe(camera_info_topic, 0, &CMaxSLAM::cameraInfoCallback, this);
    camera_info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, 10,
        std::bind(&CMaxSLAM::cameraInfoCallback, this, std::placeholders::_1));
    got_camera_info_ = false;

    // Load prcessing options
    OptionsProcess process_opt;
    process_opt.contrast_measure = node_->declare_parameter<int>("contrast_measure", 0);

    LOG(INFO) << "*************** Processing options ******************";
    LOG(INFO) << "Contrast Measure: " << ((process_opt.contrast_measure == 0)? "Variance": "Mean Square");

    // Load front-end configurations
    front_end_params_.process_opt = process_opt;
    front_end_params_.num_events_per_packet = node_->declare_parameter<int>("num_events_per_packet", 30000);
    front_end_params_.dt_ang_vel = node_->declare_parameter<double>("dt_ang_vel", 0.02);
    front_end_params_.warp_opt.blur_sigma = node_->declare_parameter<double>("frontend_blur_sigma", 1.0);
    front_end_params_.warp_opt.event_batch_size = node_->declare_parameter<int>("event_batch_size", 100);
    front_end_params_.warp_opt.event_sample_rate = node_->declare_parameter<int>("frontend_event_sample_rate", 1);
    front_end_params_.data_opt.show_iwe = node_->declare_parameter<bool>("show_local_iwe", false);

    LOG(INFO) << "*************** Front-end params ******************";
    LOG(INFO) << "frontend_blur_sigma: " << front_end_params_.warp_opt.blur_sigma;
    LOG(INFO) << "event_batch_size: " << front_end_params_.warp_opt.event_batch_size;
    LOG(INFO) << "frontend_event_sample_rate: " << front_end_params_.warp_opt.event_sample_rate;

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

    LOG(INFO) << "*************** Back-end params ******************";
    LOG(INFO) << "time_window_size: " << back_end_params_.sliding_window_opt.time_window_size;
    LOG(INFO) << "sliding_window_stride: " << back_end_params_.sliding_window_opt.sliding_window_stride;
    LOG(INFO) << "backend_blur_sigma: " << back_end_params_.warp_opt.blur_sigma;
    LOG(INFO) << "event_batch_size: " << back_end_params_.warp_opt.event_batch_size;
    LOG(INFO) << "backend_event_sample_rate: " << back_end_params_.warp_opt.event_sample_rate;
    LOG(INFO) << "dt_knots: " << back_end_params_.traj_opt.dt_knots;
    LOG(INFO) << "spline_degree: " << back_end_params_.traj_opt.spline_degree;
    LOG(INFO) << "show_pano_map: " << back_end_params_.data_opt.show_iwe;
    LOG(INFO) << "pano_height: " << back_end_params_.map_opt.pano_height;
    LOG(INFO) << "Y-angle = " << back_end_params_.map_opt.Y_angle;
    LOG(INFO) << "draw_FOV = " << back_end_params_.draw_FOV;
    LOG(INFO) << "gamma = " << back_end_params_.gamma;

    // New a angular velocity estimator (the front-end runs in the main thread)
    ang_vel_estimator_ = new AngVelEstimator(node);

    // New a pose graph optimizer
    pose_graph_optimizer_ = new PoseGraphOptimizer(node);

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
        RCLCPP_INFO(node_->get_logger(), "Intialize the back-end");

    }
}

void CMaxSLAM::eventsCallback(const event_camera_msgs::msg::EventPacket::SharedPtr msg)
{
    if(!got_camera_info_)
    {
        RCLCPP_ERROR(node_->get_logger(), "Received events but camera info is still missing");
        return;
    }


    auto decoder = factory.getInstance(*msg);
    if (!decoder) return;
    decoder->setTimeMultiplier(1);
    decoder->setTimeBase(msg->time_base);
    batch_processor_.setHasSensorTimeSinceEpoch(decoder->hasSensorTimeSinceEpoch());


    decoder->decode(*msg, &batch_processor_);
    std::vector<DvsEvent> event_subset = batch_processor_.events();
    batch_processor_.clear();
    
    // Print size and first 5 events
    RCLCPP_INFO(node_->get_logger(), "Event subset size: %zu", event_subset.size());
    for (size_t i = 0; i < std::min(event_subset.size(), size_t(5)); ++i) {
        const auto& ev = event_subset[i];
        // RCLCPP_INFO(node_->get_logger(), "Event %zu: t=%f, x=%d, y=%d, p=%d", 
        //            i, ev.ts, ev.x, ev.y, ev.p);

        std::cout << "eventCD---ts(s): " << ev.ts/ 1e9 << ", x: " << ev.x << ", y: " << ev.y << ", polarity: " << static_cast<int>(ev.polarity) << std::endl;


    }


    // for (auto ev = msg->events.begin(); ev < msg->events.end();
    //      ev += front_end_params_.warp_opt.event_sample_rate)
    // {
    //     // Push events into the frontend, which will pass them to the backend then.
    //     ang_vel_estimator_->pushEvent(*ev);
    // }

    for (auto ev = event_subset.begin(); ev < event_subset.end();  // Changed from msg->events to event_subset
         ev += front_end_params_.warp_opt.event_sample_rate)
    {
        // std::cout << "Event " <<  ": t=" << ev->ts << ", x=" << ev->x 
        //     << ", y=" << ev->y << ", p=" << ev->polarity << std::endl;
        // // Push events into the frontend, which will pass them to the backend then.
        ang_vel_estimator_->pushEvent(*ev);  // Now using decoded events from event_subset
        // std::cout<< "finish pushEvent"<< std::endl;
    }

}

}
