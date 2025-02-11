#pragma once

#include "backend/pose_graph_optimizer.h"
#include "utils/parameters.h"

#include <opencv2/core.hpp>
#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <event_camera_codecs/Event.h>
#include <event_camera_codecs/EventArray.h>

#include <deque>
#include <vector>
#include <mutex>
#include <fstream>

namespace cmax_slam {

class PoseGraphOptimizer;

class AngVelEstimator {
public:
    // Constructor
    AngVelEstimator(rclcpp::Node::SharedPtr nh);

    // Deconstructor
    ~AngVelEstimator();

    // Initialization
    void initialize(image_geometry::PinholeCameraModel* cam,
                    const AngVelEstParams& params,
                    const std::vector<cv::Point3d>& precomputed_bearing_vectors);

    // Set the pointer to the back-end
    void setBackend(PoseGraphOptimizer* ptr) { pose_graph_optimizer_ = ptr; }

    // Push the event to the frontend
    void pushEvent(const event_camera_codecs::Event& event);

    // Compute IWE for CMax optimization (with Gaussian blurring)
    void computeImageOfWarpedEvents(
            const cv::Point3d& ang_vel,
            cv::Mat* image_warped,
            cv::Mat* image_warped_deriv);

    // Compute IWE for display OptionsWarp (without Gaussian blurring, for visualization)
    void computeImageOfWarpedEvents(
            const cv::Point3d& ang_vel,
            cv::Mat* image_warped);

    // Parameters
    AngVelEstParams params;

    // The vector to save all events that are going to be processed
    std::vector<event_camera_codecs::Event> events_;

    // Delete old (processed) events (called by the backend)
    void deleteOldEvents(const int idx_backend);

private:
    // Pointer to the ROS node, for publishing
    rclcpp::Node::SharedPtr nh_;

    // Publishers
    image_transport::ImageTransport it_;
    image_transport::Publisher img_pub_;
    cv_bridge::CvImage cv_event_image_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ang_vel_pub_;

    // Publish results
    void publishAngularVelocity();
    void publishEventImage();

    // Get the events in the current time window
    void getEventSubset();
    void slideWindow();

    // Processing. Motion estimation
    void processEventPacket();

    // Slide window
//     std::vector<event_camera_codecs::Event> event_subset_; 
    std::vector<event_camera_codecs::EventPacket> event_subset_;        // Event subset for the current window
    std::deque<std::pair<int, int>> event_subsets_info_;
    int num_event_total_; // Current total event number stored in the frontend
    int ev_beg_idx_, ev_end_idx_;
    bool sliding_window_initialized_; // Set sliding window when
    rclcpp::Duration dt_av_; // Frequency of output angular velocity
    int num_ev_half_packet_; // num_events_per_packet/2

    // States of the front-end estimator
    rclcpp::Time time_packet_; // Timestamp of the current event packet (angular velocity)
    rclcpp::Time time_get_subset_; // Timestamp of getting the next subset
    cv::Point3d ang_vel_; // Angular velocity

    // Function to warp a small event batch
    void warpAndAccumulateEvents(
            const cv::Point3d& ang_vel,
            const int& event_batch_begin_idx,
            const int& event_batch_end_idx,
            const rclcpp::Time time_ref,
            cv::Mat* image_warped,
            cv::Mat* image_warped_deriv);

    // Optimization
    double setupProblemAndOptimize_ceres(cv::Point3d& ang_vel);
    double setupProblemAndOptimize_gsl(cv::Point3d& ang_vel);

    // Camera information (size, intrinsics, lens distortion)
    int cam_width_, cam_height_;
    cv::Matx33d camera_matrix_;

    // Precomputed bearing vectors for each image pixel
    std::vector<cv::Point3d> precomputed_bearing_vectors_;

    // Pointer to the pose graph solver (back-end)
    PoseGraphOptimizer* pose_graph_optimizer_;
};

} // namespace
