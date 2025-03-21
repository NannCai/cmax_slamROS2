from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # CMax-SLAM node
        Node(
            package="cmax_slam",
            executable="cmax_slam",
            name="cmax_slam",
            output="screen",
            parameters=[
                {"events_topic": "/event_camera/events"},
                {"camera_info_topic": "/event_camera/camera_info"},
                {"contrast_measure": 0},
                {"frontend_blur_sigma": 1.0},
                {"backend_blur_sigma": 1.0},
                {"event_batch_size": 100},
                {"frontend_event_sample_rate": 1},
                {"backend_event_sample_rate": 1},
                {"num_events_per_packet": 200000},
                {"dt_ang_vel": 0.01},
                {"show_local_iwe": True},
                {"backend_time_window_size": 0.2},
                {"backend_sliding_window_stride": 0.2},
                {"spline_degree": 1},
                {"dt_knots": 0.05},
                {"pano_height": 2048},
                {"backend_min_ev_rate": 10000},
                {"max_update_times": 200},
                {"Y_angle": 0.0},
                {"gamma": 0.75},
                {"show_pano_map": True},
                {"draw_FOV": True},
            ],
        ),

        # Publish camera calibration
        Node(
            package="ros2topic",
            executable="ros2topic",
            name="dvs_cam_info_pub",
            arguments=[
                "pub",
                "/event_camera/camera_info",
                "sensor_msgs/CameraInfo",
                "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},"
                "height: 480, width: 640, distortion_model: 'plumb_bob',"
                "D: [-0.3796, 0.1789, -0.0073, 0.0016, 0.0],"
                "K: [588.1, 0, 339.8, 0, 593.9, 242.4, 0, 0, 1],"
                "R: [1, 0, 0, 0, 1, 0, 0, 0, 1],"
                "P: [588.1, 0, 339.8, 0, 0, 593.9, 242.4, 0, 0, 0, 1, 0],"
                "binning_x: 0, binning_y: 0,"
                "roi: {x_offset: 0, y_offset: 0, height: 0, width: 0, do_rectify: false}}",
                "--once",
            ],
        ),

        # Play rosbag
        Node(
            package="rosbag2",
            executable="play",
            name="player",
            arguments=["--rate", "1.0", "/home/shuang/datasets/ECRot_dataset/river/events.bag"],
        ),

        # Visualize local motion-compensated images
        Node(
            package="image_view",
            executable="image_view",
            name="local_iwe_view",
            remappings=[("image", "local_iwe")],
            parameters=[{"autosize": False}],
        ),

        # Display the panoramic map
        Node(
            package="rqt_image_view",
            executable="rqt_image_view",
            name="image_view",
        ),
    ])
