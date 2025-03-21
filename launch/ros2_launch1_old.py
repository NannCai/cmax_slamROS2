from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # CMax-SLAM node
        Node(
            package='cmax_slam',
            executable='cmax_slam',
            output='screen',
            parameters=[{
                'events_topic': '/event_camera/events',
                'camera_info_topic': '/event_camera/camera_info',
                'contrast_measure': 0,
                'frontend_blur_sigma': 1.0,
                # Add other parameters from ijrr.launch here
            }]
        ),
        
        # ROS2 bag play (modify path to your bag)
        ExecuteProcess(
            # cmd=['ros2', 'bag', 'play', '/path/to/your/bag',
            #      '--remap', '/event_camera/events:=/event_camera/events', 
            #      '/event_camera/camera_info:=/event_camera/camera_info'],
            cmd=['ros2', 'bag', 'play', 'events_2025-03-06-15-26-47/', '-r', '0.1'],
            output='screen'
        ),

        ExecuteProcess(
            cmd=[
                "ros2", "topic", "pub", "/event_camera/camera_info", "sensor_msgs/CameraInfo",
                "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},"
                "height: 480, width: 640, distortion_model: 'plumb_bob',"
                "D: [-0.3796, 0.1789, -0.0073, 0.0016, 0.0],"
                "K: [588.1, 0, 339.8, 0, 0, 593.9, 242.4, 0, 0, 1],"
                "R: [1, 0, 0, 0, 1, 0, 0, 0, 1],"
                "P: [588.1, 0, 339.8, 0, 0, 593.9, 242.4, 0, 0, 0, 1, 0],"
                "binning_x: 0, binning_y: 0,"
                "roi: {x_offset: 0, y_offset: 0, height: 0, width: 0, do_rectify: false}}",
                "--once"
            ],
            output="screen"
        ),


        
        # Node(
        #     package="ros2topic",
        #     executable="ros2topic",
        #     name="dvs_cam_info_pub",
        #     arguments=[
        #         "pub",
        #         "/event_camera/camera_info",
        #         "sensor_msgs/CameraInfo",
        #         "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},"
        #         "height: 480, width: 640, distortion_model: 'plumb_bob',"
        #         "D: [-0.3796, 0.1789, -0.0073, 0.0016, 0.0],"
        #         "K: [588.1, 0, 339.8, 0, 593.9, 242.4, 0, 0, 1],"
        #         "R: [1, 0, 0, 0, 1, 0, 0, 0, 1],"
        #         "P: [588.1, 0, 339.8, 0, 0, 593.9, 242.4, 0, 0, 0, 1, 0],"
        #         "binning_x: 0, binning_y: 0,"
        #         "roi: {x_offset: 0, y_offset: 0, height: 0, width: 0, do_rectify: false}}",
        #         "--once",
        #     ],
        # ),

        

        # Node(
        #     package='ros2cli',
        #     executable='ros2',
        #     arguments=[
        #         'topic', 'pub', '--once',
        #         '/event_camera/camera_info',
        #         'sensor_msgs/msg/CameraInfo',
        #         '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, '
        #         'height: 480, width: 640, distortion_model: "plumb_bob", '
        #         'd: [-0.37962772883714485, 0.17892560312683234, -0.007389826827862441, 0.0016114593719536608, 0.0], '
        #         'k: [588.0999081785662, 0.0, 339.8258855424323, 0.0, 593.9887179137335, 242.42516485069098, 0.0, 0.0, 1.0], '
        #         'r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], '
        #         'p: [588.0999081785662, 0.0, 339.8258855424323, 0.0, 0.0, 593.9887179137335, 242.42516485069098, 0.0, 0.0, 0.0, 1.0, 0.0], '
        #         'binning_x: 0, binning_y: 0, '
        #         'roi: {x_offset: 0, y_offset: 0, height: 0, width: 0, do_rectify: false}}'
        #     ],
        #     output='screen'
        # ),


        # Visualization (optional)
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            arguments=['/pano_map']
        ),
    ]) 