from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory
# import yaml
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import TimerAction


def generate_launch_description():
    cmax_slam_yaml = os.path.join(
        get_package_share_directory('cmax_slam'),
        'config',
        'cmax_slam_params.yaml')
    # with open(cmax_slam_yaml, 'r') as ymlfile:
    #     cmax_slam_config = yaml.safe_load(ymlfile)


    # bag_play = TimerAction(
    #     period=0.0,
    #     actions=[
    #         ExecuteProcess(
    #             cmd=['ros2', 'bag', 'play', '-r', '0.1', 'events_2025-03-11-17-33-35/'],
    #             output='screen'
    #         )
    #     ]
    # )

    return LaunchDescription([


        ExecuteProcess(
            cmd=[
                "ros2", "topic", "pub", "/event_camera/camera_info", "sensor_msgs/msg/CameraInfo",
                "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, "
                "height: 480, width: 640, distortion_model: \"plumb_bob\", "
                "d: [-0.4037346426737943, 0.3885646319682674, 0.018323193342350153, -0.004115209655268411, -0.526145650187008], "
                "k: [723.9092705728091, 0.0, 308.3813139363713, 0.0, 711.8775025926858, 172.2630515473448, 0.0, 0.0, 1.0], "
                "r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], "
                "p: [723.9092705728091, 0.0, 308.3813139363713, 0.0, 0.0, 711.8775025926858, 172.2630515473448, 0.0, 0.0, 0.0, 1.0, 0.0], "
                "binning_x: 0, binning_y: 0, "
                "roi: {x_offset: 0, y_offset: 0, width: 0, height: 0, do_rectify: false}}",
                "--times", "1" 
                # " --once"
                # "--rate", "1"
            ],
            output="screen"
        ),


        # CMax-SLAM node
        Node(
            package='cmax_slam',
            executable='cmax_slam',
            output='screen',
            arguments=['--v', '1'],
            parameters=[cmax_slam_yaml]
        ),
        


        # ROS2 bag play (modify path to your bag)
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '-r', '0.1', 'events_2025-03-21-16-38-32/'],
            # cmd=['ros2', 'bag', 'play', '-r', '0.1', 'events_2025-03-11-17-33-35/'],
            
            output='screen'
        ),



        # Visualization (optional)
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            arguments=['/pano_map']
        ),
    ]) 