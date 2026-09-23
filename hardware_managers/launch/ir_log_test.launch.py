from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import ExecuteProcess
from launch_ros.descriptions import ComposableNode

from datetime import datetime

def generate_launch_description():

    ld = LaunchDescription()

    log_path = "/home/hex/data/"

    now = datetime.now()

    #log_id = str(now)
    log_id = "bag_" + now.strftime("%Y_%m_%d_%H_%M_%S")
    log_test_mode = True

    print(log_id)

    namespace = 'hardware'
    ld.add_action(ComposableNodeContainer(
        namespace=namespace,
        name=namespace,
        package='rclcpp_components',
        executable='component_container', # single-threaded callback execution
        composable_node_descriptions=[
            ComposableNode(
                package="logger",
                plugin='logger::Logger',
                namespace=namespace,
                name='logger',
                parameters=[{'log_prefix': log_id, 'log_path': log_path, 'log_test': log_test_mode}],
            ),
            ComposableNode(
                package="ircamera_manager",
                plugin='ircamera_manager::IRCameraManager',
                namespace=namespace,
                name='ircamera_manager',
                parameters=[],
            ),
        ],
        output='screen',
    ))

    # NOTE: v4l2_camera in ROS 2 Humble does not honor frame_rate or publish_rate as
    # ROS parameters. The effective frame rate is set by the V4L2 device itself, and
    # lowering the publishing frequency must be done downstream (for example with a
    # throttle node) instead of using unsupported parameters here.
    ld.add_action(
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            namespace='/hardware',
            name='rgb_camera',
            remappings=[
                ('image_raw', 'rgb_camera/rgb_image'),
                ('image_raw/compressed', 'rgb_camera/rgb_image/compressed'),
                ('image_raw/compressedDepth', 'rgb_camera/rgb_image/compressedDepth'),
                ('image_raw/theora', 'rgb_camera/rgb_image/theora'),
                ('camera_info', 'rgb_camera/camera_info'),
            ],
            parameters=[{
                'video_device': '/dev/video0',
                'image_size': [640, 480],
                'pixel_format': 'yuyv',
                'camera_frame_id': 'camera',
                'camera_name': 'rgb_camera',
                'camera_info_url': '',
                'output_encoding': 'yuv422',
            }],
            output='screen',
        )
    )

    ld.add_action(
        Node(
            package='topic_tools',
            executable='throttle',
            name='rgb_image_throttle',
            arguments=['messages', '/hardware/rgb_camera/rgb_image', '15.0', '/hardware/rgb_camera/rgb_image_throttled'],
            output='screen',
        )
    )

    ld.add_action(
        ExecuteProcess(
        cmd=[[
            'MicroXRCEAgent udp4 --port 8888 -v'
        ]],
        shell=True
    ))

    return ld