from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, ExecuteProcess
from launch_ros.descriptions import ComposableNode

from datetime import datetime

def generate_launch_description():

    ld = LaunchDescription()

    now = datetime.now()

    log_path = "/home/hex/data/"
    log_id = now.isoformat()
    log_test_mode = False
    print(log_id)

    namespace = 'hardware'
    ld.add_action(ComposableNodeContainer(
        namespace=namespace,
        name=namespace,
        package='rclcpp_components',
        executable='component_container', # multi-threaded callback execution
        composable_node_descriptions=[
            ComposableNode(
                package="streamer",
                plugin='streamer::Streamer',
                namespace=namespace,
                name='streamer',
                parameters=[],
            ),
            ComposableNode(
                package="ircamera_manager",
                plugin='ircamera_manager::IRCameraManager',
                namespace=namespace,
                name='ircamera_manager',
                parameters=[],
            ),
            ComposableNode(
                package='logger',
                plugin='logger::Logger',
                namespace=namespace,
                name='logger',
                parameters=[{'log_prefix': log_id, 'log_path': log_path, 'log_test': log_test_mode}],
            ),
        ],
        output='screen',
    ))

    ld.add_action(
        ExecuteProcess(
        cmd=[[
            'MicroXRCEAgent udp4 --port 8888 -v'
        ]],
        shell=True
    ))

    return ld