from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from datetime import datetime

def generate_launch_description():

    ld = LaunchDescription()

    now = datetime.now()

    log_path = "/DroneWorkspace/data/"
    log_id = now.isoformat()
    log_test_mode = 1
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
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=["udp4", "-p", "8888", "-v6"]
        )
    )

    return ld