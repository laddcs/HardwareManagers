from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    ld = LaunchDescription()

    log_path = "/DroneWorkspace/data/"
    log_id = "test_log7"

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
                parameters=[{'log_prefix': log_id, 'log_path': log_path}],
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

    return ld