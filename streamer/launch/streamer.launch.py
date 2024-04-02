from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    ld = LaunchDescription()

    pkg_name = "streamer"

    namespace = 'hardware'
    ld.add_action(ComposableNodeContainer(
        namespace=namespace,
        name=namespace+'_streamer',
        package='rclcpp_components',
        executable='component_container', # single-threaded callback execution
        composable_node_descriptions=[
            ComposableNode(
                package=pkg_name,
                plugin='streamer::Streamer',
                namespace=namespace,
                name='streamer',
                parameters=[],
            ),
        ],
        output='screen',
    ))

    return ld