import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    ld = LaunchDescription()

    pkg_name = "gimbal_manager"

    param_config = os.path.join(
        get_package_share_directory(pkg_name),
        'config', 'gimbal_params.yaml'
    )

    namespace = 'hardware'
    ld.add_action(ComposableNodeContainer(
        namespace=namespace,
        name=namespace+'_gimbal_manager',
        package='rclcpp_components',
        executable='component_container', # single-threaded callback execution
        composable_node_descriptions=[
            ComposableNode(
                package=pkg_name,
                plugin='gimbal_manager::GimbalManager',
                namespace=namespace,
                name='gimbal_manager',
                parameters=[param_config],
            ),
        ],
        output='screen',
    ))

    return ld