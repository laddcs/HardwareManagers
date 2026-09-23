from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_name",
                default_value="rgb_camera",
                description="Name prefix used for the published image and camera_info topics.",
            ),
            DeclareLaunchArgument(
                "camera_namespace",
                default_value="/sensing/camera",
                description="Namespace to launch the camera under.",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="rgb_image",
                description="Topic name for the image stream.",
            ),
            DeclareLaunchArgument(
                "video_device",
                default_value="/dev/video0",
                description="Linux V4L2 device path for the camera.",
            ),
            DeclareLaunchArgument(
                "camera_frame_id",
                default_value="camera",
                description="TF frame id written into the image header.",
            ),
            DeclareLaunchArgument(
                "image_width",
                default_value="640",
                description="Image width in pixels.",
            ),
            DeclareLaunchArgument(
                "image_height",
                default_value="480",
                description="Image height in pixels.",
            ),
            DeclareLaunchArgument(
                "frame_rate",
                default_value="30",
                description="Capture frame rate.",
            ),
            DeclareLaunchArgument(
                "pixel_format",
                default_value="yuyv2rgb",
                description="V4L2 pixel format conversion mode used by the driver.",
            ),
            DeclareLaunchArgument(
                "camera_info_url",
                default_value="",
                description="Optional camera calibration YAML URL.",
            ),
            DeclareLaunchArgument(
                "publish_rate",
                default_value="-1.0",
                description="Publish rate cap in Hz; -1 disables throttling.",
            ),
            DeclareLaunchArgument(
                "use_sensor_data_qos",
                default_value="False",
                description="Use sensor-data QoS for the image stream.",
            ),
            DeclareLaunchArgument(
                "use_v4l2_buffer_timestamps",
                default_value="True",
                description="Use V4L2 buffer timestamps for image headers.",
            ),
            DeclareLaunchArgument(
                "use_image_transport",
                default_value="True",
                description="Enable image_transport for the image stream.",
            ),
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
                namespace=LaunchConfiguration("camera_namespace"),
                name=LaunchConfiguration("camera_name"),
                remappings=[
                    ("image_raw", [LaunchConfiguration("camera_name"), "/", LaunchConfiguration("image_topic")]),
                    (
                        "image_raw/compressed",
                        [
                            LaunchConfiguration("camera_name"),
                            "/",
                            LaunchConfiguration("image_topic"),
                            "/compressed",
                        ],
                    ),
                    (
                        "image_raw/compressedDepth",
                        [
                            LaunchConfiguration("camera_name"),
                            "/",
                            LaunchConfiguration("image_topic"),
                            "/compressedDepth",
                        ],
                    ),
                    (
                        "image_raw/theora",
                        [
                            LaunchConfiguration("camera_name"),
                            "/",
                            LaunchConfiguration("image_topic"),
                            "/theora",
                        ],
                    ),
                    ("camera_info", [LaunchConfiguration("camera_name"), "/camera_info"]),
                ],
                parameters=[
                    {
                        "video_device": LaunchConfiguration("video_device"),
                        "image_width": LaunchConfiguration("image_width"),
                        "image_height": LaunchConfiguration("image_height"),
                        "frame_rate": LaunchConfiguration("frame_rate"),
                        "pixel_format": LaunchConfiguration("pixel_format"),
                        "camera_frame_id": LaunchConfiguration("camera_frame_id"),
                        "camera_name": LaunchConfiguration("camera_name"),
                        "camera_info_url": LaunchConfiguration("camera_info_url"),
                        "publish_rate": LaunchConfiguration("publish_rate"),
                        "use_sensor_data_qos": LaunchConfiguration("use_sensor_data_qos"),
                        "use_v4l2_buffer_timestamps": LaunchConfiguration("use_v4l2_buffer_timestamps"),
                        "use_image_transport": LaunchConfiguration("use_image_transport"),
                    }
                ],
                output="screen",
            ),
        ]
    )