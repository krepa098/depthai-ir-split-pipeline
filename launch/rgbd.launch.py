import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file")
    depthai_prefix = get_package_share_directory("depthai_ros_driver")

    name = LaunchConfiguration('name').perform(context)

    return [
        Node(
            package='image_transport',
            executable='republish',
            name='oak_republish_stereo_compressedDepth',
            namespace='',
            output='screen',
            remappings=[
                ("in", "/oak/stereo/image_raw"),
                ("out/compressedDepth", "/oak/stereo/image_raw/compressedDepth"),
            ],
            arguments=['raw', 'compressedDepth'],
        ), 
        Node(
            package = "oak",
            executable = "oak_node",
            name = "oak",
            parameters = [{'rectify_mono': False}]
        ),
    ]


def generate_launch_description():
    depthai_prefix = get_package_share_directory("oak")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("camera_model", default_value="OAK-D-Pro"),
        DeclareLaunchArgument("parent_frame", default_value="oak-d-base-frame"),
        DeclareLaunchArgument("cam_pos_x", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_y", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_z", default_value="0.0"),
        DeclareLaunchArgument("cam_roll", default_value="0.0"),
        DeclareLaunchArgument("cam_pitch", default_value="0.0"),
        DeclareLaunchArgument("cam_yaw", default_value="0.0"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(depthai_prefix, 'config', 'rgbd.yaml')),
        DeclareLaunchArgument("use_rviz", default_value="False"),
        DeclareLaunchArgument("rectify_rgb", default_value="False"),
        DeclareLaunchArgument("rectify_r", default_value="False"),
        DeclareLaunchArgument("rectify_l", default_value="False"),
        DeclareLaunchArgument("enable_pc", default_value="False"),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
