from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
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
            package = "depthai-ir-split-pipeline",
            executable = "oak_node",
            name = "oak",
            output='screen',
            parameters = [
                {'fps': 24.0},
                {'pool_size': 12},
                {'encoder_quality': 90},
                {'floodlight_intensity': 0.2},
                {'dot_intensity': 0.4},
            ]
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [OpaqueFunction(function=launch_setup)]
    )
