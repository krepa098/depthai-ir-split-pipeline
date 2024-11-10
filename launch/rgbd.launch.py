from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    return [
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
                {'.stereo.image_raw.compressedDepth.png_level': 3},
                {'.stereo.image_raw.compressedDepth.format': 'rvl'},
            ]
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [OpaqueFunction(function=launch_setup)]
    )
