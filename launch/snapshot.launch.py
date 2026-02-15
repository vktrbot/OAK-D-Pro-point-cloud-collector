from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="oak_cloud_collector",
            executable="snapshot",
            name="oak_cloud_collector",
            output="screen",
            parameters=[{
                "topic": "/oak/points",
                "out_dir": "~/Desktop/OAKD/clouds",
                "max_points": 200000,
                "format": "ply",
                "binary": True,
                "voxel_size": 0.0,
            }],
        )
    ])
