from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('late_fusion_for_yolos_cpp')
    params = os.path.join(pkg_share, 'config', 'late_fusion_params.yaml')

    return LaunchDescription([
        Node(
            package='late_fusion_for_yolos_cpp',
            executable='late_fusion_node',
            name='late_fusion_node',
            output='screen',
            parameters=[params]
        )
    ])
