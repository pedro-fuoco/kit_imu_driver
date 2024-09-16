import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'kit_imu_driver'
    imu_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'imu_params.yaml')
    
    return LaunchDescription([
        Node(
            package='kit_imu_driver',
            executable='imu_node',
            name='kit_imu',
            parameters=[imu_params_file],
        )
    ])