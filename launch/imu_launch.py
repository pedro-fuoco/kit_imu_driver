import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'kit_imu_driver'
    imu_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'imu_params.yaml')

    # O argumento 'debug' é declarado com o valor padrão de `false`
    debug_mode_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable or disable TF publishing for debugging'
    )

    # Esse arquivo launch recebe a configuração de `debug` para determinar se deve executar
    # o node `imu_complementary_filter` no modo de debug.

    # Nesse caso, o modo de debug significa que o proprio node do filtro publicará a matriz de
    # transformação entre o eixo da IMU e o eixo de odometria do robô. Isso permite debug facil
    # através de ferramentas de visualização como rviz2, mas idealmente deve ser responsabilidade 
    # de um node de fusão de sensores, em outro pacote.
    debug = LaunchConfiguration('debug')

    return LaunchDescription([
        debug_mode_arg,
        Node(
            package='kit_imu_driver',
            executable='imu_node',
            name='kit_imu',
            parameters=[imu_params_file],
        ),
        Node(
            package='imu_complementary_filter',
            executable='complementary_filter_node',
            name='kit_imu_filter',
            namespace='kit',
            parameters=[{'publish_tf': debug}]
        )
    ])
