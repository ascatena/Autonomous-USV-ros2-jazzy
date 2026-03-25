import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # CAMBIO IMPORTANTE: Obtener la ruta instalada ("share"), no la de "src"
    pkg_robot_config = get_package_share_directory('robot_config')

    # 1. EKF Local
    ekf_local_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_config, 'launch', 'ekf_local.launch.py')
        )
    )

    # 2. EKF Global
    ekf_global_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_config, 'launch', 'ekf.launch.py')
        )
    )

    # 3. Navsat Transform
    navsat_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_config, 'launch', 'navsat.launch.py')
        )
    )

    return LaunchDescription([
        navsat_launch,
        ekf_local_launch,
        ekf_global_launch
    ])