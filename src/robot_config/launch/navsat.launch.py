from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Ruta a archivo de configuración .yaml
    config_file = '/home/cdr/ros2_ws/src/robot_config/config/navsat.yaml'

    return LaunchDescription([
        # ---------------------------------------------------------
        # 1. TRANSFORMACIONES ESTÁTICAS (TF)
        # ---------------------------------------------------------
        # Define dónde está la IMU respecto al centro del barco (base_link)
        # Formato: x y z yaw pitch roll parent_frame child_frame        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = [
                '--x', '0', '--y', '0', '--z', '0.1',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'imu_frame'
            ]
        ),

        # Define dónde está el GPS respecto al centro del barco        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = [
                '--x', '0.2', '--y', '0', '--z', '0.15',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'gps_frame'
            ]
        ),

        # ---------------------------------------------------------
        # 2. NAVSAT TRANSFORM NODE (GPS -> UTM)
        # ---------------------------------------------------------
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[config_file],
            # remappings=[
            #     # Entradas del nodo
            #     ('imu/data', '/imu/data'),
            #     ('gps/fix', '/gps/fix'),
                
            #     # Odometría filtrada (feedback).                
            #     ('odometry/filtered', 'odometry/global') 
            # ]
        )
    ])