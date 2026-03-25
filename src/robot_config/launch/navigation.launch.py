import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile

def generate_launch_description():
    # 1. CONFIGURACIÓN BÁSICA
    package_name = 'robot_config'
    
    # Obtener directorios instalados
    pkg_share = get_package_share_directory(package_name)
    
    # Rutas a los archivos
    params_file_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    
    # --- LA CORRECCIÓN MÁGICA ---
    # Calculamos la ruta absoluta del XML aquí mismo con Python
    bt_xml_path = os.path.join(pkg_share, 'config', 'usv_nav.xml')
    
    # Argumentos
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file', default_value=params_file_path,
        description='Full path to the ROS2 parameters file to use')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    # Nodos a gestionar
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother'
    ]

    # Cargar parámetros del YAML base
    configured_params = ParameterFile(LaunchConfiguration('params_file'), allow_substs=True)

    load_nodes = GroupAction(
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', 'info']),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', 'info']),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', 'info']),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', 'info']),
            
            # --- NODO BT NAVIGATOR CORREGIDO ---
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                # AQUÍ ESTÁ EL TRUCO:
                # Pasamos 'configured_params' (el YAML)
                # Y LUEGO sobrescribimos las rutas del XML con la variable de Python
                parameters=[configured_params, 
                            {'default_nav_to_pose_bt_xml': bt_xml_path},
                            {'default_nav_through_poses_bt_xml': bt_xml_path}
                           ],
                arguments=['--ros-args', '--log-level', 'info']),
            
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', 'info']),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', 'info']),
            
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]),
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(load_nodes)

    return ld