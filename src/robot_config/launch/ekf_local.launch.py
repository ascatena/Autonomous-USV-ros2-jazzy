from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_local',
            output='screen',
            parameters=[ 
                '/home/cdr/ros2_ws/src/robot_config/config/ekf_local.yaml'
            ],
            remappings=[
                ('/odometry/filtered', '/odometry/filtered/local'),
            ]            
        )
    ])
