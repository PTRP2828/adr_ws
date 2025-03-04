from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('host_control')
    
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        
        # Static Transform Publisher (if needed)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_broadcaster',
            arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'laser_frame'],
            output='screen'
        ),
        
        # SLAM Toolbox node with modified parameters
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_file,
                {
                    'use_sim_time': use_sim_time,
                    'max_laser_range': 12.0,
                    'minimum_time_interval': 0.2,
                    'transform_timeout': 0.5,
                    'update_rate': 5.0
                }
            ]
        ),
        
        # Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'config', 'mapping.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])
