from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    map_name = LaunchConfiguration('map_name')
    
    # Сервер карты
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': os.path.join(
            get_package_share_directory('my_navigation'), 'maps', f'{map_name}.yaml')}],
        output='screen'
    )
    
    # AMCL
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[os.path.join(get_package_share_directory('my_navigation'), 'config', 'amcl_config.yaml')],
        output='screen'
    )
    
    # Запуск move_base
    move_base = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('my_navigation'), 'launch', 'move_base.py')
    )
    
    # Менеджер жизненного цикла
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'autostart': True},
                   {'node_names': ['map_server', 'amcl']}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('map_name', default_value='map'),
        map_server,
        amcl,
        move_base,
        lifecycle_manager
    ])