from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Получаем путь к директории с конфигами
    package_dir = get_package_share_directory('my_navigation')
    
    # Запуск SLAM (аналог gmapping для ROS 2)
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[os.path.join(package_dir, 'config', 'slam_config.yaml')],
        output='screen'
    )
    
    # Запуск навигации
    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[os.path.join(package_dir, 'config', 'controller_server.yaml')]
    )
    
    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[os.path.join(package_dir, 'config', 'planner_server.yaml')]
    )
    
    # Менеджер жизненного цикла
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'autostart': True},
                   {'node_names': ['controller_server',
                                   'planner_server']}]
    )
    
    return LaunchDescription([
        slam,
        controller,
        planner,
        lifecycle_manager
    ])