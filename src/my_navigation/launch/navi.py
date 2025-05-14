from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Параметры запуска
    use_amcl = LaunchConfiguration('use_amcl', default='true')
    map_name = LaunchConfiguration('map_name', default='map')
    
    # Запуск Stage через мост ROS 1
    stage_bridge = Node(
        package='ros1_bridge',
        executable='dynamic_bridge',
        name='stage_bridge',
        output='screen'
    )
    
    # Навигация с AMCL или без
    navigation_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('my_navigation'), 'launch', 'navigation_amcl.py'),
        condition=IfCondition(use_amcl),
        launch_arguments={'map_name': map_name}.items()
    ) if use_amcl else IncludeLaunchDescription(
        os.path.join(get_package_share_directory('my_navigation'), 'launch', 'my_navigation.py')
    )
    
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('my_navigation'), 'navi.rviz']],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_amcl', default_value='true'),
        DeclareLaunchArgument('map_name', default_value='map'),
        stage_bridge,
        navigation_launch,
        rviz
    ])