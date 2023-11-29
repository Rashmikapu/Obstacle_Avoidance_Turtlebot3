
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
import os

def generate_launch_description():
    record_flag = LaunchConfiguration('record_flag')
    return LaunchDescription([

        DeclareLaunchArgument(
            'record_flag',
            default_value='False'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch'),
            '/turtlebot3_world.launch.py'])
        ),
        Node(
            package="turtlebot_walker",
            executable="walker",
            name="walker",
            output="screen",
            emulate_tty=True,
        ),
        ExecuteProcess(
        condition=IfCondition(record_flag),
        cmd=[
            'ros2', 'bag', 'record', '-o walker_bag', '-a', '-x "/camera.+"'
        ],
        shell=True
        )
    ])