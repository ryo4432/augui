from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'debug',
            default_value='False',
            description='If play on debug mode, set to True.'
        ),
        Node(
            package='augui',
            executable='augui'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'augui', 'mockup'],
            output='screen',
            log_cmd=True,
            condition=LaunchConfigurationEquals('debug', 'True'))
    ])
