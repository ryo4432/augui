from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    debug_mode = LaunchConfiguration('debug')
    return LaunchDescription([
        DeclareLaunchArgument(
            'debug',
            default_value='False',
            description='If play on debug mode, set to True.'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/augui_launch.py']
            ),
            launch_arguments={
                'debug': debug_mode
            }.items()
        ),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rosbridge_server'),
                    'launch',
                    'rosbridge_websocket_launch.xml'
                ])
            ]),
            launch_arguments={
                "port": "9090",  # default is 9090
                "address": ""  # default is localhost
            }.items()
        )
    ])
