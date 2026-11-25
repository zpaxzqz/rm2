from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_dir = PathJoinSubstitution([FindPackageShare('board_recognition'), 'config'])
    source = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('camera_frame'), 'launch', 'source.launch.py',
        ])),
        launch_arguments={
            name: LaunchConfiguration(name)
            for name in ('source', 'video_path', 'camera_info_url', 'loop')
        }.items(),
    )
    detector = Node(
        package='board_recognition',
        executable='board_recognition_node',
        output='screen',
        parameters=[{
            'board_config': ParameterValue(LaunchConfiguration('board_config'), value_type=str),
            'show_window': ParameterValue(LaunchConfiguration('show_window'), value_type=bool),
            'input_timeout_sec': ParameterValue(
                LaunchConfiguration('input_timeout_sec'), value_type=float),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument('source', default_value='video', choices=['video', 'live']),
        DeclareLaunchArgument('video_path', default_value=''),
        DeclareLaunchArgument(
            'camera_info_url', default_value=[
                'file://', PathJoinSubstitution([config_dir, 'DA3113952.yaml']),
            ],
        ),
        DeclareLaunchArgument('loop', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'board_config', default_value=PathJoinSubstitution([config_dir, 'board.yaml']),
        ),
        DeclareLaunchArgument('show_window', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('input_timeout_sec', default_value='0.5'),
        detector,
        source,
    ])
