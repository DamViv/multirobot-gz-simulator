from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([FindPackageShare('multirobot_gz_simulator'), 'models'])
        ),
        DeclareLaunchArgument(
            'world_file',
            description='Path to the world file to load into Gazebo'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                ])
            ),
            launch_arguments={
                'gz_args': [TextSubstitution(text=''), LaunchConfiguration('world_file')],
                'on_exit_shutdown': 'True',
                'gui': 'false', 
            }.items(),
        ),
    ])
