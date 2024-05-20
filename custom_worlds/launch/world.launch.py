import os
import sys

import launch
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = 'custom_worlds'
    pkg_share = get_package_share_directory(package_name)
    
    world_file_name_default = "house_empty.world"
    world = os.path.join(get_package_share_directory('custom_worlds'), 'worlds', world_file_name_default)

    gazebo_ros = get_package_share_directory('gazebo_ros')
    gazebo_client = launch.actions.IncludeLaunchDescription(
	launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('gui'))
     )
    
    gazebo_config = os.path.join(pkg_share, 'config', 'gazebo_config.yaml') 

    gazebo_server = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_config
        }.items()
    )


    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
          'world',
          default_value=[world, ''],
          description='SDF world file'),
        launch.actions.DeclareLaunchArgument(
            name='gui',
            default_value='false'
        ),
        gazebo_server,
        gazebo_client
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
