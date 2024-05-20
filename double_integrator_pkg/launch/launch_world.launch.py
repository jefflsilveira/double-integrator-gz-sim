import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    world = 'house_empty'
    package_name = 'double_integrator_pkg'
    pkg_share = get_package_share_directory(package_name)

    default_world_path = os.path.join(get_package_share_directory('custom_worlds'), 'worlds', world + '.world')  
    
    world = launch.actions.IncludeLaunchDescription(
    launch.launch_description_sources.PythonLaunchDescriptionSource([
            get_package_share_directory('custom_worlds'), '/launch/world.launch.py'
        ]), launch_arguments={'world': default_world_path, 'gui': 'true'}.items()
    )

    return launch.LaunchDescription([
        world,
    ])