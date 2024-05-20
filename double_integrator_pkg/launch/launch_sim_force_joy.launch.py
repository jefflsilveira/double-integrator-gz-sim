import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():

    package_name='double_integrator_pkg'
    joy_params = os.path.join(get_package_share_directory(package_name),'config','joystick_force.yaml')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp_force.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo_world = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('double_integrator_pkg'), 'launch', 'launch_world.launch.py')]),
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')


    rviz_node = Node(package='rviz2', executable='rviz2',
                     arguments=['-d', os.path.join(get_package_share_directory(package_name), 'rviz', 'rviz_base.rviz')],
                     output='screen')

    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params],
        )
    
    joy_teleop_node = Node(
            package='joy_teleop',
            executable='joy_teleop',
            name='joy_teleop',
            parameters=[joy_params],
            )

    odom_tf_pub = Node(
        package='odom_tf_pub',
        executable='odom_tf_pub',
        name= 'odom_tf_pub',
        parameters=[{'use_sim_time': True}],
    )
    
    
    
    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo_world,
        spawn_entity,
        # rviz_node,
        joy_node,
        joy_teleop_node,
        odom_tf_pub
        
    ])
