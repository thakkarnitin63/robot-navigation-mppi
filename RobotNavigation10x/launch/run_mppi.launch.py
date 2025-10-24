"""
MPPI Controller Launch File
Launches TurtleBot3 in obstacle environment with MPPI-based trajectory tracking.
Demonstrates advanced model predictive control with hybrid obstacle avoidance.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Locate required ROS2 packages
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot_navigation_10x = get_package_share_directory('RobotNavigation10x')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    launch_file_dir = os.path.join(pkg_turtlebot3_gazebo, 'launch')

    # Simulation configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Robot spawn position (bottom-left of obstacle world)
    x_pose = LaunchConfiguration('x_pose', default='-4.5')
    y_pose = LaunchConfiguration('y_pose', default='-4.5')

    # Environment setup: obstacle world for testing avoidance
    world = os.path.join(
        pkg_robot_navigation_10x,
        'worlds',
        'nav_world_obst.world'
    )
    
    # MPPI controller parameters
    mppi_params_file = os.path.join(
        pkg_robot_navigation_10x,
        'config',
        'mppi_params.yaml'
    )
    
    # TurtleBot3 robot description
    urdf_file_name = 'turtlebot3_burger.urdf'
    urdf_path = os.path.join(pkg_turtlebot3_gazebo, 'urdf', urdf_file_name)
    with open(urdf_path, 'r') as f:
        robot_description_config = f.read()

    # Launch Gazebo simulation server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # Launch Gazebo GUI client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Publish robot state transformations
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_config
        }]
    )

    # Spawn TurtleBot3 in simulation
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # Start MPPI tracker with obstacle avoidance
    mppi_tracker_node = Node(
        package='RobotNavigation10x',
        executable='mppi_tracker_node',
        name='mppi_tracker',
        output='screen',
        parameters=[mppi_params_file]
    )

    # Assemble launch description
    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(mppi_tracker_node)

    return ld