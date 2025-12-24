import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():
    pkg_name = 'my_robot_description'
    pkg_share = get_package_share_directory(pkg_name)

    # Paths
    urdf_path = os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'urdf_config.rviz')

    # 1. Process URDF
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # 2. Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # 3. Spawn Entity
    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
        output='screen'
    )

    # 4. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # 5. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # 6. Controllers
    joint_state_broadcaster = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    pivot_controller = Node(
        package="controller_manager", executable="spawner",
        arguments=["pivot_controller"],
    )

    drive_controller = Node(
        package="controller_manager", executable="spawner",
        arguments=["drive_controller"],
    )
    
    # 7. Custom Swerve Logic Node
    swerve_node = Node(
        package=pkg_name,
        executable='swerve_controller', # No .py extension here!
        output='screen'
    )

    # Define Launch Order
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        rviz_node,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster, pivot_controller, drive_controller, swerve_node],
            )
        ),
    ])
