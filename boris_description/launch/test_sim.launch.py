import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable, PathJoinSubstitution, Command
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Arguments 
    rviz_launch_arg = DeclareLaunchArgument(
        'use_rviz', default_value = 'true',
        description = 'Start RViz2 automatically with this launch file.',
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('boris_description'), 'rviz', 'boris.rviz']
    )

    world_arg = DeclareLaunchArgument(
        'world', default_value = 'world.sdf',
        description = 'Name of the Gazebo world file to load'
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name = 'xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('boris_description'), 'urdf', 'boris.xacro']
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    pkg_boris_description = get_package_share_directory('boris_description')

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare('boris_description'), 'config', 'boris_controllers.yaml']
    )

    model_arg = DeclareLaunchArgument(
        'model', default_value = robot_description,
        description = 'Name of the URDF description to load'
    )
    
    # Nodes launch
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_boris_description, 'launch', 'world.launch.py'),
        ),
        launch_arguments = {
        'world': LaunchConfiguration('world'),
        }.items()
    )

    # Launch rviz
    rviz_node = Node(
        package ='rviz2',
        executable ='rviz2',
        arguments = ['-d', rviz_config_file],
        condition = IfCondition(LaunchConfiguration('use_rviz')),
        parameters = [
            {'use_sim_time': True},
        ]
    )

    # Spawn the URDF model using the `/world/<world_name>/create` service
    spawn_urdf_node = Node(
        package = 'ros_gz_sim',
        executable = 'create',
        arguments = [
            '-name', 'boris_sim',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.5', '-Y', '0.0'  # Initial spawn position
        ],
        output='screen',
        parameters=[
            {'use_sim_time': True},
        ]
    )

    control_node = Node(
        package = 'controller_manager',
        executable = 'ros2_control_node',
        parameters = [robot_description, robot_controllers],
        output = 'both',
    )

    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        name = 'robot_state_publisher',
        output = 'both',
        parameters=[robot_description],
        remappings=[
            ('/hoverboard_base_controller/cmd_vel_unstamped', '/cmd_vel'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    joint_state_publisher_node = Node(
        package = 'joint_state_publisher',
        executable = 'joint_state_publisher',
        name = 'joint_state_publisher',
        output = 'both',
        parameters = [{'source_list': "['/mobile_wx200/joint_states']",}],
    )

    joint_state_publisher_gui_node = Node(
        package = 'joint_state_publisher_gui',
        executable = 'joint_state_publisher_gui',
    )

    joint_state_broadcaster_spawner = Node(
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    robot_controller_spawner = Node(
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['hoverboard_base_controller', '--controller-manager', '/controller_manager'],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
       event_handler = OnProcessExit(
           target_action = joint_state_broadcaster_spawner,
           on_exit = [rviz_node],
       )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler = OnProcessExit(
            target_action = joint_state_broadcaster_spawner,
            on_exit = [robot_controller_spawner],
        )
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(world_launch)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(spawn_urdf_node)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    launchDescriptionObject.add_action(joint_state_publisher_gui_node)

    # launchDescriptionObject.add_action(control_node)
    # launchDescriptionObject.add_action(joint_state_publisher_node)
    # launchDescriptionObject.add_action(joint_state_broadcaster_spawner)
    # launchDescriptionObject.add_action(delay_rviz_after_joint_state_broadcaster_spawner)
    # launchDescriptionObject.add_action(delay_robot_controller_spawner_after_joint_state_broadcaster_spawner)

    return launchDescriptionObject