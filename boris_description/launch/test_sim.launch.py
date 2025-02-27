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

    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        name = 'robot_state_publisher',
        output = 'both',
        parameters=[robot_description],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

     # Node to bridge messages like /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V"
        ],
        output="screen",
        parameters=[
            {'use_sim_time': True},
        ]
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(world_launch)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(spawn_urdf_node)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    launchDescriptionObject.add_action(gz_bridge_node)

    return launchDescriptionObject