from launch import LaunchDescription
import os
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "arm_z_position",
            description="Arm support height from the the top of the base.",
        )
    )

    # Initialize Arguments
    use_rviz = LaunchConfiguration("use_rviz")
    arm_z_position = LaunchConfiguration("arm_z_position")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("boris_description"), "urdf", "boris.urdf.xacro"]
            ),
            " ",
            "arm_z_position:=",
            arm_z_position
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("boris_description"), "rviz", "boris.rviz"]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ("/hoverboard_base_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )

    rviz_node = Node(
       package="rviz2",
       executable="rviz2",
       name="rviz2",
       output="log",
       arguments=["-d", rviz_config_file],
       condition=IfCondition(use_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hoverboard_base_controller", "--controller-manager", "/controller_manager"],
    )
    

    nodes = [        
        robot_state_pub_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
