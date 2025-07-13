from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Package directories
    description_pkg = FindPackageShare("owl_description")
    world_path = PathJoinSubstitution([description_pkg, "worlds", "orangewood_world.world"])
    robot_xacro_path = PathJoinSubstitution([description_pkg, "urdf", "6.5", "robot_robotiq2f85.xacro"])

    # Launch arguments
    use_rviz = LaunchConfiguration("use_rviz")

    # Generate robot description from xacro
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        robot_xacro_path
    ])

    return LaunchDescription([
        # Declare optional arguments
        DeclareLaunchArgument("use_rviz", default_value="false", description="Launch RViz"),

        # Start Gazebo server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
            ]),
            launch_arguments={"world": world_path}.items(),
        ),

        # Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}]
        ),

        # Joint State Publisher
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            output="screen"
        ),

        # ros2_control_node
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{"robot_description": robot_description}],
            output="screen"
        ),

        # Spawn robot entity in Gazebo
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-topic", "robot_description",
                "-entity", "owl_robot",
                "-x", "0", "-y", "0", "-z", "0.1"
            ],
            output="screen"
        ),

        # Spawner for joint_state_broadcaster
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_state_broadcaster"],
                    output="screen",
                )
            ]
        ),

        # Spawner for main arm controller
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["arm_controller"],
                    output="screen",
                )
            ]
        ),
    ])
