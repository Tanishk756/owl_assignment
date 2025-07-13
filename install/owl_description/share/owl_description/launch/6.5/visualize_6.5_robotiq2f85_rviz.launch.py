import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def conditionally_launch_joint_state_publisher(context, *args, **kwargs):
    # Get the value of the 'use_gui' argument
    use_gui = LaunchConfiguration("use_gui").perform(context)

    # Conditionally launch the appropriate joint state publisher
    if use_gui.lower()=="true":
        return [Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        )]
    else:
        return [Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        )]

def conditionally_launch_rviz(context, *args, **kwargs):
    """Conditionally launch RViz based on 'rviz' argument."""
    rviz = LaunchConfiguration("rviz").perform(context)

    if rviz.lower() == "true":
        rviz_config_path = os.path.join(
            get_package_share_directory("owl_description"), "rviz", "robot.rviz"
        )

        return [Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory("owl_description"), "rviz", "robot.rviz")]
        )]
    else:
        return []
    
def generate_launch_description():

    owl_description_pkg = get_package_share_directory("owl_description")

    rviz_arg = DeclareLaunchArgument(
        'rviz', 
        default_value='True', 
        description='Turn RViZ on/off'
    )

    use_gui_arg = DeclareLaunchArgument(
        'use_gui', 
        default_value='False', 
        description='Turn joint state publisherf GUI on/off'
    )

    robot_description_path = PathJoinSubstitution([
        owl_description_pkg,
        "urdf", "6.5", "robot_robotiq2f85.xacro"]
    )

    robot_description_content = Command([
        "xacro ", robot_description_path
    ])

    # Node to publish the robot description to /robot_description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    js_pub_launch_file = OpaqueFunction(function=conditionally_launch_joint_state_publisher)
    rviz_launch_file = OpaqueFunction(function=conditionally_launch_rviz)

    return LaunchDescription([
        rviz_arg,
        use_gui_arg,
        robot_state_publisher_node,
        js_pub_launch_file,
        rviz_launch_file
    ])
