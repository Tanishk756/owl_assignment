# owl_description/launch/realsense.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'enable_rgb': True,
                'enable_depth': True,
                'enable_pointcloud': True,
                'align_depth.enable': True,
                'pointcloud.enable': True,
                'unite_imu_method': 'none',
                'publish_tf': True,
            }],
            remappings=[
                ('/camera/color/image_raw', '/camera/color/image_raw'),
                ('/camera/depth/image_rect_raw', '/camera/depth/image_rect_raw'),
            ]
        )
    ])
