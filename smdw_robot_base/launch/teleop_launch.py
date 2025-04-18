from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='smdw_robot_base',
            executable='twist_to_twist_stamped',
            name='twist_to_twist_stamped',
            output='screen'
        )
    ])