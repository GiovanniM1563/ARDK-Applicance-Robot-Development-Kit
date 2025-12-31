from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ardk_lifecycle',
            executable='state_manager.py',
            name='state_manager',
            output='screen',
            # We assume root access or properly configured sudoers isn't strictly needed 
            # for the subprocesses if running as the user who owns the ROS install/workspace.
            # Smoke tests ran fine as user 'gio'.
        )
    ])
