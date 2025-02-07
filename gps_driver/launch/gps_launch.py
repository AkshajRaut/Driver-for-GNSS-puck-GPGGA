from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_driver',
            executable='driver',     
            name='gps_driver_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
            }]
        )
    ])
