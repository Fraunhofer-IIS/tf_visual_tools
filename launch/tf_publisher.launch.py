import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    
    rviz_file = os.path.join(get_package_share_directory('tf_visual_tools'), 'launch', 'tf.rviz')

    return LaunchDescription([
        
        # # Start RViz with the demo file
        Node(
            package='rviz2', 
            executable='rviz2', 
            arguments=['--display-config', rviz_file],
            output='screen'
        ),
        
        # Start tf_visual_tools_publisher node
        Node(
            package='tf_visual_tools',
            executable='tf_publisher_node',
            output='screen'
        )
    ])
