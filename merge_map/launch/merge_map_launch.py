from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'map1_topic',
            default_value='/robot1/map',
            description='Topic for the first map'
        ),
        DeclareLaunchArgument(
            'map2_topic',
            default_value='/robot2/map',
            description='Topic for the second map'
        ),
        DeclareLaunchArgument(
            'merged_map_topic',
            default_value='/merge_map',
            description='Topic for the merged map'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='1.0',
            description='Rate at which to publish the merged map (Hz)'
        ),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.7',
            description='Threshold for accepting new information'
        ),
        DeclareLaunchArgument(
            'filter_noise',
            default_value='True',
            description='Whether to filter noise from the merged map'
        ),
        
        # Map merger node
        Node(
            package='merge_map',  # Replace with your actual package name
            executable='merge_map',
            name='map_merger_node',
            output='screen',
            parameters=[{
                'map1_topic': LaunchConfiguration('map1_topic'),
                'map2_topic': LaunchConfiguration('map2_topic'),
                'merged_map_topic': LaunchConfiguration('merged_map_topic'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'filter_noise': LaunchConfiguration('filter_noise'),
            }],
        ),
    ])
