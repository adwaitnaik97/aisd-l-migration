from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare a launch argument for the metadata file path
    declare_metadata_file_path = DeclareLaunchArgument(
        'metadata_file_path',
        default_value='/home/adwait/workspace/ros2_packages/aisd-l-migration/src/rosbags_to_csv/bag_files/rosbag2_2025_04_24-10_45_07/metadata.yaml',
        description='Path to the metadata.yaml file'
    )

    # Use the launch argument in the Node configuration
    metadata_file_path = LaunchConfiguration('metadata_file_path')

    return LaunchDescription([
        declare_metadata_file_path,
        Node(
            package='rosbags_to_csv',  # Replace with your package name
            executable='rosbags_to_csv',  # Replace with your node executable name
            name='rosbags_to_csv_node',
            parameters=[
                {'metadata_file_path': metadata_file_path}
            ]
        )
    ])