import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Get the package share directory
    zed_splitter_share_dir = get_package_share_directory('zed_splitter')

    # Define the path to the default config file
    default_conf_file = os.path.join(zed_splitter_share_dir, 'config', 'SN39912280.conf')

    # Zed Splitter Node
    zed_splitter_node = Node(
        package='zed_splitter',
        executable='zed_splitter_node',
        name='zed_splitter_node',
        output='screen',
        parameters=[
            {'conf_file_path': default_conf_file},
            # You can optionally remap the input topic here if it's not /zed/image_raw
            # {'remap': [('/zed/image_raw', '/your/input/topic')]}
        ]
    )

    return LaunchDescription([
        zed_splitter_node
    ])