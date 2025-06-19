import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Get the package share directory
    zed_splitter_share_dir = get_package_share_directory('zed_splitter')
    
    # Define the default path to the config file
    default_conf_file = os.path.join(zed_splitter_share_dir, 'config', 'SN39912280.conf')

    # Declare launch arguments
    declared_args = [
        DeclareLaunchArgument(
            'conf_file_path', 
            default_value=default_conf_file,
            description='Full path to the ZED camera calibration .conf file.'
        ),
        DeclareLaunchArgument(
            'video_device', 
            default_value='/dev/video2',
            description='Video device path for the camera.'
        ),
        DeclareLaunchArgument(
            'frame_width', 
            default_value='2560', # Default to HD resolution (2*1280=2560)
            description='Total width of the combined side-by-side frame.'
        ),
        DeclareLaunchArgument(
            'frame_height', 
            default_value='720', # Default to HD resolution
            description='Height of the frame.'
        ),
        DeclareLaunchArgument(
            'fps', 
            default_value='30',
            description='Frames per second to request from the camera.'
        ),
    ]

    # Zed Splitter Node that captures from the camera
    zed_splitter_node = Node(
        package='zed_splitter',
        executable='zed_splitter_node',
        name='zed_splitter_node',
        output='screen',
        parameters=[{
            'conf_file_path': LaunchConfiguration('conf_file_path'),
            'video_device': LaunchConfiguration('video_device'),
            'frame_width': LaunchConfiguration('frame_width'),
            'frame_height': LaunchConfiguration('frame_height'),
            'fps': LaunchConfiguration('fps'),
        }]
    )

    return LaunchDescription(declared_args + [zed_splitter_node])