from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rb5_vision_dir = get_package_share_directory('rb5_ros2_vision')
    rb_camera_main_ocv_launch = os.path.join(rb5_vision_dir, 'launch', 'rb_camera_main_ocv_launch.py')

    return LaunchDescription([
        # Including another launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rb_camera_main_ocv_launch)
        ),
        
        # April detection node
        Node(
            package='ros2_april_detection',
            executable='april_detection_node',
            name='april_detection_node',
            output='log'
        ),

        # Kalman filter node
        Node(
            package='hw3',
            executable='kalman_filter.py',
            name='kalman_filter',
        ),

        # Mpi twist
        Node(
            package='hw3',
            executable='mpi_twist.py',
            name='mpi_twist',
            output='log'
        ),

        # Controller
        Node(
            package='hw3',
            executable='control_node.py',
            name='control_node',
            output='log'
        )

    ])