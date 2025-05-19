import os
import yaml
import pathlib
from launch import LaunchDescription
import launch.actions
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    ld = LaunchDescription()

    param_config = os.path.join(
        get_package_share_directory('msis_pcl'),
        'config',
        'params.yaml'
    )

    node = Node(
        package='msis_pcl',
        executable='msis_pcl',
        name='msis_pcl_node',
        namespace="alpha_rise",
        output='screen',
        parameters=[param_config, {'use_sim_time': True}]        
    )

    ld.add_action(node)

    return ld