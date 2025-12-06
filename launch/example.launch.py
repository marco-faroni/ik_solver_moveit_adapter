from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Arguments

    # Get config file path
    config_file = os.path.join(
        get_package_share_directory('ik_solver_moveit_adapter'),
        'config',
        'params.yaml'
    )

    ik_node = Node(
        package='ik_solver_moveit_adapter',
        executable='ik_adapter_node',
        name='ik_adapter_node',
        output='screen',
        #parameters=[config_file]
    )

    return LaunchDescription([
        ik_node
    ])
