from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    moveit_adapter_node = Node(
        package="ik_solver_moveit_adapter",
        executable="ik_adapter_node",
        name="ik_adapter_node",
        output="screen",
        parameters=[
            {
                "custom_ik_service_name": "/ur_ik_solver/get_ik",
                "moveit_service_name": "/ik_adapter_service",
                "seed_weights": [0.0, 0.0,0.0,0.0,0.0,0.0] # [0.0, 0.0,0.0,0.0,0.0,0.0] is random choice!
            }
        ]
    )

    return LaunchDescription([
        moveit_adapter_node
    ])
