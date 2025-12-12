from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Get config file path
    config_file = os.path.join(
        get_package_share_directory('ik_solver_moveit_adapter'),
        'config',
        'example_ur5e.yaml'
    )

    moveit_adapter_node = Node(
        package="ik_solver_moveit_adapter",
        executable="ik_adapter_node",
        name="ik_adapter_node",
        output="screen",
        parameters=[
            {
                "custom_ik_service_name": "/ur_ik_solver/get_ik",
                "moveit_service_name": "/ik_adapter_service",
                "seed_weights": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0] # per-joint weights to computed weighted distance from seed
            }
        ]
    )

    
    ik_node = Node(
        package='ik_solver',
        executable='ik_solver_node',
        name='ur_ik',
        output='screen',
        namespace='ur_ik_solver'
    )

    load_params = ExecuteProcess(cmd=['cnr_param_server', '-p', str(config_file)])
    
    return LaunchDescription([
        ik_node,
        moveit_adapter_node,
        load_params
    ])
