from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pathlib import Path
import os
from ament_index_python.packages import get_package_share_directory

nav2_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config','planner_server.yaml')
controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config','controller.yaml')
bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config','bt_navigator.yaml')
recovery_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config','recovery.yaml')

def generate_launch_description():
    ## ***** Launch arguments *****
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="false", description="Open RViz."
    )

    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]

        if "SDF_PATH" in os.environ:
            sdf_path = os.environ["SDF_PATH"]
            os.environ["SDF_PATH"] = sdf_path + ":" + gz_sim_resource_path
        else:
            os.environ["SDF_PATH"] = gz_sim_resource_path

    return LaunchDescription([
        use_sim_time_arg,
        rviz_arg,

        # Nav2 servers
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_yaml]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml]
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            output='screen',
            parameters=[recovery_yaml]
        ),

        # Cartographer nodes
        Node(
            package="cartographer_ros",
            executable="cartographer_node",
            name="cartographer_ros",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            arguments=[
                "-configuration_directory",
                FindPackageShare("ardupilot_cartographer").find("ardupilot_cartographer") + "/config",
                "-configuration_basename",
                "cartographer.lua",
            ],
            output="screen",
            remappings=[
                ("/imu", "/imu"),
                ("/odom", "/odometry"),
                ('scan', '/scan')
            ],
        ),
        Node(
            package="cartographer_ros",
            executable="cartographer_occupancy_grid_node",
            name="cartographer_occupancy_grid_node_ros",
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                {"resolution": 0.05},
            ],
        ),

        # Lifecycle manager (only for Nav2 lifecycle nodes)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'autostart': True},
                {'node_names': [
                    'planner_server',
                    'controller_server',
                    'bt_navigator',
                    'recoveries_server'
                ]}
            ]
        ),

        # Static TF
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
        # ),

        # RViz
        # Node(
        #     package="rviz2",
        #     executable="rviz2",
        #     name="rviz2_cartographer",
        #     arguments=[
        #         "-d",
        #         str(Path(
        #             FindPackageShare("ardupilot_cartographer").find("ardupilot_cartographer"),
        #             "rviz",
        #             "cartographer.rviz",
        #         ))
        #     ],
        #     condition=IfCondition(LaunchConfiguration("rviz")),
        # ),

        # Custom nodes
        Node(
            package='path_planner_server',
            executable='cmdvel_to_stamped.py',
            name='cmdvel_to_stamped',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='path_planner_server',
            executable='nav_to_pose_action_client.py',
            name='nav_to_pose_action_client',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
