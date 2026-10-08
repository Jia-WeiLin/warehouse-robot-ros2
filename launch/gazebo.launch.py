from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    package_path = get_package_share_directory('warehouse_robot')
    world_file = os.path.join(
    package_path,
    'worlds',
    'warehouse.world'
    )

    xacro_file = os.path.join(
        package_path,
        'urdf',
        'robot.urdf.xacro'
    )

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # 發布機器人的 URDF 與 TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True
            }
        ]
    )

    # 啟動 Gazebo Sim
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': ['-r ', world_file]
        }.items()
    )

    # 將 robot_description 中的機器人生成到 Gazebo
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'warehouse_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.3'
        ]
    )
    diff_drive_controller_spawner= Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )
    joint_state_broadcaster_spawner= Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )
    lidar_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/warehouse_world/model/warehouse_robot/link/base_link/sensor/lidar_sensor/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        remappings=[
        (
        '/world/warehouse_world/model/warehouse_robot/link/base_link/sensor/lidar_sensor/scan',
        '/scan'
    )
        ],
        output='screen'
)   
    clock_bridge_node = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
    ],
    output='screen'
)
    lidar_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=[
        '0', '0', '0',
        '0', '0', '0',
        'lidar_link',
        'warehouse_robot/base_link/lidar_sensor'
    ],
    output='screen'
)

    return LaunchDescription([
        gazebo_node,
        robot_state_publisher_node,
        spawn_robot_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        clock_bridge_node,
        lidar_tf_node,
        lidar_bridge_node
        
    ])