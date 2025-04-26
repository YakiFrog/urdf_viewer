#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # パッケージへのパスを取得
    pkg_name = 'urdf_viewer'
    pkg_dir = get_package_share_directory(pkg_name)
    
    # URDFファイルの絶対パス
    default_urdf_path = os.path.join(pkg_dir, 'models', 'Sirius3', 'Sirius3.urdf')
    
    # パスが存在するかログ出力
    print(f"URDFファイルパス: {default_urdf_path}")
    if not os.path.exists(default_urdf_path):
        print(f"警告: URDFファイルが見つかりません: {default_urdf_path}")
    
    # LaunchParameterの宣言
    declare_model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_urdf_path,
        description='URDFモデルへの絶対パス'
    )
    
    # robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': Command(['cat ', LaunchConfiguration('model')]),
                     'use_sim_time': True}],
        output='screen'
    )
    
    # Gazebo Ignitionでのブリッジ設定
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
            '/model/Sirius3/pose@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/model/Sirius3/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
        ],
        output='screen'
    )

    # URDFをGazebo Fortressにスポーンするスクリプト
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        arguments=[
            '-file', LaunchConfiguration('model'),
            '-name', 'Sirius3',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # Gazebo Ignitionの起動（少し改良したワールドを使用）
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': '-r empty.sdf'
        }.items()
    )
    
    # キーボード操作ノード
    teleop_node = Node(
        package='urdf_viewer',
        executable='teleop_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e'  # 別ウィンドウでキーボード入力を受け付ける
    )
    
    return LaunchDescription([
        declare_model_arg,
        ignition_gazebo,
        robot_state_publisher_node,
        gz_bridge,
        spawn_robot,
        teleop_node,
    ])