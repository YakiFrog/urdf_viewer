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
    
    # URDFファイルの絶対パス (パッケージ相対パスに変更)
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
    
    # rviz設定ファイルへのパス
    rviz_config = os.path.join(pkg_dir, 'resource', 'rviz', 'urdf_config.rviz')
    
    # robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': Command(['cat ', LaunchConfiguration('model')]),
                     'use_sim_time': False}],  # use_sim_timeをFalseに変更
        output='screen'
    )
    
    # joint_state_publisherの設定を調整
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': False,  # use_sim_timeをFalseに変更
            'rate': 30,  # 更新レートを上げる
        }],
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
            # joint_statesトピックのブリッジを追加
            '/model/Sirius3/joint_state@sensor_msgs/msg/JointState@ignition.msgs.Model',
            # オドメトリのブリッジを追加し、適切な変換を設定
            # '/model/Sirius3/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            # TF情報を適切に変換するための設定を追加
            '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V'
        ],
        # remappings=[
        #     # オドメトリトピックのマッピングを明示的に設定
        #     ('/model/Sirius3/odometry', '/odom')
        # ],
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
    
    # オドメトリからTFを生成するノード
    # 静的な変換ではなく、動的なオドメトリ情報に基づいてTF変換を発行
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'frequency': 30.0,
            'two_d_mode': True,
            'publish_tf': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'base_footprint',
            'world_frame': 'odom',
            'odom0': '/odom',
            'odom0_config': [
                True, True, False,  # x, y, z position
                False, False, True,  # roll, pitch, yaw
                True, True, False,  # vx, vy, vz
                False, False, True,  # vroll, vpitch, vyaw
                False, False, False  # ax, ay, az
            ]
        }]
    )
    
    # キーボード操作ノード
    teleop_node = Node(
        package='urdf_viewer',
        executable='teleop_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e'  # 別ウィンドウでキーボード入力を受け付ける
    )
    
    # Rviz2ノードを追加
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],  # use_sim_timeをFalseに変更
        output='screen'
    )

    return LaunchDescription([
        declare_model_arg,
        ignition_gazebo,
        robot_state_publisher_node,
        joint_state_publisher_node,
        gz_bridge,
        spawn_robot,
        robot_localization_node,  # 静的TF変換の代わりに動的なTF変換を行うノードを使用
        teleop_node,
        rviz_node,
    ])