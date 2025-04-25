import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # パッケージへのパスを取得
    pkg_name = 'urdf_viewer'
    pkg_dir = get_package_share_directory(pkg_name)
    
    # URDFファイルの絶対パス
    default_urdf_path = '/home/kotantu-desktop/ros2_ws/src/urdf_viewer/models/Sirius3/Sirius3.urdf'
    
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
    
    declare_gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='joint_state_publisher_guiを起動するかどうか'
    )
    
    # rviz設定ファイルへのパス
    rviz_config = os.path.join(pkg_dir, 'resource', 'rviz', 'urdf_config.rviz')
    
    # robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': Command(['cat ', LaunchConfiguration('model')])}],
        output='screen'
    )
    
    # joint_state_publisher (GUIがない場合)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )
    
    # joint_state_publisher_gui (GUIがある場合)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    
    # rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    return LaunchDescription([
        declare_model_arg,
        declare_gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])