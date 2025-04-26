# URDF Viewer

URDFビューアーは、ROS 2環境でURDF（Unified Robot Description Format）モデルを簡単に表示・確認するためのパッケージです。

## 機能

- URDFモデルをRviz2で視覚化
- Joint State Publisherによる関節制御
- GUIによるインタラクティブな関節操作

## システム構成

以下はURDFビューアーのシステム構成図です：

```mermaid
flowchart TD
    A[URDF Model] -->|読み込み| B[Robot State Publisher]
    C[Joint State Publisher] -->|関節状態| B
    C -->|GUIで制御| D[Joint State Publisher GUI]
    B -->|TF発行| E[Rviz2]
    A -->|表示| E
    F[display.launch.py] -->|起動| B
    F -->|起動| C
    F -->|起動| D
    F -->|起動| E
```

## 起動ノードの説明

URDFビューアーでは以下の主要なノードが起動されます：

### 1. robot_state_publisher
- **役割**: URDFモデルを読み込み、ロボットの関節状態から変換行列（transforms）を計算し、tfツリーとして公開
- **入力**: URDFモデル（`model`パラメータで指定）とjoint_states
- **出力**: tf/tf2メッセージ
- **パラメータ**: 
  - `robot_description`: URDFファイルの内容

### 2. joint_state_publisher
- **役割**: URDFで定義された関節の状態（角度や位置）を管理・公開
- **条件**: GUIが無効の場合に起動（`gui=false`）
- **出力**: sensor_msgs/JointState型のjoint_statesメッセージ

### 3. joint_state_publisher_gui
- **役割**: 関節状態の管理・公開とGUIインターフェースによる対話的操作
- **条件**: GUIが有効の場合に起動（`gui=true`、デフォルト）
- **特徴**: スライダーなどのUIで関節を操作可能
- **出力**: sensor_msgs/JointState型のjoint_statesメッセージ

### 4. rviz2
- **役割**: ロボットモデルやセンサーデータなどを3D視覚化
- **設定**: urdf_config.rvizファイルを使用
- **入力**: tfデータ、URDFモデル情報

## TF（Transform）ツリー

URDFモデルから生成されるTFツリーは、ロボットの各リンク間の位置関係を表します。以下は一例です：

```
base_link
├── link1
│   └── link2
│       └── link3
└── link4
    └── link5
```

TFを確認するには以下のコマンドを使用できます：

```bash
# URDFビューアーを起動した状態で実行してください
# TFツリーを表示（ROS 2 Humble用）
ros2 run tf2_tools view_frames

# 特定のTF関係を確認
ros2 run tf2_ros tf2_echo [source_frame] [target_frame]

# TFデータを確認
ros2 topic echo /tf
```

実行後に生成される`frames.pdf`ファイルでTFツリーの全体構造を確認できます。

**注意**: TFコマンドを実行するには、事前にURDFビューアーを起動してtfデータが配信されている必要があります。

## 使用方法

### インストール

```bash
cd ~/ros2_ws
colcon build --packages-select urdf_viewer
source install/setup.bash
```

### URDFモデルの表示

基本的な起動コマンド:

```bash
ros2 launch urdf_viewer display.launch.py
```

異なるURDFモデルを表示する場合:

```bash
ros2 launch urdf_viewer display.launch.py model:=/path/to/your/model.urdf
```

GUIなしで起動する場合:

```bash
ros2 launch urdf_viewer display.launch.py gui:=false
```

## パッケージ構成

- `launch/`: 起動ファイル
- `models/`: URDFモデルファイル
  - `Sirius3/`: Sirius3ロボットのURDFモデル
- `resource/rviz/`: Rvizの設定ファイル

## ディレクトリ構造

```mermaid
graph TD
    A[urdf_viewer] -->|ディレクトリ| B[launch]
    A -->|ディレクトリ| C[models]
    A -->|ディレクトリ| D[resource]
    C -->|ディレクトリ| E[Sirius3]
    D -->|ディレクトリ| F[rviz]
    B -->|ファイル| G[display.launch.py]
    E -->|ファイル| H[sirius3.urdf]
    F -->|ファイル| I[urdf.rviz]
```

## 依存パッケージ

- rclpy
- xacro
- urdf
- joint_state_publisher
- joint_state_publisher_gui
- robot_state_publisher
- rviz2
- launch_ros