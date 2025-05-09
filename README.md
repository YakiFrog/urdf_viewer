# URDF Viewer

URDFビューアーは、ROS 2環境でURDF（Unified Robot Description Format）モデルを簡単に表示・確認するためのパッケージです。

## 機能

- URDFモデルをRviz2で視覚化
- Joint State Publisherによる関節制御
- GUIによるインタラクティブな関節操作
- Gazebo FortressでのURDFモデル表示
- キーボード操作によるロボットの操作デモ

## システム構成

URDFビューアーは主に3つのランチファイルを提供し、それぞれ異なる環境でロボットモデルを表示・操作します。

### Rviz2表示システム（display.launch.py）

以下はRviz2を使用したURDFビューアーのシステム構成図です：

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

### Gazebo Fortress表示システム（display_ignition.launch.py）

以下はGazebo Fortressを使用したURDFビューアーのシステム構成図です：

```mermaid
flowchart TD
    A[URDF Model] -->|読み込み| B[Robot State Publisher]
    A -->|読み込み・シミュレーション| G[Gazebo Fortress]
    H[display_ignition.launch.py] -->|起動| B
    H -->|起動| G
    H -->|起動| I[ROS-GZ Bridge]
    B -->|TF連携| I
    I -->|メッセージ変換| G
    J[spawn_robot] -->|モデルスポーン| G
    H -->|起動| J
```

### 操作デモシステム（demo_teleop.launch.py - 新機能）

以下はGazebo Fortressでロボットを操作するデモシステムの構成図です：

```mermaid
flowchart TD
    A[URDF Model] -->|読み込み| B[Robot State Publisher]
    A -->|読み込み・シミュレーション| G[Gazebo Fortress]
    H[demo_teleop.launch.py] -->|起動| B
    H -->|起動| G
    H -->|起動| I[ROS-GZ Bridge]
    B -->|TF連携| I
    I -->|メッセージ変換| G
    J[spawn_robot] -->|モデルスポーン| G
    H -->|起動| J
    K[Teleop Keyboard] -->|操作コマンド| L[cmd_vel]
    L -->|速度指令| I
    H -->|起動| K
```

## 起動ノードの説明

URDFビューアーでは各ランチファイルで以下の主要なノードが起動されます：

### 共通ノード

#### 1. robot_state_publisher
- **役割**: URDFモデルを読み込み、ロボットの関節状態から変換行列（transforms）を計算し、tfツリーとして公開
- **入力**: URDFモデル（`model`パラメータで指定）とjoint_states
- **出力**: tf/tf2メッセージ
- **パラメータ**: 
  - `robot_description`: URDFファイルの内容

### Rviz2表示システム固有のノード（display.launch.py）

#### 2. joint_state_publisher
- **役割**: URDFで定義された関節の状態（角度や位置）を管理・公開
- **条件**: GUIが無効の場合に起動（`gui=false`）
- **出力**: sensor_msgs/JointState型のjoint_statesメッセージ

#### 3. joint_state_publisher_gui
- **役割**: 関節状態の管理・公開とGUIインターフェースによる対話的操作
- **条件**: GUIが有効の場合に起動（`gui=true`、デフォルト）
- **特徴**: スライダーなどのUIで関節を操作可能
- **出力**: sensor_msgs/JointState型のjoint_statesメッセージ

#### 4. rviz2
- **役割**: ロボットモデルやセンサーデータなどを3D視覚化
- **設定**: urdf_config.rvizファイルを使用
- **入力**: tfデータ、URDFモデル情報

### Gazebo Fortress表示システム固有のノード（display_ignition.launch.py）

#### 5. Gazebo Fortress（gz_sim）
- **役割**: 物理シミュレーション環境でURDFモデルを表示・シミュレーション
- **設定**: empty.sdfワールドを使用
- **入力**: URDFモデル、ros-gzブリッジからのデータ

#### 6. ros_gz_bridge
- **役割**: ROS 2とGazebo Ignition間でメッセージを変換
- **変換対象**:
  - クロック（/clock）
  - ポーズ（/model/Sirius3/pose）
  - TF（/model/Sirius3/tf）

#### 7. spawn_robot（ros_gz_sim create）
- **役割**: URDFモデルをGazebo Fortressの世界にスポーン
- **設定**: モデル名、位置（x, y, z）を指定
- **入力**: URDFモデルファイル

### 操作デモシステム固有のノード（demo_teleop.launch.py - 新機能）

#### 8. teleop_keyboard
- **役割**: キーボード入力を受け付け、ロボットへの速度指令に変換
- **出力**: geometry_msgs/Twist型のcmd_velメッセージ
- **操作方法**:
  - W: 前進
  - S: 後退
  - A: 左回転
  - D: 右回転
  - スペース: 停止
  - Q: 終了

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

### URDFモデルの表示（Rviz2）

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

### URDFモデルの表示（Gazebo Fortress）

Gazebo Fortressでモデルを表示する基本コマンド:

```bash
ros2 launch urdf_viewer display_ignition.launch.py
```

異なるURDFモデルを表示する場合:

```bash
ros2 launch urdf_viewer display_ignition.launch.py model:=/path/to/your/model.urdf
```

### ロボットの操作デモ（新機能）

キーボードでロボットを操作するデモを起動:

```bash
ros2 launch urdf_viewer demo_teleop.launch.py
```

操作方法:
- W: 前進
- S: 後退
- A: 左回転
- D: 右回転
- スペース: 停止
- Q: 終了

**注意**: 操作デモを実行するには、xterm（端末エミュレータ）がインストールされている必要があります。インストールされていない場合は以下のコマンドでインストールしてください:
```bash
sudo apt install -y xterm
```

## Gazebo Fortressでの表示について

Gazebo Fortressは新しいGazeboアーキテクチャに基づく物理エンジンで、以下の特徴があります：

- 高性能・軽量な物理シミュレーション
- モダンなレンダリングエンジン
- ROS 2との統合が容易
- 分散シミュレーション対応

### 必要なパッケージ

Gazebo Fortressでの表示には以下のパッケージが必要です：

```bash
# インストールコマンド
sudo apt install -y ros-humble-ros-gz
```

### システム構成

Gazebo Fortressでの表示時の主要コンポーネント：

1. **robot_state_publisher**: URDFモデル情報とTFツリーを提供
2. **ros_gz_bridge**: ROS 2とGazebo Ignition間でのメッセージ変換
3. **ros_gz_sim**: シミュレーション環境の提供とモデルのスポーン

### モデルの動作制御

ロボットの動作制御は、以下の仕組みで行われています：

1. **ディファレンシャルドライブプラグイン**: URDFファイルに組み込まれたプラグインで、車輪の制御を管理
2. **cmd_velトピック**: 速度指令を伝達するための標準的なROS 2トピック
3. **ROS-GZブリッジ**: ROS 2のcmd_velメッセージをGazebo用に変換

### 技術的制限

- 従来のGazeboタグとIgnitionガゼボのタグが一部異なります
- 複雑な形状や特定のプラグインは互換性がない場合があります
- 初回起動時は処理に時間がかかることがあります

## パッケージ構成

- `launch/`: 起動ファイル
  - `display.launch.py`: Rviz2での表示用ランチファイル
  - `display_ignition.launch.py`: Gazebo Fortress表示用ランチファイル
  - `demo_teleop.launch.py`: 操作デモ用ランチファイル（新規追加）
- `models/`: URDFモデルファイル
  - `Sirius3/`: Sirius3ロボットのURDFモデル
- `resource/rviz/`: Rvizの設定ファイル
- `urdf_viewer/`: Pythonモジュール
  - `teleop_keyboard.py`: キーボード操作ノード（新規追加）

## ディレクトリ構造

```mermaid
graph TD
    A[urdf_viewer] -->|ディレクトリ| B[launch]
    A -->|ディレクトリ| C[models]
    A -->|ディレクトリ| D[resource]
    A -->|ディレクトリ| K[urdf_viewer]
    C -->|ディレクトリ| E[Sirius3]
    D -->|ディレクトリ| F[rviz]
    B -->|ファイル| G[display.launch.py]
    B -->|ファイル| H[display_ignition.launch.py]
    B -->|ファイル| I[demo_teleop.launch.py]
    E -->|ファイル| J[Sirius3.urdf]
    F -->|ファイル| L[urdf.rviz]
    K -->|ファイル| M[teleop_keyboard.py]
```

## 依存パッケージ

- rclpy: ROS 2 Python クライアントライブラリ
- xacro: XMLマクロ言語
- urdf: URDFパーサー 
- joint_state_publisher: 関節状態の公開
- joint_state_publisher_gui: GUI付き関節状態公開
- robot_state_publisher: ロボット状態の公開
- rviz2: 視覚化ツール
- launch_ros: ランチファイルユーティリティ
- ros_gz_bridge: ROS-Gazebo Ignitionブリッジ
- ros_gz_sim: Gazebo Ignitionシミュレーション連携
- geometry_msgs: 幾何学的メッセージ型
- xterm: 外部ターミナルエミュレータ（操作デモ用）