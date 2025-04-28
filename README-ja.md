# Unity_ROS2_ロボットシミュレータ
[English](README.md) | 日本語

Unity をベースとした ROS2（Robot Operating System 2）と連携するロボットシミュレータです。ロボティクスのシミュレーションと開発のための高精細な視覚環境を提供します。

## 概要
このプロジェクトは、ROS2 通信機能を備えた Unity でのロボットシミュレーションを可能にし、物理的なハードウェアへの展開前に現実的な仮想環境でロボットアルゴリズムをテストすることができます。

## 特徴
- Unity の物理エンジンによるリアルな物理シミュレーション
- 標準的なロボティクス通信のための ROS2 統合
- 様々なロボットモデルとセンサータイプのサポート
- 異なるテストシナリオのためのカスタマイズ可能な環境

## 必要条件
- Unity 6000.0.47f1 LTS 以降
- ROS2 (Humble 以降)
- [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)
- [URDF-Importer](https://github.com/Unity-Technologies/URDF-Importer)
- [UnitySensors](https://github.com/Field-Robotics-Japan/UnitySensors)
- [UnitySensorsROS](https://github.com/Field-Robotics-Japan/UnitySensors)
注: 現在、UnitySensors と UnitySensorsROS は修正版を使用しています。

## インストール
1. このリポジトリをクローンします
2. Unity でプロジェクトを開きます
3. (任意) アプリケーションをビルドします

## 使用方法
1. ビルドしたアプリケーションまたは Editor からプロジェクトを実行します

2. 以下のコマンドで TCP Connector を実行します
   ```bash
   ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
   ```

3. ロボットモデルとセンサーを設定します
   ```bash
   ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{ name: '<YOURROBOTNAME>', allow_renaming: false, uri: '/your/urdf/path/robot.urdf', resource_string: '', entity_namespace: '', initial_pose: { header: { stamp: { sec: 0, nanosec: 0 }, frame_id: '' }, pose: { position: { x: 0.0, y: 0.0, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } } } }"
   ```

4. シミュレーションを開始します
   ```bash
   ros2 service call /set_simulation_state simulation_interfaces/srv/SetSimulationState "{ state: { state: 1 } }"
   ```

注: これらのサービスは [simulation_interfaces](https://github.com/ros-simulation/simulation_interfaces) に基づいています

## ライセンス
このプロジェクトは Apache 2.0 ライセンスの下で提供されています - 詳細はライセンスファイルをご覧ください。
