# Unity_ROS2_ロボットシミュレータ
[English](README.md) | 日本語

Unity をベースとした ROS2（Robot Operating System 2）と連携するロボットシミュレータです。ロボティクスのシミュレーションと開発のための高精細な視覚環境を提供します。

## 概要
このプロジェクトは、ROS2 通信機能を備えた Unity でのロボットシミュレーションを可能にし、物理的なハードウェアへの展開前に現実的な仮想環境でロボットアルゴリズムをテストすることができます。

一般的な2輪移動ロボットのサンプルプロジェクトは[ここ](https://github.com/REACT-ROBOT/Unity_ROS2_sample)にあります。

## 特徴
- Unity の物理エンジンによるリアルな物理シミュレーション
- 標準的なロボティクス通信のための ROS2 統合
- 様々なロボットモデルとセンサータイプのサポート
- 異なるテストシナリオのためのカスタマイズ可能な環境

## 必要条件
- Unity 6000.3.21f1 (検証済み)。6000.0.47f1 LTS 以降で動作します
- ROS 2 Humble (Ubuntu 22.04) または Jazzy (Ubuntu 24.04) — どちらも検証済み
- [ROS-TCP-Connector (hijimasa fork)](https://github.com/hijimasa/ROS-TCP-Connector) — publisher の登録解除 API が必要
- [ROS-TCP-Endpoint (hijimasa fork)](https://github.com/hijimasa/ROS-TCP-Endpoint) — **本家は不可**。`__remove_subscriber` が未実装で、受け取ると TCP 接続ごと落ちます
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
   ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{ name: '<YOURROBOTNAME>', allow_renaming: false, entity_resource: { uri: 'file:///your/urdf/path/robot.urdf', resource_string: '' }, entity_namespace: '', initial_pose: { header: { stamp: { sec: 0, nanosec: 0 }, frame_id: '' }, pose: { position: { x: 0.0, y: 0.0, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } } } }"
   ```

4. シミュレーションを開始します
   ```bash
   ros2 service call /set_simulation_state simulation_interfaces/srv/SetSimulationState "{ state: { state: 1 } }"
   ```

注: これらのサービスは [simulation_interfaces](https://github.com/ros-simulation/simulation_interfaces) **2.1.0** に基づいています。
2.0.0 で `uri` と `resource_string` が `Resource` メッセージ (`entity_resource`) にまとめられているので、
それ以前の書き方とは互換性がありません。

対応しているサービスは `get_simulator_features` で確認できます。

```bash
ros2 service call /get_simulator_features simulation_interfaces/srv/GetSimulatorFeatures "{}"
```

複数体を一度に生成する場合は `spawn_entities` を使います (`spawn_entity` は 2.0.0 で deprecated)。
同じ URDF から複数体出すときは、URDF に書かれたトピック名が衝突するため `entity_namespace` を
指定してください。

### GUI からロボットをスポーンする

ROS を使わずにロボットを出すこともできます。サイドバーの**メッシュファイルボタン**は
メッシュ形式に加えて `.urdf` も受け付けます。メッシュを選べば景観オブジェクト、URDF を
選べば `/spawn_entity` と同じ経路を通ってロボットがスポーンされます。生成されたロボットは
通常のエンティティです: `get_entities` に載り、`/joint_states` などのロボット単位の
トピックを publish し、`delete_entity` で消せて、`reset_simulation` の `SCOPE_SPAWNED` で
デスポーンされます。エンティティ名はファイル名由来 (衝突時は `_1`, `_2`, … を付加) で、
名前空間は付きません。位置は原点から ROS x 方向へ 1 m 刻みで探した最初の空き地点の
地面上です。`.xacro` は非対応なので、先に展開してください
(`ros2 run xacro xacro robot.urdf.xacro > robot.urdf`)。サービスと同じく、ロボットの
スポーンにはワールドがロード済みである必要があります。URDF の隣で解決できない
`package://` のメッシュ参照は `simulation_resources.json` の探索パスから解決されます。
失敗の理由はログに出ます。

### エンティティ一覧と関節スライダ

画面右下のボタン列 (追加/保存/読込ボタンの並び) にあるロボットアイコンのボタンで
右端のパネルがスライドして開き、スポーン済みのエンティティが一覧されます
(`get_entities` と同じ内容で、約 1 秒ごとに更新)。エンティティを選ぶと可動関節ごとに
スライダが出ます (revolute/prismatic は URDF のリミット範囲、continuous は -180…180 度
表示)。ラベルには関節名と現在位置が出ます。スライダを動かすと `/joint_states` の指令と
同じ経路 (ドライブ目標、サーボモデル指定時はモデル経由) で関節に指令が入るため、手動
スライダと ROS 指令は同じ書き込み先を取り合い、後から書いたほうが勝ちます。ドラッグして
いない間はスライダが実際の関節位置に追従するので、外部からの指令による動きも見えます。

### センサ可視化

関節スライダの下に、選択中のロボットの可視化できるセンサ出力が On/Off ボタン付きで
並びます。点群系の出力 (LiDAR スキャン、深度・RGBD カメラの点群) は UnitySensors 同梱の
ビジュアライザで 3D ビューに重畳表示され、カメラ系の出力 (RGB/魚眼/パノラマ画像、RGBD の
カラー) はパネル内のライブプレビューとして表示されます。点群の重畳はパネルを閉じたり
別のエンティティを選んでも表示されたままです (再選択するとボタン状態が復元されます)。
画像プレビューは表示先がパネル内なので、パネルと一緒に閉じます。重畳はセンサカメラが
描画しない専用レイヤーに描くため、ROS へ配信される画像には何を可視化しても影響しません。
深度カメラの生の深度画像はプレビュー対象外です (メートル単位の float テクスチャのため
白飛びして見えません) — 代わりに点群を使ってください。デバッグ用に、環境変数
`SIM_AUTO_SENSOR_VIZ=N` (N ≥ 1) を付けて起動すると最初にスポーンされたエンティティを
自動選択して全可視化を ON にし、OFF→ON の切り替えを N 回繰り返します (最後は ON のまま)。
取り付け/解除の経路をヘッドレスで通すための入り口です。

srv で定義されている 22 サービスすべてに対応しています (エンティティの取得・変更・削除、
`step_simulation`、ワールドの読み込みを含みます)。各サービスの解釈、ワールドをシーン JSON に
対応づけている理由、`get_spawnables` や名前付き姿勢の設定ファイル `simulation_resources.json`
については [docs/Simulation-Interfaces-Services-ja.md](docs/Simulation-Interfaces-Services-ja.md)
を参照してください。

## シミュレーション時刻

シミュレーション時刻は `/clock` (`rosgraph_msgs/msg/Clock`、最大 100 Hz。実際の周期は
アプリのフレームレート — 起動時 10 FPS、UI から変更可 — で頭打ちになります) で配信されるので、
`use_sim_time` を付けて起動した ROS 2 ノードはシミュレータの時計に従います。停止中・一時
停止中も配信は止まらず、Gazebo と同じく値が凍結するだけです。`reset_simulation` を
`SCOPE_TIME` で呼ぶと時刻はゼロに戻ります。ここから配信されるメッセージのスタンプ
(`/joint_states`、`/ground_truth`、`/tf`) もすべて同じ時計から取っています。

## サービスの動作確認

上記のサービス群が仕様どおりに動くかを、実際に動いているシミュレータへ接続して自動検証する
適合性テストを用意しています。とくに `reset_simulation` の前後で状態が初期化され、
リセット後も指令を受け付け続けるかを確認できます。

```bash
# Unity_ROS2_sample のコンテナ内で
cd ~/colcon_ws && ./scripts/service_conformance_test.sh
```

詳細は [docs/Service-Conformance-Test-ja.md](docs/Service-Conformance-Test-ja.md) を参照してください。

## URDF の拡張要素

摩擦係数は URDF の独自要素 `<collision_material>` で設定します。書き方、`combine` の
落とし穴、速度を上げたときに滑る理由と実測値は
[docs/URDF-Collision-Material-ja.md](docs/URDF-Collision-Material-ja.md) にまとめてあります。

GNSS 受信機は他のセンサ (imu, lidar, camera, contact など) と同様に `<simulation>` 内で
宣言します:

```xml
<simulation>
  <sensor type="gnss" name="gnss_link">
    <update_rate>10</update_rate>
    <origin_latitude>35.681236</origin_latitude>   <!-- 省略可; 度 -->
    <origin_longitude>139.767125</origin_longitude><!-- 省略可; 度 -->
    <origin_altitude>0.0</origin_altitude>         <!-- 省略可; メートル -->
  </sensor>
</simulation>
```

`origin_*` 要素は Unity ワールド原点を地球上のどこに置くかを指定します。測地原点はシーン
全体で 1 つを全ロボットで共有し、GNSS センサを宣言して最初にスポーンしたロボットの指定が
優先されます (後から別の原点を指定すると警告を出して既存の原点を使います)。測位結果は
`sensor_msgs/NavSatFix` として `/<ロボット名>/<リンク名>/fix` に配信されます。

## 既知の制約

意図的に保留にしている項目と、設計として受け入れている制約は
[docs/Known-Limitations-ja.md](docs/Known-Limitations-ja.md) にまとめてあります。
「なぜそうなっているのか」と「着手するなら何を決める必要があるか」を書いてあるので、
気になる挙動に当たったらまずここを見てください。

## ライセンス
このプロジェクトは Apache 2.0 ライセンスの下で提供されています - 詳細はライセンスファイルをご覧ください。

## 謝辞

本プロジェクトの開発にあたり、以下のオープンソースプロジェクトに感謝します。

* **[dbrizov/NaughtyWaterBuoyancy](https://github.com/dbrizov/NaughtyWaterBuoyancy)**：オブジェクトの**浮力計算**に利用しています。 
* **[MARUSimulator/marus-core](https://github.com/MARUSimulator/marus-core)**：浮力以外の**水力学パラメータを適用する設計・実装方法**の参考にさせていただきました。
* **[gasgiant/Aircraft-Physics](https://github.com/gasgiant/Aircraft-Physics)**：**空力学パラメータを適用する設計・実装方法**の参考にさせていただきました。
