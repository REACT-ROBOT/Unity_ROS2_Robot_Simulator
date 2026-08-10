# ROS2 サービス適合性テスト

シミュレータが `simulation_interfaces` のサービスとして公開している機能 (`spawn_entity` /
`set_simulation_state` / `get_simulation_state` / `reset_simulation` / `step_simulation`) が
仕様どおりに動くかを、実際に動いているシミュレータへ接続して自動検証する仕組みです。

とくに **「reset_simulation を呼んだあとロボットが指令を受け付けなくなる」** 種類の不具合を
再現・切り分けできることを狙って作ってあります。

テスト本体は [Unity_ROS2_sample](https://github.com/hijimasa/Unity_ROS2_sample) 側の
`colcon_ws/src/simulation_service_tests` にあります。シミュレータ本体はビルド済みバイナリを
そのまま使うため、このリポジトリ側に手を入れずに回せます。

## 実行方法

Unity_ROS2_sample のコンテナの **中** で実行します。ROS 2 Humble / Jazzy のどちらでも
そのまま動きます (スクリプトは決め打ちせず `${ROS_DISTRO}` を見ます)。

```bash
cd ~/colcon_ws
colcon build --packages-select simulation_service_tests simulation_ros2_utils
source install/setup.bash

./scripts/service_conformance_test.sh                       # 既定 (diffbot)
./scripts/service_conformance_test.sh --profile servo_demo  # 軽い構成
```

スクリプトが ROS-TCP-Endpoint とシミュレータの起動、テストの実行、後始末までまとめて行います。
既に立ち上がっているシミュレータへ相乗りしたいときは `--no-sim` を付けます。

> **注意**: `colcon_ws` を humble と jazzy で使い回すときは、先に `rm -rf build install log`
> してから `colcon build` すること。Python のバージョン (3.10 / 3.12) が違うため、
> 前の distro の成果物が残っていると
> `UnsupportedTypeSupport: Could not import 'rosidl_typesupport_c'` でテストが起動しません。

### 自分の変更を検証する

テストは動いているプレイヤーへ接続するので、このリポジトリを直したときは
プレイヤーを作り直してから流します。

```bash
# シミュレータ側 (このリポジトリ) で Linux プレイヤーをビルド
~/Unity/Hub/Editor/6000.2.7f2/Editor/Unity -batchmode -nographics -quit \
  -projectPath . -executeMethod BuildLinuxPlayer.Build \
  -buildOutput /tmp/simbuild/Unity_ROS2_Robot_Simulator.x86_64 -logFile /tmp/build.log

# できたものをコンテナへ渡してテスト (docker run に -v /tmp/simbuild:/home/unity/simbuild:ro)
./scripts/service_conformance_test.sh --sim-dir ~/simbuild
```

終了コードは `0` = すべて期待どおり、`1` = 不具合を検出、`2` = 実行できなかった、です。
CI に組み込む場合は `--junit PATH` で JUnit XML を出力できます。

出力先 (既定 `/tmp/service_conformance/`):

| ファイル | 内容 |
|---|---|
| `<profile>.console.log` | 実行ログ |
| `<profile>.junit.xml` | JUnit XML (CI 用) |
| `<profile>.report.json` | 機械可読レポート |
| `Player.log` | シミュレータ側の Unity ログ (例外の実体はここ) |

## 検証項目

| ID | 内容 |
|----|------|
| A1–A3 | 中核 7 サービスの疎通、起動直後の状態、`Result` コードの規約適合 |
| B1–B3 | 状態遷移 (start / 同一状態 / 不正値の拒否) |
| C1–C6 | スポーン、基準状態の記録、`ground_truth`、**指令が効くことの基準取り**、**車輪の空転 (URDF の摩擦が効いているか)**、pause/resume |
| D1–D10 | **`reset_simulation` の全スコープ**。エンティティ生存、関節・姿勢の復元、リセット後の指令受付、サービス生存、デスポーン、再スポーン、時刻リセット、反復安定性 |
| E1–E2 | `STATE_STOPPED` でのデスポーンと、その後の再スポーン |
| F1–F3 | `step_simulation` の進み量 (`n` と `2n` の比が 2 になること)、空シーンへのリセット、**状態遷移が sim 時刻に届いていること** |
| G1–G6 | **simulation_interfaces 2.x**。**申告とサービス実体の突き合わせ**、`Resource` によるスポーン、`spawn_entities` (複数生成・部分失敗の報告)、`entity_namespace` によるトピック分離、**`resource_string` からの生成** |
| H1–H9 | **任意サービス**。**加速度が重力を捉えること**、`get_entities` / `get_entity_state` と `ground_truth` の一致、`set_entity_state`、`entity_info`、`get_entity_bounds`、`EntityFilters`、`delete_entity`、`get_spawnables` / 名前付き姿勢、world のライフサイクル、**world のタグ絞り込み** |

| I1–I2 | **`simulate_steps` アクション**。1 ステップごとの feedback、途中キャンセル |

★ 印 (D5 / D8 / D10) が報告されている不具合の直接検証にあたります。

判定は `PASS` / `FAIL` / `KNOWN_GAP` (未実装と分かっている項目、既定では終了コードに数えない) /
`SKIP` / `ERROR` の 5 種類です。

## 検出された不具合と修正

リリース版バイナリ v0.9.3 に対して流したときの結果です。

```
PASS 19  FAIL 4  KNOWN_GAP 3  SKIP 0  ERROR 0   (diffbot)
PASS 19  FAIL 3  KNOWN_GAP 3  SKIP 1  ERROR 0   (servo_demo)
```

以下の修正と simulation_interfaces 2.1.0 対応を入れた現在の HEAD では、両プロファイルとも
全項目が通ります (`servo_demo` の 1 SKIP は固定台ロボットでは意味を持たない C5)。

```
PASS 31  FAIL 0  KNOWN_GAP 0  SKIP 0  ERROR 0   (diffbot)
PASS 30  FAIL 0  KNOWN_GAP 0  SKIP 1  ERROR 0   (servo_demo)
```

同じ結果を ROS 2 Humble (Ubuntu 22.04) と Jazzy (Ubuntu 24.04) の両方で確認しています。

残り 15 サービスと `WORLD_TAGS`、`SimulateSteps` アクションを実装して H 群 (9 シナリオ) と
I 群 (2 シナリオ) を足した後は、次のようになります。

```
PASS 46  FAIL 0  KNOWN_GAP 0  SKIP 0  ERROR 0   (diffbot)
PASS 43  FAIL 0  KNOWN_GAP 0  SKIP 3  ERROR 0   (servo_demo)
```

ROS 2 Jazzy (Ubuntu 24.04) と Humble (Ubuntu 22.04) の両方で確認しています。

`simulate_steps` の feedback は、届く件数が要求ステップ数より少ないことがあります。
終了直前に出した feedback は結果 (サービス応答) に追い越されることがあり、rclpy の
クライアントは結果を受け取った時点でその goal の feedback 購読を畳むためです。
実際に何ステップ進んだかは sim 時刻で確かめているので、I1 は件数ではなく
「経過が逐次報告されていること」を判定します。

### FAIL → 修正済: デスポーン後に再スポーンすると指令を受け付けない (D8 / D10 / E2)

報告されていた不具合はここで再現しました。

```
FAIL  D8   ★デスポーン後に再スポーンして指令を受け付ける
      left_wheel_joint: cmd_vel=+3.000 obs_vel=-0.000 dpos=+0.000 -> NG
      right_wheel_joint: cmd_vel=+3.000 obs_vel=-0.000 dpos=+0.000 -> NG
FAIL  D10  ★リセットを繰り返しても指令受付が壊れない
      1 / 3 巡目で指令が効かなくなった
```

`Player.log` に原因が出ています。

```
Publisher for topic /diffbot/joint_states registered twice!
NullReferenceException: Object reference not set to an instance of an object.
  at UnityEngine.ArticulationBody.get_xDrive ()
  at JointStateSub.Callback (RosMessageTypes.Sensor.JointStateMsg msg)
  at Unity.Robotics.ROSTCPConnector.RosTopicState.OnMessageReceived (System.Byte[] data)
```

`SimulationControl.DespawnAllEntities()` は `GameObject.Destroy()` するだけで、
`JointStateSub.Start()` が `ROSConnection.Subscribe()` で登録した購読を解除していません。
`RosTopicState` はコールバックを `List` に貯め込み `List.ForEach` で回すため、

1. 破棄済みの `JointStateSub` のコールバックがリストに残る
2. 再スポーンすると、その **後ろに** 新しいコールバックが積まれる
3. 指令が届くと古いコールバックが先に呼ばれ、破棄済み `ArticulationBody` の
   `xDrive` 参照で `NullReferenceException` を投げる
4. `ForEach` がそこで中断し、**新しいロボットのコールバックまで到達しない**

という経路で「リセット後に指令を受け付けなくなる」状態になっていました。

**修正**: `JointStateSub` に `DetachFromRos()` を追加し、`DespawnAllEntities()` が
`Destroy()` の **前に** 呼ぶようにしました。`ROSConnection.Unsubscribe()` で購読を解除し、
かつ解除がエンドポイントへ届くまでの間に来るメッセージはフラグで弾きます。
同じ購読の作りをしていた `LinkThruster` にも同じ処理を入れています。

`OnDestroy()` 任せにしていないのは、Unity の `Destroy()` がフレーム終端まで遅延され、
破棄済みコールバックが 1 フレーム分生き残るためです。デスポーン側から明示的に呼ぶことで
順序を確定させています。

> **エンドポイント側の要件**: `Unsubscribe()` は `__remove_subscriber` sys コマンドを
> ROS-TCP-Endpoint へ送ります。本家 (Unity-Technologies) の v0.7.0 はこのコマンドを
> 実装しておらず、受け取ると `handle_syscommand` の `getattr` が `AttributeError` を投げて
> **TCP 接続ごと落ちます** (検証済み: 全サービスが無応答になります)。
> [hijimasa/ROS-TCP-Endpoint](https://github.com/hijimasa/ROS-TCP-Endpoint) では
> `__remove_subscriber` を含む 4 種類の unregister コマンドを実装し、あわせて未知コマンドや
> 壊れたペイロードで接続が落ちないようにしてあります。本家のエンドポイントと組み合わせて
> 使うことはできません。

`JointStatePub` / `GroundTruthPub` の `registered twice!` 警告は、publisher の登録解除 API が
`ROSConnection` に無いことによるものです。`RegisterPublisher` は二重登録時に早期 return する
だけで publish 自体は動き続けるため実害はありませんが、ログが警告で埋まって本物の異常が
埋もれるので、登録前に `GetTopic(...).IsPublisher` を見て重複登録を避けるようにしました。
publisher 側の登録解除は下の「publisher の登録解除」で対応しています。

### FAIL → 修正済: SCOPE_STATE でルート姿勢が戻りきらない (D4, diffbot のみ)

```
FAIL  D4   SCOPE_STATE でルート姿勢がスポーン位置に戻る
      基準 (-0.000, -0.000, +0.000) -> 現在 (+0.080, +0.001, -0.000) 差 0.080 m
```

`ResetAllEntitiesState()` は `TeleportRoot()` で位置だけ戻し、関節速度を戻していませんでした。
車輪が回ったままテレポートするので、直後からまた走り出して 0.08 m ずれます
(テスト側が「リセット直後の関節速度 +2.9 rad/s」を併記するため、位置を戻していないのではなく
戻した直後に動き出していることが読み取れます)。下の D3 と同じ原因です。

**修正**: D3 と同じ処理でまとめて解決しました。加えて `TeleportRoot()` の後にルートの
`linearVelocity` / `angularVelocity` もゼロにしています。

### KNOWN_GAP → 修正済: SCOPE_STATE で関節状態が戻らない (D3)

```
KNOWN_GAP  D3  最大ずれ right_wheel_joint で 26.110 rad
```

`ResetAllEntitiesState()` はルートの `transform` と `TeleportRoot()` だけで、
配下の `ArticulationBody` の `jointPosition` / `jointVelocity` / `jointForce` を初期化して
いませんでした。

**修正**: `SpawnEntity()` がスポーン時に行っていたゼロ化を `ResetArticulationState()` として
切り出し、スポーンとリセットの両方から呼ぶようにしました。あわせて

- `xDrive.target` / `targetVelocity` も 0 に戻す
  (残すと関節をゼロにした次の物理ステップで最後の指令位置へ飛び戻る)
- `ServoJointModel.ResetState()` でロータ角・伝達ばねのたわみ・指令値を初期化する
  (残すと巻き上がったトルクがリセット直後に解放されて関節が跳ねる)

関節を戻してからルートをテレポートする順序にしてあります。逆にすると、その 1 ステップ分だけ
関節が動いた状態でソルバが回ります。

### KNOWN_GAP → 修正済: SCOPE_TIME が未実装 (D9)

`SimulationControl.ResetSimulation()` の `SCOPE_TIME` 分岐が `// TODO: Reset Time` のままでした。

**修正**: `Time.timeAsDouble` は巻き戻せないため、`Clock` に原点 (`s_TimeOrigin`) を持たせ、
`Clock.ResetTime()` がそれを現在時刻へ進める方式にしました。公開される sim 時刻は
すべてこの原点を差し引いた値になります。

### KNOWN_GAP → 修正済: Result コードの規約 (A3)

`Result.msg` では成功が `RESULT_OK = 1`、`0` は `RESULT_FEATURE_UNSUPPORTED` です。
どのサービスも成功時に既定値の `0` を返していました。

**修正**: `get_simulation_state` / `set_simulation_state` / `reset_simulation` / `spawn_entity`
が成功時に `RESULT_OK` を返すようにしました。クライアント側 (`simulation_ros2_utils`) は
古いシミュレータとも繋がるよう `0` と `1` の両方を成功として受けます。

## publisher の登録解除

購読側 (`__remove_subscriber`) と同じく、publisher 側もデスポーン時に解除するようにしました。
修正前は、消えたロボットのトピックが `ros2 topic list` に残り続けていました。

```
                            修正前の despawn 後      修正後の despawn 後
/diffbot/joint_command      (消える)                 (消える)
/diffbot/joint_states       残る                     消える
/diffbot/lidar_link/scan    残る                     消える
/diffbot/camera_link/...    残る                     消える
/ground_truth               残る                     消える
```

3 か所に手を入れています。

**1. ROS-TCP-Connector に公開 API を追加**

`InternalAPI.SendPublisherUnregistration()` は元からありましたが、どこからも呼ばれておらず、
`ROSConnection` にも `RosTopicState` にも公開の入口がありませんでした (購読側の
`Unsubscribe()` に相当するものが無い)。`RosTopicState.UnregisterPublisher()` と
`ROSConnection.UnregisterPublisher(topic)` を追加しています。

**2. ROS-TCP-Endpoint に `__remove_publisher` を実装**

`__remove_subscriber` と同じ 4 種類の unregister コマンドをまとめて実装しました。

**3. シミュレータ側でトピックを Entity に紐づけて管理**

センサ系の publisher は UnitySensors のクラスなので解除 API を持たせられません。かわりに
`SimulationControl` がスポーン時に登録したトピック名を Entity ごとに控え
(`TrackPublishedTopic`)、デスポーン時に解除します (`ReleasePublishedTopics`)。
`/tf` と `/ground_truth` は複数ロボットが同じ名前へ publish するため参照カウントを持ち、
最後の 1 台が消えたときだけ解除します。

デスポーン時はまず `entity.SetActive(false)` で配下の `Update()` を止めてから解除します。
これを飛ばすと、UnitySensors の `RosMsgPublisher.Update()` が持つ「未登録なら登録する」
遅延登録が、解除した直後に同じフレームで登録し直してしまいます
(`Destroy()` はフレーム終端まで遅延されるため、コンポーネントはまだ生きています)。

## simulation_interfaces 2.1.0 対応

1.0.0 から 2.1.0 へ更新しました。破壊的変更があるため、2.0.0 より前のクライアントとは
互換性がありません。

### インターフェース側の変更

| 変更 | 内容 |
|---|---|
| `Resource` メッセージの導入 | `SpawnEntity` の `uri` / `resource_string` が `entity_resource` にまとめられた (2.0.0, 破壊的) |
| `SpawnEntities` の追加 | 複数体を一度に生成する。`SpawnEntity` は deprecated |
| `SpawnResult` の追加 | 個々のスポーン結果 (101-109 の詳細コード付き) |
| world 系サービス | `LoadWorld` / `UnloadWorld` / `GetCurrentWorld` / `GetAvailableWorlds` |
| `SimulationState` | `STATE_NO_WORLD` (4) と `STATE_LOADING_WORLD` (5) を追加 |
| `SetEntityState` | `set_pose` / `set_twist` / `set_acceleration` フラグを追加 |

### シミュレータ側の対応

- **C# メッセージの再生成**: `Assets/Editor/GenerateRosMessages.cs` を追加しました。
  エディタのメニューを手で辿らずに、定義から `Assets/RosMessages` 以下を作り直せます。

  ```bash
  Unity -batchmode -nographics -quit -projectPath . \
    -executeMethod GenerateRosMessages.Generate \
    -messageInput ../Unity_ROS2_sample/colcon_ws/src/simulation_interfaces
  ```

- **`spawn_entity`**: `entity_resource.uri` から読むよう変更。あわせて結果コードを仕様に
  合わせました (リソース無し → `NO_RESOURCE`、ファイルが無い → `MISSING_ASSETS`、
  `resource_string` 指定 → `UNSUPPORTED_FORMAT`)。
- **`spawn_entities`**: 実装しました。1 件でも失敗すると全体は `ENTITIES_SPAWN_FAILED` になり、
  個々の成否は `results[i]` に入ります。途中で失敗しても残りの要求は続行します。
- **エンティティ名の一意性**: 従来は要求名を無視して URDF の robot 名をそのまま使っていたため、
  同じ URDF から 2 体出すと両方が同じ名前・同じトピックになっていました。仕様どおり
  「要求名が空ならリソース側の名前」を使い、衝突時は `allow_renaming` が false なら
  `NAME_NOT_UNIQUE`、true なら連番を付けて一意化します。
- **`entity_namespace`**: 実装しました。URDF の `ros2_control` に書かれたトピック名は
  リソース側で固定なので、同じ URDF から複数体出すときはこれが無いと衝突します。
- **`get_simulator_features`**: 実装しました。**実装しているものだけ**を申告します
  (G1 が未実装機能の申告を失敗として検出します)。

### 対応していないもの

`resource_string` からの生成 (`SPAWNING_RESOURCE_STRING`) は未対応です — URDF の mesh 参照は
URDF ファイルからの相対パスで解決するため、文字列だけ受け取ってもアセットを見つけられません。

> **その後の変更**: この節に挙げていた world 系 4 サービス
> (`LoadWorld` / `UnloadWorld` / `GetCurrentWorld` / `GetAvailableWorlds`) と `step_simulation` は
> 実装済みです。あわせてエンティティ系 (`GetEntities` / `GetEntityState` / `SetEntityState` /
> `DeleteEntity` など) と `GetSpawnables` / 名前付き姿勢も入れたので、srv で定義されている
> 22 サービスすべてに対応しています。解釈と設定方法は
> [docs/Simulation-Interfaces-Services-ja.md](Simulation-Interfaces-Services-ja.md) を参照してください。
>
> テスト側 (Unity_ROS2_sample) も追従済みです。**F1** は進み量まで測るようになり
> (`n` ステップと `2n` ステップの比が 2 になること)、**G1** は「未実装のはず」の名前を
> ベタ書きするのをやめて、申告と ROS グラフ上のサービスを機械的に突き合わせるように
> なりました。新しく実装した 15 サービスは **H1–H9** が検証します。

## テストを読み書きするときの前提

シミュレータ側の性質のうち、テストの組み方に効くもの:

- **停止中はトピックが完全に止まる。** `Time.timeScale = 0` の間 Unity の `FixedUpdate` は
  回らないため、`STOPPED` / `PAUSED` では `joint_states` も `ground_truth` も publish されません。
  「エンティティが生きているか」をトピックの有無で判定するときは、必ず `PLAYING` にしてから
  観測します。デスポーン判定 (D7 / E1) が `play()` を挟んでいるのはこのためです。
- **`joint_states` のタイムスタンプは sim 時刻。** 一時停止中は進まないのでタイムアウト計算には
  使えません。ハーネスは一貫して壁時計 (`time.monotonic()`) を使います。
- **指令は送り続ける必要がある。** `JointStateSub` は受信値を `xDrive.target` /
  `targetVelocity` へ写すだけなので、ros2_control 相当の周期送信をテスト側で行っています。
- **ros2_control を経由しない。** `joint_command` へ直接 publish し `joint_states` を直接読みます。
  controller_manager やスポナーを挟まないぶん、失敗の原因がシミュレータ側だと切り分けやすく
  なります。

## 手作業で再現したいとき

```bash
# ロボットをスポーンして動かす
ros2 launch unity_diffbot_sim diffbot_spawn.launch.py

# リセットする (scope: default | time | state | spawned | all)
ros2 run simulation_ros2_utils reset_simulation --ros-args -p scope:=state

# 再開して指令が通るか見る
ros2 run simulation_ros2_utils set_sim_state --ros-args -p set_state:=start
```
