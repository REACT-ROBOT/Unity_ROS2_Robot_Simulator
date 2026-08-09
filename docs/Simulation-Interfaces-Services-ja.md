# simulation_interfaces サービス一覧

このシミュレータが公開している [simulation_interfaces](https://github.com/ros-simulation/simulation_interfaces)
**2.1.0** のサービスと、その解釈です。実装状況は `get_simulator_features` の申告と必ず一致させて
あります (申告と実装がずれていないかは適合性テストの G1 が検査します)。

```bash
ros2 service call /get_simulator_features simulation_interfaces/srv/GetSimulatorFeatures "{}"
```

## 対応状況

srv で定義されている 22 サービスすべてに対応しています。

| 分類 | サービス | 備考 |
|---|---|---|
| スポーン | `spawn_entity` / `spawn_entities` | `spawn_entity` は 2.0.0 で deprecated |
| | `delete_entity` | 1 体だけデスポーンする |
| | `get_spawnables` | 設定ファイルの `spawnable_paths` を走査 |
| エンティティ | `get_entities` / `get_entities_states` | 名前・カテゴリ・タグ・bounds で絞り込み |
| | `get_entity_state` / `set_entity_state` | 姿勢と速度 |
| | `get_entity_info` / `set_entity_info` | カテゴリ・説明・タグ |
| | `get_entity_bounds` | 基準リンク座標系の AABB |
| 名前付き姿勢 | `get_named_poses` / `get_named_pose_bounds` | 設定ファイルから読む |
| シミュレーション | `get_simulation_state` / `set_simulation_state` | |
| | `reset_simulation` | SCOPE_TIME / STATE / SPAWNED |
| | `step_simulation` | 一時停止中のみ |
| | `get_simulator_features` | |
| ワールド | `load_world` / `unload_world` | シーン JSON |
| | `get_current_world` / `get_available_worlds` | タグで絞り込める |

### 対応していないもの

- **`SimulateSteps` アクション** — ROS-TCP-Connector にアクションを実装する口が無く、
  シミュレータ側だけでは足せません。多ステップ実行は `step_simulation` を使ってください
  (`STEP_SIMULATION_MULTIPLE` は申告していますが `STEP_SIMULATION_ACTION` は申告していません)。
- **`SPAWNING_RESOURCE_STRING`** — URDF の mesh 参照は URDF ファイルからの相対パスで解決するため、
  文字列だけ受け取ってもアセットを見つけられません。`entity_resource.uri` を使ってください。
  なお **ワールド** の `resource_string` には対応しています (シーン JSON はメッシュを
  絶対パスで指すため、文字列だけでも成立するからです)。
- **`ENTITY_BOUNDS_CONVEX`** — bounds による絞り込みは `TYPE_BOX` と `TYPE_SPHERE` のみ。
  凸包を渡すと `RESULT_FEATURE_UNSUPPORTED` を返します。
- **`EntityState.acceleration`** — 常にゼロを返し、`set_entity_state` でも無視します。
  Unity には加速度を直接与える入口が無く、`EntityState.msg` も「シミュレータによっては
  無視される」と明記している項目です。要求された場合は結果を `RESULT_OK` にしたまま
  `error_message` に無視した旨を残します。

## 「エンティティ」の範囲

エンティティは **`spawn_entity` / `spawn_entities` で生成したものだけ**です。UI から置いた
床や障害物、ライトはワールド側の景観として扱い、`get_entities` にも出てきません。

こうしないと、`delete_entity` や `set_entity_state` の対象が「ROS から作ったもの」と
「GUI で置いたもの」で二重になり、`load_world` が景観を丸ごと差し替えるという整理と
噛み合わなくなります。

カテゴリの既定値は、URDF から作ったものが `CATEGORY_ROBOT`、メッシュ単体が
`CATEGORY_OBJECT` です。`set_entity_info` でいつでも上書きできます。

### 座標系

姿勢・速度・bounds はすべて ROS (右手系・Z 上・m) で受け渡しします。Unity 内部の
左手系・Y 上との読み替えは `/ground_truth` の publish と同じ規則です。

`get_entity_state` が返す姿勢は、URDF ロボットの場合 **ベースリンク**のものです。
ルートの GameObject は ArticulationBody のソルバに追随しないため、そちらを読むと
スポーン位置から動かないように見えます。

### 名前フィルタ

`EntityFilters.filter` は名前**全体**との一致を見ます (simulation_interfaces の参照実装が
`std::regex_match` を使うのに合わせています)。部分一致させたい場合は `.*foo.*` のように
書いてください。仕様上は POSIX 拡張正規表現ですが、.NET の正規表現エンジンで評価するため、
素直な表現はそのまま通ります。

## ワールドはシーン JSON

このシミュレータの「ワールド」は、GUI の Save Scene が書き出すシーン JSON
(`SavedSceneData` 形式) です。Unity のシーンを切り替える方式にしなかったのは、
それだとワールドを増やすたびにプレイヤーのビルドが必要になるためです。JSON なら
ランタイムで完結し、GUI で組んだ景観をそのまま `load_world` で配れます。

地面や既定のライトといった**組み込みシーンの土台は常に在り**、ワールドはその上に載る
景観として扱います。起動直後は組み込みシーンを「ロード済みのワールド」と見なすので、
起動直後が `STATE_STOPPED` であるという従来の挙動は変わりません。

```bash
# 景観を差し替える (エンティティは全部消え、停止状態に戻り、時刻もリセットされる)
ros2 service call /load_world simulation_interfaces/srv/LoadWorld \
  "{ world_resource: { uri: 'file:///home/user/worlds/warehouse.json', resource_string: '' },
     fail_on_unsupported_element: false, ignore_missing_or_unsupported_assets: false }"

# 今のワールドを見る
ros2 service call /get_current_world simulation_interfaces/srv/GetCurrentWorld "{}"

# 降ろす
ros2 service call /unload_world simulation_interfaces/srv/UnloadWorld "{}"
```

`unload_world` の後は `STATE_NO_WORLD` になります。この状態では

- `set_simulation_state` は `STATE_QUITTING` 以外を `INCORRECT_TRANSITION` で断る
- `spawn_entity` / `spawn_entities` / `reset_simulation` / `step_simulation` は
  `RESULT_INCORRECT_STATE` を返す

ようにしてあります。`load_world` を呼べば元に戻ります。ただし**組み込みシーンは
ファイルとして取り出せない**ので、`get_current_world` が返す `world_resource.uri` は
起動直後は空です。降ろしたあとに同じものへ戻すことはできません。

### ワールドの名前・説明・タグ

シーン JSON の先頭に書けます。3 つとも省略可で、省略すると名前はファイル名、説明は
オブジェクト数、タグは空になります。メタデータをワールドファイル自身に持たせているので、
ファイルを配るだけでタグまで一緒に運べます (別置きの索引を同期する必要がありません)。

```json
{
  "name": "warehouse",
  "description": "棚とパレットのある屋内環境",
  "tags": ["indoor", "warehouse"],
  "objects": [ ... ]
}
```

ここに書いた内容が `get_available_worlds` の一覧にも `get_current_world` にも載ります。
GUI の Save Scene は、読み込んだワールドのメタデータを引き継いで書き戻すので、
開いて保存し直してもタグは消えません。

`get_available_worlds` の `filter` でタグ絞り込みができます (`WORLD_TAGS`)。

```bash
# indoor か outdoor のどちらかを持つワールド (FILTER_MODE_ANY)
ros2 service call /get_available_worlds simulation_interfaces/srv/GetAvailableWorlds \
  "{ filter: { tags: ['indoor', 'outdoor'], filter_mode: 0 } }"

# indoor と warehouse を両方持つワールド (FILTER_MODE_ALL)
ros2 service call /get_available_worlds simulation_interfaces/srv/GetAvailableWorlds \
  "{ filter: { tags: ['indoor', 'warehouse'], filter_mode: 1 } }"
```

- タグを指定すると、**タグを持たないワールドは外れます**。
- 一致するものが無ければ空リストを `RESULT_OK` で返します。見つからないことは
  エラーではありません。
- `filter_mode` が `0` / `1` 以外なら `RESULT_OPERATION_FAILED` です。黙って `ANY` として
  扱うと、要求とは違う絞り込み結果を「正常」として返してしまうためです。同じ理由で、
  `get_entities` / `get_entities_states` / `get_named_poses` のタグ絞り込みも未知の
  `filter_mode` を弾きます。

### 読み込み時の結果コード

`load_world` は消す前に JSON の構文を確かめます。読めないものを渡したときに景観だけ
消えて何も残らない、という壊れ方をしないためです。

| 状況 | 結果コード |
|---|---|
| `uri` が file 以外 / 拡張子が `.json` でない | `UNSUPPORTED_FORMAT` (101) |
| `uri` も `resource_string` も空 | `NO_RESOURCE` (102) |
| JSON として読めない | `RESOURCE_PARSE_ERROR` (103) |
| ファイルが無い / メッシュを読み込めない | `MISSING_ASSETS` (104) |
| 知らない `type` の要素がある | `UNSUPPORTED_ELEMENTS` (106) |

`ignore_missing_or_unsupported_assets` / `fail_on_unsupported_element` で、それぞれ
`MISSING_ASSETS` / `UNSUPPORTED_ELEMENTS` を無視させられます。読み飛ばした要素は
どちらの場合も `error_message` に残ります。

## step_simulation

一時停止中に限り、指定ステップだけ進めてまた止めます。

```bash
ros2 service call /set_simulation_state simulation_interfaces/srv/SetSimulationState "{ state: { state: 2 } }"
ros2 service call /step_simulation simulation_interfaces/srv/StepSimulation "{ steps: 10 }"
```

- 一時停止していないときは `RESULT_OPERATION_FAILED` を返します (仕様どおり)。
- 進み終わるまで応答を返しません。1 回の呼び出しで進められるのは 100000 ステップまでで、
  超えると `RESULT_OPERATION_FAILED` です。桁を間違えた要求でシミュレータが何時間も
  戻ってこなくなるのを防ぐための上限です。
- 進めている途中に `set_simulation_state` / `reset_simulation` / 画面のボタンが入ると
  打ち切り、`RESULT_OPERATION_FAILED` を返します。

`Physics.Simulate()` ではなく `Time.timeScale` を戻して回しているのは、前者だと
`FixedUpdate` が呼ばれず、`ServoJointModel` や `JointStateSub` といった制御側が
動かないまま物理だけ進んでしまうためです。

## simulation_resources.json

`get_spawnables` / `get_named_poses` / `get_named_pose_bounds` / `get_available_worlds` が
返す中身は、この設定ファイルから来ます。Unity のプレイヤーには ament のパッケージ検索パスに
相当する仕組みが無いので、「どこを見えていることにするか」を明示的に与える必要があります。

探索順は次のとおりで、どれも無ければ空リストを `RESULT_OK` で返します
(設定なしで起動できることを優先しているので、ファイルが無いこと自体はエラーにしません)。

1. 環境変数 `SIMULATION_RESOURCES_CONFIG` が指すパス
2. プレイヤー実行ファイルと同じディレクトリの `simulation_resources.json`
   (エディタではプロジェクトルート)
3. `Application.persistentDataPath/simulation_resources.json`

```json
{
  "spawnable_paths": [
    "/root/colcon_ws/install/diffbot_description/share",
    "/root/models"
  ],
  "world_paths": [
    "/root/worlds"
  ],
  "named_poses": [
    {
      "name": "charger",
      "description": "充電ステーションの手前",
      "tags": ["spawn", "parking"],
      "position": [1.0, 2.0, 0.0],
      "rpy": [0.0, 0.0, 1.5708],
      "bounds": {
        "type": "box",
        "min": [-0.3, -0.3, 0.0],
        "max": [0.3, 0.3, 0.5]
      }
    },
    {
      "name": "gate",
      "position": [-4.0, 0.0, 0.0],
      "orientation": [0.0, 0.0, 0.0, 1.0],
      "bounds": { "type": "sphere", "center": [0.0, 0.0, 0.5], "radius": 1.0 }
    }
  ]
}
```

- 座標はすべて ROS (右手系・Z 上・m・rad)。
- 姿勢は `orientation` (クォータニオン `[x, y, z, w]`) か `rpy` (`[roll, pitch, yaw]`) の
  どちらかで書きます。両方あれば `orientation` を優先します。
- `bounds` は省略可 (`TYPE_EMPTY` になります)。`type` は `box` か `sphere` のみです。
- `spawnable_paths` は再帰的に走査し、`.urdf` `.obj` `.stl` `.dae` `.fbx` `.ply` を拾います。
- `world_paths` からは `.json` のうち**シーンとして読めるもの**だけを候補にします
  (設定ファイルなど無関係な JSON が混ざるためです)。
- 走査は合計 2000 ファイルで打ち切ります。打ち切ったときは黙って減らさず
  `error_message` に出します。

`get_spawnables` の `sources` と `get_available_worlds` の `additional_sources` に、
その場限りの探索先を足せます。存在しないものを渡しても失敗はせず、`error_message` に
「どれが読めなかったか」が入ります (`get_available_worlds` は `continue_on_error` を
true にしない限り `DEFAULT_SOURCES_FAILED` を返します)。

## 使用例

```bash
# 今いるエンティティを列挙する
ros2 service call /get_entities simulation_interfaces/srv/GetEntities "{ filters: { filter: '' } }"

# ロボットだけ、状態つきで取る
ros2 service call /get_entities_states simulation_interfaces/srv/GetEntitiesStates \
  "{ filters: { categories: [{ category: 1 }] } }"

# 半径 3 m の球に重なるものだけ取る
ros2 service call /get_entities simulation_interfaces/srv/GetEntities \
  "{ filters: { bounds: { type: 3, points: [{x: 0.0, y: 0.0, z: 0.0}, {x: 3.0, y: 0.0, z: 0.0}] } } }"

# 姿勢だけ書き換える (速度・加速度は触らない)
ros2 service call /set_entity_state simulation_interfaces/srv/SetEntityState \
  "{ entity: 'diffbot', set_pose: true, set_twist: false, set_acceleration: false,
     state: { pose: { position: {x: 1.0, y: 0.0, z: 0.0},
                      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0} } } }"

# タグを付ける
ros2 service call /set_entity_info simulation_interfaces/srv/SetEntityInfo \
  "{ entity: 'diffbot', info: { category: { category: 1 }, description: '2 輪台車', tags: ['agv'] } }"

# 1 体だけ消す
ros2 service call /delete_entity simulation_interfaces/srv/DeleteEntity "{ entity: 'diffbot' }"
```
