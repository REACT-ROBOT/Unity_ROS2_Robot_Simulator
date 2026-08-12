# simulation_interfaces サービス一覧

このシミュレータが公開している [simulation_interfaces](https://github.com/ros-simulation/simulation_interfaces)
**2.1.0** のサービスと、その解釈です。実装状況は `get_simulator_features` の申告と必ず一致させて
あります (申告と実装がずれていないかは適合性テストの G1 が検査します)。

```bash
ros2 service call /get_simulator_features simulation_interfaces/srv/GetSimulatorFeatures "{}"
```

## 対応状況

srv で定義されている 22 サービスと `SimulateSteps` アクション、および
`SimulatorFeatures` に並ぶ機能のすべてに対応しています。

| 分類 | サービス | 備考 |
|---|---|---|
| スポーン | `spawn_entity` / `spawn_entities` | `uri` と `resource_string` の両方。`spawn_entity` は 2.0.0 で deprecated |
| | `delete_entity` | 1 体だけデスポーンする |
| | `get_spawnables` | 設定ファイルの `spawnable_paths` を走査 |
| エンティティ | `get_entities` / `get_entities_states` | 名前・カテゴリ・タグ・bounds (box / 球 / 凸包) で絞り込み |
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
| アクション | `simulate_steps` | 一時停止中のみ。1 ステップごとに feedback、途中キャンセル可 |

### 対応していないもの

srv で定義されているサービスと `SimulatorFeatures` の機能はすべて実装しています。
- **`set_entity_state` の `acceleration`** — 無視します (取得はできます)。加速度を直接
  与える入口が Unity に無く、力へ読み替えるには質量の扱いを決める必要があるためです。
  `EntityState.msg` が「シミュレータによっては無視される」と明記している項目なので、
  要求された場合は結果を `RESULT_OK` にしたまま `error_message` に無視した旨を残します。

## 「エンティティ」の範囲

エンティティは **スポーン経路で生成したものだけ**です — `spawn_entity` / `spawn_entities`
サービス、および同じ実装を呼ぶ GUI の「Spawn Robot (URDF)」ボタン。UI から置いた
床や障害物、ライトはワールド側の景観として扱い、`get_entities` にも出てきません。

こうしないと、`delete_entity` や `set_entity_state` の対象が「ROS から作ったもの」と
「GUI で置いたもの」で二重になり、`load_world` が景観を丸ごと差し替えるという整理と
噛み合わなくなります。

カテゴリの既定値は、URDF から作ったものが `CATEGORY_ROBOT`、メッシュ単体が
`CATEGORY_OBJECT` です。`set_entity_info` でいつでも上書きできます。

### 座標系

姿勢・速度・bounds はすべて ROS (右手系・Z 上・m) で受け渡しします。Unity 内部の
左手系・Y 上との読み替えは `/ground_truth` の publish と同じ規則です。

`acceleration` は `FixedUpdate` でルートボディの速度を控えて 1 階差分を取った値です
(Unity に加速度を読む口が無いため)。平滑化はしていないので接触の瞬間などは大きく振れます。
`timeScale = 0` の間は `FixedUpdate` が回らないため、停止中は最後に計算した値が残ります。

`get_entity_state` が返す姿勢は、URDF ロボットの場合 **ベースリンク**のものです。
ルートの GameObject は ArticulationBody のソルバに追随しないため、そちらを読むと
スポーン位置から動かないように見えます。

### bounds による絞り込み

`EntityFilters.bounds` は `TYPE_BOX` / `TYPE_SPHERE` / `TYPE_CONVEX_HULL` に対応しています
(`TYPE_EMPTY` は絞り込みなし)。エンティティ側は AABB として扱い、それが bounds と
重なるものを返します。

凸包は **GJK** で判定しています。分離軸判定 (SAT) には凸包の面法線が要りますが、
`Bounds.msg` が持っているのは頂点だけで面の構成は入っていません。GJK は形状を
「向きを与えると最も遠い点を返す関数」としてしか見ないので、面を組み立てずに判定できます。
同一平面上の点しか無い凸包 (三角形や四角形) や一直線上の点でも、退化した凸集合として
そのまま扱えます。

`TYPE_CONVEX_HULL` は `Bounds.msg` の規定どおり **3 点以上**が必要で、足りなければ
`RESULT_OPERATION_FAILED` です。

### 名前フィルタ

`EntityFilters.filter` は名前**全体**との一致を見ます (simulation_interfaces の参照実装が
`std::regex_match` を使うのに合わせています)。部分一致させたい場合は `.*foo.*` のように
書いてください。仕様上は POSIX 拡張正規表現ですが、.NET の正規表現エンジンで評価するため、
素直な表現はそのまま通ります。

## ワールドはシーン JSON (または SDF のサブセット)

このシミュレータの「ワールド」は、GUI の Save Scene が書き出すシーン JSON
(`SavedSceneData` 形式) です。Unity のシーンを切り替える方式にしなかったのは、
それだとワールドを増やすたびにプレイヤーのビルドが必要になるためです。JSON なら
ランタイムで完結し、GUI で組んだ景観をそのまま `load_world` で配れます。

`load_world` は Gazebo の SDF ワールドファイル (`.sdf` / `.world`) も受け付け、
その**静的なサブセット**を同じ景観表現へ変換します — 下の
[SDF ワールド](#sdf-ワールド) を参照。

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

ようにしてあります。`load_world` を呼べば元に戻ります。

**組み込みシーンもファイルなしで戻せます。** ファイルとして取り出せないので `uri` は
空ですが、かわりに「景観が何も載っていない状態」を表すシーン JSON が
`world_resource.resource_string` に入っています。`get_current_world` で受け取った
`Resource` をそのまま `load_world` へ渡せば元に戻ります。文字列から読み込んだワールドも
同じように `resource_string` を持ち続けるので、読み直せます。

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
| `uri` が file 以外 / 拡張子が `.json` `.sdf` `.world` のどれでもない | `UNSUPPORTED_FORMAT` (101) |
| `uri` も `resource_string` も空 | `NO_RESOURCE` (102) |
| JSON として読めない (SDF なら XML に `<world>` が無い) | `RESOURCE_PARSE_ERROR` (103) |
| ファイルが無い / メッシュや `<include>` のモデルを解決できない | `MISSING_ASSETS` (104) |
| 知らない `type` の要素がある / 未対応の SDF 要素を落とした | `UNSUPPORTED_ELEMENTS` (106) |

`ignore_missing_or_unsupported_assets` / `fail_on_unsupported_element` で、それぞれ
`MISSING_ASSETS` / `UNSUPPORTED_ELEMENTS` を無視させられます。読み飛ばした要素は
どちらの場合も `error_message` に残ります。

## SDF ワールド

`load_world` (と GUI のロードボタン) は Gazebo の SDF ワールドファイルを受け付けます。
`<` で始まる `resource_string` も SDF として解釈します。読み込み時に既存のシーン JSON と
同じ景観オブジェクトへ変換され、Gazebo のシーングラフを保持するわけではありません。
対応しているもの:

- `<model>` (SDF 1.8 の入れ子モデル含む)。pose は `model → link → visual` の順で合成し、
  ROS (Z-up, 右手系) から Unity 座標系へ変換。`<visual>` の無いリンクは `<collision>` の
  形状で代用。
- geometry: `box` / `cylinder` / `sphere` / `plane` / `mesh`。メッシュは ROS/Gazebo 慣習
  (Z-up) のまま扱う: STL は URDF インポータと同じローダ (頂点単位で座標変換)、Collada は
  `up_axis` を尊重。`heightmap` や `polyline` などは未対応として報告。
- `<material>` の `diffuse` / `ambient` 色 (Gazebo の `<script>` マテリアルは無視)。
- `model://` URI の `<include>`。検索順: ワールドファイルのディレクトリ → その隣の
  `models/` → `simulation_resources.json` の `world_paths` と `spawnable_paths` →
  環境変数 `GZ_SIM_RESOURCE_PATH` / `GAZEBO_MODEL_PATH`。include 側の `<pose>` /
  `<name>` / `<static>` 上書きに対応。
- `<light>` の directional / point / spot (位置、`<direction>`、diffuse 色)。
- `<link>` の形状と `<script><trajectory>` を持つ `<actor>`: 最初の visual 形状が
  ウェイポイント補間で巡回する**動く障害物**になります (`<loop>` を尊重。
  スケルタルアニメーション (skin) は未対応)。

それ以外は**静的な景観**として配置します。static でないモデルもそのまま置かれます
(`error_message` に記録)。落ちたり押されたりはしません — 動くロボットは URDF から
スポーンするエンティティの領分です。`<physics>` `<scene>` `<gui>` `<plugin>` などの
設定要素は黙って無視し、`<population>`、`<joint>` による可動、pose の `relative_to`
フレームは未対応として報告します。`<world>` の無いモデル単体の SDF はワールドでは
ないので拒否します。

ワールド名は `<world name="...">` から取り、`get_available_worlds` は `world_paths` の
`.sdf` / `.world` もシーン JSON と並べて列挙します (SDF ワールドにタグは無いので、
タグで絞ると除外されます)。

## シミュレーション時刻と /clock

シミュレーション時刻は `/clock` (`rosgraph_msgs/msg/Clock`) で、実時間基準の固定レート
(既定 100 Hz) で配信します。publisher は `Update()` 駆動なので、実際の周期は
`min(配信レート, フレームレート)` になります。起動時は `Application.targetFrameRate = 10`
のため約 10 Hz で、UI からフレームレートを上げると追従します。エンティティ単位ではなくシミュレータ全体で 1 本の
グローバルトピックで、起動時に 1 度だけ登録され、名前空間も付かず、デスポーンや
`reset_simulation` でも消えません。

物理ステップではなく実時間の周期で publish しているのは、一時停止が `Time.timeScale` を
0 にする実装で、その間 `FixedUpdate` が回らないためです。停止中・一時停止中も
**凍結した値のまま**配信が続くので (Gazebo と同じ挙動)、下流の `use_sim_time` な
ノードが時計を失って止まることはありません。`reset_simulation` を `SCOPE_TIME` で呼ぶと
時刻はゼロに戻り、直後の `/clock` からそれが反映されます。

`/joint_states`・`/ground_truth`・`/tf` のスタンプも同じ時計から取っているので、
ここから配信されるものは 1 本の時間軸を共有します。

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

## resource_string からのスポーン

`entity_resource.uri` を空にして `resource_string` に URDF そのものを入れられます
(`SPAWNING_RESOURCE_STRING`)。xacro を展開した結果をファイルに落とさず直接渡す用途です。

受け取った定義は一時ファイルへ書き出してから通常の経路に合流させます。URDF Importer も
こちらの XML 解析もパスを受け取る作りなので、文字列専用の経路を別に作るより読み口を
1 本にしたほうが分岐が増えないためです。一時ファイルはスポーンの完了時に削除します。

### mesh の解決先

文字列で渡すと「URDF の隣」という起点が無くなるので、mesh 参照の解決先を別に与える
必要があります。`simulation_resources.json` の **`spawnable_paths` が検索パス**として
使われます (Gazebo の `GZ_SIM_RESOURCE_PATH` にあたるもの)。あわせて
**`AMENT_PREFIX_PATH`** も見るので、ROS 2 環境を source してから起動していれば
`package://` は設定なしで解決します。

`package://<pkg>/<rest>` は検索パスごとに次の順で探します。

| 候補 | 検索パスが指しているもの |
|---|---|
| `<root>/<pkg>/<rest>` | パッケージが並ぶディレクトリ |
| `<root>/share/<pkg>/<rest>` | ament の install prefix |
| `<root>/<pkg>/share/<pkg>/<rest>` | colcon の install (分離配置) |
| `<root>/<rest>` | そのパッケージのディレクトリそのもの |

この探索は **URDF の隣に見つからなかったときだけ**走るので、これまで読めていた URDF の
挙動は変わりません。絶対パスの `file://` 参照 (このリポジトリの description が使っている形)
は元から起点に依存しないため、検索パスなしでも解決します。

なおこの検索パスは URDF Importer 側に入れたので、`uri` で渡す通常のスポーンでも
`package://` が引けるようになっています。

## simulate_steps アクション

`step_simulation` と同じことを、**1 ステップごとの経過報告**と**途中キャンセル**つきで
行います。長いステップ数を進める場合はこちらが向いています。

```bash
ros2 service call /set_simulation_state simulation_interfaces/srv/SetSimulationState "{ state: { state: 2 } }"
ros2 action send_goal -f /simulate_steps simulation_interfaces/action/SimulateSteps "{ steps: 100 }"
```

- ゴールは**常に受理**します。一時停止していないなどの失敗は、結果の
  `result` (`OPERATION_FAILED`) と goal の status (`ABORTED`) で伝えます。ゴールを
  拒否するには実行開始前にもう 1 往復必要になるうえ、`SimulateSteps.action` 自身が
  「一時停止していなければ結果で OPERATION_FAILED を返す」と書いているためです。
- feedback は 1 物理ステップにつき 1 回、`completed_steps` と `remaining_steps` を返します。
- キャンセルするとその時点で止まり、status は `CANCELED` になります。進んだぶんは
  有効なので結果の `result` 自体は `RESULT_OK` です。
- 上限とキャンセルの扱いは `step_simulation` と共有しています。サービスとアクションが
  同時に走ることはなく、後から来たほうは `OPERATION_FAILED` になります。

### エンドポイント側の要件

アクションは ROS-TCP-Connector / ROS-TCP-Endpoint の**両方**に手を入れて実現しています
(本家にはアクションの口がありません)。Connector に
`ROSConnection.ImplementAction<TGoal, TResult>` を、Endpoint に `__unity_action` /
`__action_goal` / `__action_cancel` / `__action_feedback` / `__action_result` の
システムコマンドを追加してあります。**どちらも hijimasa fork の対応版が要ります。**

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
- `world_paths` からは `.json` のうち**シーンとして読めるもの**と、`.sdf` / `.world` の
  うち **SDF ワールドとして読めるもの**だけを候補にします
  (設定ファイルなど無関係な JSON やモデル単体の SDF が混ざるためです)。
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

## 実行時設定 (simulation_resources.json の `settings`)

`simulation_resources.json` には任意の `settings` 要素を置ける。起動時に 1 度だけ反映される。

```json
{
  "settings": {
    "physics_hz": 200,
    "target_fps": 30
  }
}
```

- `physics_hz` — 物理演算レート。`Time.fixedDeltaTime = 1 / physics_hz` を設定する。
  10〜1000 Hz にクランプ。キーが無ければプロジェクト既定の 50 Hz のまま。
- `target_fps` — 描画フレームレート (`Application.targetFrameRate`)。1〜1000 にクランプ。
  キーが無ければ起動時既定の 10 FPS のまま。

要素・フィールドはすべて任意で、無い構成は従来どおりに動く。指定した場合は起動時の
既定値 (フレームレート入力欄がハードコードしている 10 FPS を含む) より優先され、
サイドバーの入力欄の表示も実際に効いている値へ更新される。

どちらのレートもサイドバー (`Frame Rate[Hz]` / `Physics Rate[Hz]`) から実行中に変更
できる。物理レートを上げると高速走行時のホイールスリップが大きく減る
([URDF-Collision-Material.md](URDF-Collision-Material.md): 1.5 m/s で 50 Hz → 74% スリップ、
200 Hz → 7%)。ただし実行中の変更は決定性を壊し、サーボモデルの安定余裕 (伝達剛性の
上限) も物理ステップに紐づいて変わる ([Known-Limitations.md](Known-Limitations.md) 参照)。
再現性が要る場合は実行中に触らず `settings` で起動時に決めておくこと。FPS 表示の隣の
HUD には実時間係数 (`RTF`: 壁時計 1 秒あたりのシミュレーション秒。停止・一時停止中は
`0.00`) が出るので、物理レートを上げたコストはすぐ確認できる。
