# 既知の制約と保留中の項目

「直っていない」のではなく **意図的にそのままにしてある**もの、および仕様上できないことの
一覧です。詳細は各ドキュメントにあるので、ここには **何が・なぜ保留なのか**と、
着手するとしたら何を決める必要があるかだけを書きます。

最終更新: 2026-08-12

## 保留中 (直しうるが、いま直していない)

### 1. サーボモデルの物理的制約

`ServoJointModel` には、モデルの作りと物理エンジンの都合から来る制約があります。
主なものは伝達剛性の上限 (離散安定限界と、それより先に効く定量精度の限界) と、
ジョイントリミット面で静止した関節がドライブトルクに応答しなくなるエンジン挙動です。

- 詳細と実測値: [Servo-Model-Guide-ja.md の「制約・注意点」](Servo-Model-Guide-ja.md#制約注意点)
- 妥当性検証: [Servo-Model-Validation-ja.md](Servo-Model-Validation-ja.md)

**なぜ保留か**: どちらもコードの不具合ではなく、離散時間で解く以上避けられない性質か、
PhysX 側の挙動です。「サービスを実装する」ような形で消せる種類のものではありません。

**着手するなら**: モデルそのものを変える話になります。最低限、次を決める必要があります。

- 剛性上限を上げたいのか、それとも現在の上限を守れているか検証で担保したいのか
- 上げるなら、伝達を陰的に解く (半陰解法) のか、物理ステップを細かくするのか。
  前者はモデルの書き換え、後者は全体の計算量に効きます
- リミット面での固着は、リミット手前で止める運用で回避するのか、エンジン側の挙動を
  回避する実装を入れるのか

再測定は `Assets/Tests/ServoModelTests/ServoEnvironmentProbe.cs` の診断テストで行えます。

### 2. `humble` ブランチに新しい検証が入っていない

適合性テストの `humble` ブランチ (Unity_ROS2_sample) は **Humble で検証を通した時点の
スナップショット**で、その後に足した検証項目 (`WORLD_TAGS` 以降の H 群後半、I 群、
G6 / F3 / H2b) は入っていません。

**なぜ保留か**: ブランチ構成を決めたときの取り決めどおりです。`humble` は Humble 固有の
修正を入れる場所で、新機能を追いかける場所ではありません。既存の検証は機能申告で
分岐しているため、新機能を持つシミュレータに対しても落ちません。

**着手するなら**: `main` から該当コミットを cherry-pick します。両ブランチで同じ検証を
維持したいのか、`humble` は当時の状態で凍結しておきたいのかを先に決めてください。
詳細は [Unity_ROS2_sample の README](https://github.com/hijimasa/Unity_ROS2_sample) の
「ブランチ」節を参照。

## 仕様上できないこと

こちらは設計として受け入れているもので、着手予定はありません。

| 項目 | 内容 | 記載場所 |
|---|---|---|
| `set_entity_state` の加速度 | 取得はできるが設定は無視する。Unity に加速度を与える口が無く、力へ読み替えるには質量の扱いを決める必要がある | [Simulation-Interfaces-Services-ja.md](Simulation-Interfaces-Services-ja.md) |
| 加速度の平滑化 | 速度の 1 階差分をそのまま返すので接触時に振れる。均すのは受け取り側の責務 | 同上 |
| エンティティの範囲 | スポーン経路 (`spawn_entity` / `spawn_entities`、および同じ実装を呼ぶ GUI の URDF ボタン) で作ったものだけ。それ以外の GUI で置いた物は景観扱い | 同上 |
| 固定台ロボットで測れない検証 | ベースリンクが `immovable` なので、ルート姿勢が動く前提の項目 (C5 / H2b) は SKIP になる | [Service-Conformance-Test-ja.md](Service-Conformance-Test-ja.md) |
| 本家 ROS-TCP-Endpoint との併用 | 購読解除と Unity 側アクションのシステムコマンドを本家が持たないため、組み合わせられない | 同上 |
| 高速走行時の車輪の滑り | 物理刻みあたりの接触点の移動が大きくなり接触が保てない。摩擦係数では解消しない (50 Hz で 1.5 m/s なら滑り 74%、200 Hz なら 7%) | [URDF-Collision-Material-ja.md](URDF-Collision-Material-ja.md) |
| 摩擦の `combine` 既定 | `average` なので、床との平均が実効値になる。指定値を効かせるには `combine="maximum"` | 同上 |
| `ArticulationBody.jointVelocity` | xDrive 駆動中の関節の運動を反映しない。モデルは位置の有限差分を使う | [Servo-Model-Guide-ja.md](Servo-Model-Guide-ja.md) |
| 点群重畳のカリング | UnitySensors の点群ビジュアライザはワールド原点中心の固定 100 m バウンズで描くため、原点からおよそ 50 m 以上離れたロボットでは重畳が消える (センサトピック自体は影響なし) | README-ja.md のセンサ可視化 |
| SDF ワールドの対応サブセット外 | SDF `.world` のモデルは静的な景観として読む: static でないモデルも落ちたり押されたりせず、`<joint>` は動かず、`<population>` `<physics>` `<plugin>` は捨てる (`error_message` に報告)。`<actor>` は `<trajectory>` に追従する形状としてのみ対応 (スケルタルアニメーション無し)。動くロボットは URDF からスポーンするエンティティの領分で、ワールドがロボットを連れてくる形は別途保留中の「ワールドにエンティティを含める」案 | [Simulation-Interfaces-Services-ja.md](Simulation-Interfaces-Services-ja.md) の SDF ワールド |

## simulation_interfaces の対応状況

srv で定義されているサービス、`SimulateSteps` アクション、`SimulatorFeatures` の機能は
**すべて実装済み**です。未実装のものはありません
([Simulation-Interfaces-Services-ja.md](Simulation-Interfaces-Services-ja.md))。
