# collision_material — URDF から摩擦と「センサ専用コライダ」を設定する

URDF の標準要素には接触摩擦の指定も「センサには見えるが押し返さない形状」の指定も
ないので、このシミュレータは独自要素 `<collision_material>` を読みます。URDF Importer は
この要素を知らないため、インポート後にシミュレータ側で当てています
(`Packages/UrdfProperties/Runtime/CollisionMaterialApplier.cs`)。

## 書き方

定義は `<robot>` 直下に置き、各 `<collision>` から名前で参照します。

```xml
<robot name="diffbot">
  <collision_material name="wheel">
    <friction static="1.0" dynamic="1.0" combine="maximum"/>
    <contact_offset value="0.02"/>
  </collision_material>

  <link name="left_wheel_link">
    <collision>
      <geometry><cylinder radius="0.05" length="0.02"/></geometry>
      <collision_material name="wheel"/>
    </collision>
  </link>
</robot>
```

| 要素 / 属性 | 意味 | 既定 |
|---|---|---|
| `friction@static` | 静止摩擦係数 | 0 |
| `friction@dynamic` | 動摩擦係数 | 0 |
| `friction@combine` | 接触相手との合成方法 (`average` / `minimum` / `multiply` / `maximum`) | `average` |
| `contact_offset@value` | コライダの contact offset [m] | 触らない (Unity の既定 0.01) |
| `sensor_only@value` | `true` でコライダをトリガにする。レイキャスト系センサ (LiDAR・深度カメラ) には当たるが接触は起こさない。要素だけ書けば `true` | トリガにしない |
| `magnetic_tape@polarity` | 形状を磁気誘導センサ用の磁気テープにする (`track` か `marker`)。`sensor_only` を含意する。[Magnetic-Guide-Sensor-ja.md](Magnetic-Guide-Sensor-ja.md) 参照 | テープにしない |

`<collision>` を 1 リンクに複数書いた場合、**i 番目の `<collision>` が i 番目の形状に**
対応します。1 つの `<collision>` が複数のコライダに展開される場合 (サブメッシュなど) は
そのすべてに当たります。

旧称 `<physics_material>` も読めますが、警告が出ます。

## センサにだけ見える物体 (雑草・草むら・垂れ下がった布など)

LiDAR には映ってほしいが、ロボットを止めてはいけない物体があります。`<collision>` を
書かないリンクでは**実現できません**。LiDAR は物理レイキャストでコライダにしか当たらないので、
collision の無いリンクは単に見えなくなります。形状は普通に `<collision>` として書き、
`sensor_only` を付けてください:

```xml
<robot name="weeds">
  <collision_material name="weed">
    <sensor_only value="true"/>
  </collision_material>

  <link name="world"/>                      <!-- その場に留めるため -->
  <link name="weeds_link">
    <collision>
      <origin xyz="1.0 0.2 0.15"/>
      <geometry><cylinder radius="0.05" length="0.3"/></geometry>
      <collision_material name="weed"/>
    </collision>
    <collision>
      <origin xyz="1.3 -0.1 0.15"/>
      <geometry><cylinder radius="0.04" length="0.3"/></geometry>
      <collision_material name="weed"/>
    </collision>
  </link>
  <joint name="fix" type="fixed"><parent link="world"/><child link="weeds_link"/></joint>
</robot>
```

仕組みと注意点:

- `sensor_only` は Unity の `Collider.isTrigger` を立てます。トリガはレイキャストには
  当たり (プロジェクト設定 *Queries Hit Triggers* が有効)、接触解決には一切参加しないので、
  ロボットはすり抜け、`get_contact_events` にも記録されません。
- **1 リンクに多数の `<collision>`** を書いてください (株ごとにリンクを切らない)。
  リンクは ArticulationBody になり、PhysX は 1 つの articulation を 64 body までに制限
  します。リンクあたりのコライダ数に制限はありません。
- **`world` リンクに fixed ジョイントで吊ってください**。トリガしか持たないボディは何にも
  支えられないので、ルートが動ける状態だと床を抜けて落ち続けます。sensor_only しか持たない
  エンティティのルートが immovable でない場合、スポーン時に警告を出します。
- トリガにできるのは convex なコライダだけです。プリミティブと URDF Importer が作る
  メッシュは元から convex です。非 convex の MeshCollider は警告付きで convex
  (最大 255 面の凸包) に変えてからトリガにします。
- レイは最初のヒットで止まるので、sensor_only の形状はビーム単位では不透明です。
  一部のビームを通したいなら、1 つの塊ではなく細い茎を疎らに置いてください。
- intensity で点を区別することはできません (LiDAR の intensity は距離のみで決まります)。
  「雑草の点」のラベル付けは既知の配置から消費側で行ってください。
- それ以外は普通のエンティティです。`get_entities` に載り、`delete_entity` と
  `reset_simulation` の `SCOPE_SPAWNED` で消え、`set_entity_info` でタグを付けられます。

sensor_only のマテリアルはログ行の末尾に `sensor_only` が付きます:

```
[CollisionMaterial] Applied 'weed' to 'weeds_link' (2 collider(s), static=0, dynamic=0, combine=Average, sensor_only)
```

## combine に注意

**既定の `average` では、書いた値がそのまま効きません。** Unity は接触する 2 つの
マテリアルのうち **列挙値が大きいほうの combine を採用**します
(`average` < `minimum` < `multiply` < `maximum`)。

床にマテリアルを設定していない場合、Unity の既定値 (静止・動ともに 0.6、combine は
`average`) が相手になります。したがって

```xml
<friction static="1.0" dynamic="1.0"/>   <!-- combine 省略 = average -->
```

と書いても、実効摩擦係数は **(1.0 + 0.6) / 2 = 0.8** です。相手によらず指定値を
効かせたいときは `combine="maximum"` を指定してください。

## 摩擦を上げても滑るとき

**速度が上がると、摩擦係数とは無関係に滑ります。** 物理ステップあたりに接触点が
動く距離が大きくなり、接触が保てなくなるためです。diffbot (車輪半径 0.05 m) で
車輪の回転量と実移動量から測った滑り率:

| 車輪速度 | 相当速度 | 50 Hz (既定) | 200 Hz |
|---|---|---|---|
| 3 rad/s | 0.15 m/s | 0.0 % | 1.2 % |
| 10 rad/s | 0.5 m/s | 0.0 % | 2.0 % |
| 30 rad/s | 1.5 m/s | **74.2 %** | **7.2 %** |
| 60 rad/s | 3.0 m/s | 95.8 % | 92.1 % |

摩擦係数は 4 行とも同じ (`static=1.0 dynamic=1.0`) で、変えたのは
`Fixed Timestep` だけです。1.5 m/s の滑りが 74% から 7% へ落ちることから、
**高速側の滑りは摩擦の設定ではなく刻みが支配的**だと分かります。

50 Hz なら 1 ステップで接触点が 3 cm 進み、半径 5 cm の車輪では 1 ステップあたり
約 34° 回る計算になります。この領域では接触パッチが維持できません。

対処は次のいずれかです。

- 常用速度を 0.5 m/s 程度までに抑える (既定の 50 Hz で滑りは測定限界以下)
- `Project Settings > Time > Fixed Timestep` を小さくする。ただし全体の計算量に効きます
- 車輪半径を大きくする (1 ステップあたりの回転角が小さくなる)

## 設定が効いているかを確かめる

- **単体テスト**: `Packages/UrdfProperties/Tests/Runtime/CollisionMaterialApplierTests.cs`。
  URDF の記述がコライダの `staticFriction` / `dynamicFriction` / `frictionCombine` /
  `contactOffset` / `isTrigger` に届いているかを直接読んで検査します。アセンブリが
  全プラットフォーム対象なので、テストランナーでは PlayMode 扱いです:

  ```bash
  Unity -batchmode -nographics -projectPath <このリポジトリ> -runTests -testPlatform playmode \
        -assemblyNames UrdfPropertyTests -testResults results.xml -logFile unity.log
  ```
- **シナリオテスト**: 併設ワークスペースの
  `sim_test_utils/examples/test_sensor_only_collision.py` が sensor_only の雑草を置き、
  箱を落として接触しないこと、LiDAR が想定距離に返りを出すことを確かめます。
- **実機**: 起動中のシミュレータのログに、当たった分だけ次の行が出ます。

  ```
  [CollisionMaterial] Applied 'wheel' to 'left_wheel_link' (1 collider(s), static=1, dynamic=1, combine=Average)
  ```

  リンク名の綴り違いや未定義の参照は警告として出ます。
- **挙動**: 適合性テストの C5b が、車輪の回転量に対する実移動量の比 (滑り率) を測ります
  ([Service-Conformance-Test-ja.md](Service-Conformance-Test-ja.md))。

## 色について

色は URDF 標準の `<material><color rgba="..."/></material>` で、こちらは URDF Importer が
適用します。alpha を含めて反映されることは Importer 側のテスト
(`Tests/Runtime/Extensions/UrdfMaterialTests.cs`) で検証しています。
