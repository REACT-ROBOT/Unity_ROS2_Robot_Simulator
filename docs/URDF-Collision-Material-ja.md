# collision_material — URDF から摩擦を設定する

URDF の標準要素には接触摩擦の指定がないので、このシミュレータは独自要素
`<collision_material>` を読みます。URDF Importer はこの要素を知らないため、
インポート後にシミュレータ側で当てています
(`Assets/Scripts/UrdfProperties/CollisionMaterialApplier.cs`)。

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

`<collision>` を 1 リンクに複数書いた場合、**i 番目の `<collision>` が i 番目の形状に**
対応します。1 つの `<collision>` が複数のコライダに展開される場合 (サブメッシュなど) は
そのすべてに当たります。

旧称 `<physics_material>` も読めますが、警告が出ます。

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

- **単体テスト**: `Assets/Tests/UrdfPropertyTests/CollisionMaterialApplierTests.cs`。
  URDF の記述がコライダの `staticFriction` / `dynamicFriction` / `frictionCombine` /
  `contactOffset` に届いているかを直接読んで検査します。
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
