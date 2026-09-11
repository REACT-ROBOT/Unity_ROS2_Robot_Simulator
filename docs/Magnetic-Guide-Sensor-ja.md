# 磁気誘導センサと磁気テープ

AGV 用の磁気誘導 (ライン) センサ — Roboteq MGS1600、アキュレイトシステム UDS-1213 —
は、床から数 cm の高さに取り付けるホール素子のバーです。バーの下のどこに磁気テープが
あるか、テープがあるかどうか、そしてトラックの脇に貼った逆極性の短いテープ
「マーカ」を報告します。このシミュレータはセンサ (`<sensor type="magnetic_guide">`) と
テープ (collision 形状の `<magnetic_tape>`) の両方を模擬します。

## テープ

テープは URDF エンティティの一部なので、他と同じようにスポーン・列挙・削除・リセット
されます。1 本の帯は薄い `<collision>` box で、その `<collision_material>` に
`<magnetic_tape polarity="track"/>` または `polarity="marker"` を付けます:

```xml
<robot name="course">
  <collision_material name="track"><magnetic_tape polarity="track"/></collision_material>
  <collision_material name="marker"><magnetic_tape polarity="marker"/></collision_material>

  <link name="world"/>
  <link name="tape_link">
    <collision>
      <origin xyz="5.0 0 0.0005"/><geometry><box size="10.0 0.025 0.001"/></geometry>
      <collision_material name="track"/>
    </collision>
    <collision>
      <origin xyz="3.15 0.035 0.0005"/><geometry><box size="0.3 0.025 0.001"/></geometry>
      <collision_material name="marker"/>
    </collision>
  </link>
  <joint name="fix" type="fixed"><parent link="world"/><child link="tape_link"/></joint>
</robot>
```

- `magnetic_tape` は `sensor_only` を含意します。box はトリガになり、ロボットは踏んで走り、
  `get_contact_events` にも出ません ([URDF-Collision-Material-ja.md](URDF-Collision-Material-ja.md))。
- LiDAR はレイヤ 0 のレイキャストで、トリガにも当たります。床上 1 mm の帯は 2D スキャン面
  より下です。3D LiDAR には床の一部として見えますが、実物のテープも同じです。
- コース全体を **1 リンク** に入れてください (リンクは ArticulationBody で、PhysX は
  1 articulation あたり 64 まで)。`world` リンクに吊るとその場に留まります。
- マーカはトラック端から 15〜30 mm 外側 (データシートの決まり)。25 mm テープなら
  マーカ中心はトラック中心から ±35〜±40 mm です。
- ポリラインのコースは JSON で書いて変換するのが楽です。併設ワークスペースの
  `tools/magnetic_line/gen_tape_urdf.py` がそれをします。

## センサ

```xml
<simulation>
  <sensor name="uds_link" type="magnetic_guide">
    <update_rate>50</update_rate>
    <width>0.16</width>          <!-- バー幅 [m] -->
    <pitch>0.001</pitch>         <!-- 素子ピッチ = 出力分解能 [m] -->
    <min_height>0.01</min_height><!-- これより近いテープは読まない (擦っている) -->
    <max_height>0.06</max_height><!-- これより遠いテープは読まない (磁場が弱い) -->
    <fork>nearest</fork>         <!-- nearest | left | right: 分岐でどちらを出すか -->
    <noise><stddev>0.0</stddev></noise>  <!-- 位置に載せるガウス雑音 [m] -->
  </sensor>
</simulation>
```

`name` は他のセンサ同様、バーを付けるリンクです。リンク座標系がセンサ座標系で、原点は
バー面の中心、**+x 前、+y 左、-z がバーの見る向き**です。実機の高さ (データシートの
推奨は床上 20〜40 mm) に取り付けてください。

各素子は `max_height` だけ下へレイを打ち、最初に当たったものが `MagneticTape` を持ち
`min_height` 以上離れていれば「テープ上」です。バーとテープの間に固体があれば遮蔽
されます。連続するトラック素子が 1 本のトラックで、その中心が出力位置です。素子 1 個
の抜けではトラックは分かれません。

トピック: `/<entity>/<link>/magnetic_guide`、型 `simulation_extra_interfaces/MagneticGuide`:

| フィールド | 意味 |
|---|---|
| `track_detected` | トラック極性のテープがバーの下にある |
| `position` | 選択したトラックの横位置 [m]、**+左**、`pitch` で量子化。無ければ 0 |
| `left_marker` / `right_marker` | 選択トラックのその側 (トラック無しならバー中心基準) にマーカ極性のテープ |
| `track_positions` | バー下の全トラック、左から右。分岐では 2 本 |

センサは物理クエリなのでヘッドレスでも動きます。

## 確かめ方

- ソルバのテスト (サンプル列 → 読み): `UnitySensors/Packages/UnitySensors/Tests/Editor/MagneticGuideSolverTests.cs`
- センサ幾何のテスト (レイキャスト・左右・高さ・遮蔽): `Assets/Tests/MagneticGuideTests`
  (playmode、`-assemblyNames MagneticGuideTests`)
- シナリオテスト: 併設ワークスペースの `sim_test_utils/examples/test_magnetic_guide.py`
  がコースとセンサをスポーンしてトピックを確かめます。
