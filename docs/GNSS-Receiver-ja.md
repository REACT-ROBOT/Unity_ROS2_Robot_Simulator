# GNSS 受信機センサ

`<sensor type="gnss">` は GNSS 受信機を模擬します。同じリンクに
[`gnss_sky_view`](GNSS-Sky-View-Sensor-ja.md) があれば、それが見た空から
**RTK Fix / Float / 単独測位の判定と位置誤差**を生成します。無ければ真値をそのまま報告します。

**受信機モデルはシミュレータ側にあります。** ROS ユーザはロボットを spawn するだけで、
追加のノードを起動せずに劣化込みの測位を得られます。

## 出すトピック

| トピック | 型 | 内容 |
|---|---|---|
| `/<robot>/<link>/fix` | `sensor_msgs/NavSatFix` | 劣化込みの測位。status と position_covariance が品質に追従 |
| `/<robot>/<link>/solution` | `simulation_extra_interfaces/GnssSolution` | NavSatFix が表現できないもの |

**`NavSatFix` の `status` は RTK Fix と Float を区別できません。**
`NO_FIX` / `FIX` / `SBAS_FIX` / `GBAS_FIX` の 4 値しかなく、Fix も Float も `GBAS_FIX` に潰れます。
これはこのセンサの制約ではなくメッセージの制約で、実機のどのドライバでも同じです。
そのため**品質は `position_covariance` で伝えます**（`robot_localization` の
`navsat_transform_node` が実際に読むのもここです）。グレード別の 1σ から作っており、
`COVARIANCE_TYPE_DIAGONAL_KNOWN` で出します。

区別が必要な consumer（GGA quality 4/5 を出す NMEA ブリッジ、品質で重み付けする localizer）は
`GnssSolution` を読んでください。誤差の内訳（確率的なバイアス / 反射由来 / wrong fix）も
入っているので、評価では推定ではなく**厳密な誤差**が使えます。

## URDF

```xml
<sensor name="gnss_antenna_link" type="gnss">
  <update_rate>5.0</update_rate>
  <origin_latitude>36.549774</origin_latitude>
  <origin_longitude>139.928772</origin_longitude>
  <origin_altitude>100.0</origin_altitude>
  <reconvergence_sec>8.0</reconvergence_sec>
  <lock_seconds_for_fix>8.0</lock_seconds_for_fix>
  <wrong_fix_probability>0.01</wrong_fix_probability>
  <stochastic_error>true</stochastic_error>
  <seed>20260914</seed>
</sensor>
```

| 要素 | 既定 | 意味 |
|---|---|---|
| `origin_*` | 東京 | 測地原点。**gpsd に渡す base と一致させること** |
| `reconvergence_sec` | 8.0 | 衛星ごとのロックが使えないときの再収束時間 [s] |
| `lock_seconds_for_fix` | 8.0 | 1 衛星が整数解に寄与できるまでの連続ロック時間 [s] |
| `wrong_fix_probability` | 0.01 | 再固定のたびに誤った整数解を引く確率 |
| `stochastic_error` | true | false で乱数由来の誤差を消し、幾何が生む誤差だけ残す |
| `seed` | 20260914 | 乱数シード |

空の情報源は同じリンクから**遅延解決**するので、URDF 内の記述順に依存しません。

## モデルの中身

| 要素 | 内容 |
|---|---|
| グレード判定 | 使える衛星数と HDOP が上限を決め、整数解が解けるかが実際の到達点を決める |
| 確率的誤差 | 1 次ガウス・マルコフ。**白色ノイズではなく数十秒相関する偏り**で、平均しても消えない。グレード変化をまたいで連続 |
| 反射 (NLOS) | 過剰行路長を疑似距離バイアスとして最小二乗に入れ、**誤差の方向まで幾何と整合**させる。同じ場所に戻れば同じ誤差になる |
| 搬送波ロック | 衛星ごとにロック時間を数える。ポールを掠めて 2 個落とすのと高架下で全部落とすのは別物 |
| wrong fix | 誤った整数解。**RTK Fix として満点の自信で報告されるのに数十 cm ずれる**。下流からは原理的に見分けられない |
| Fix 中の上限 | 搬送波マルチパスは波長の 1/4 (L1 で 4.8 cm) を超えられない。方向は幾何、大きさは物理が頭打ち |

## 注意

- **測地変換の軸の規約**。このシミュレータは world を ROS ENU で publish しており
  （東 = Unity +z、北 = Unity −x）、`GeoCoordinateConverter` が内部で使う規約
  （東 = Unity x、北 = Unity z）とは 90° 違います。このセンサは ENU オフセットを明示して
  変換するので整合しますが、`GeoCoordinateSystem.GetCoordinate()` を直接使うと
  **走った距離ぶんだけずれた fix** が得られます。
- 反射損失は入射角にも材質にも依存しない定数です。実測 C/N0 に合わせ込むべき値です。
- 星配置は暦ではなく合成です。**DOP の絶対値を現地調査値のように読まないでください。**
