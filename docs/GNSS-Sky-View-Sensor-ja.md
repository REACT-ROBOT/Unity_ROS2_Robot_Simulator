# GNSS 天空視界センサ

`<sensor type="gnss_sky_view">` は、GNSS アンテナの位置から衛星ごとにレイを打ち、
**どの衛星が直達で届くか**と、**その集合の幾何がどれだけ良いか (DOP)** を publish します。

測位解も Fix/Float も誤差も出しません。それらは受信機の振る舞いに依存し、シミュレータを
再ビルドせずに調整・単体テストしたいからです。シミュレータでしか分からないのは
「建物が何を遮るか」であり、このセンサはそこだけを担当します。

併設ワークスペースでは `hardware_emulator/gps_emulator` がこのトピックを購読し、
RTK Fix / Float / Single の判定と位置誤差の生成を行って NMEA として出します。

## URDF

```xml
<sensor name="gnss_antenna_link" type="gnss_sky_view">
  <update_rate>5.0</update_rate>
  <satellite_count>24</satellite_count>
  <elevation_mask>15.0</elevation_mask>
  <max_range>500.0</max_range>
  <seed>1</seed>
  <hit_triggers>false</hit_triggers>
</sensor>
```

| 要素 | 既定 | 意味 |
|---|---|---|
| `satellite_count` | 24 | 仰角マスクより上に配置する衛星数 |
| `elevation_mask` | 15.0 | 仰角マスク [deg]。これより低い衛星は追尾しない |
| `max_range` | 500.0 | レイ長 [m]。シーンを覆う長さにする |
| `seed` | 1 | 星配置の乱数シード。同じ seed なら同じ空 |
| `hit_triggers` | false | トリガコライダで遮蔽するか |

`name` はセンサを載せるリンク名です。**アンテナの高さと位置がそのまま結果を左右します**
(マストの上なら車体は影を作らない)。

publish されるトピックは `/<robot>/<link>/sky_view`、型は
`simulation_extra_interfaces/GnssSkyView` です。

## 重要な注意

- **トリガコライダは既定で無視します。** 雑草や磁気テープはトリガで表現されており
  ([URDF-Collision-Material-ja.md](URDF-Collision-Material-ja.md))、衛星を遮ってよいものでは
  ないためです。建物は通常の solid コライダなので普通に遮蔽します。
- **星配置は合成であって暦ではありません。** 仰角マスクより上を立体角について一様に、
  黄金角らせんで並べた決定論的な配置です。実際の中緯度の GNSS 天空は中仰角に密で、
  軌道傾斜角のため極方向に穴があります。**DOP の絶対値を現地調査の値のように読まないで
  ください。** 比較 (開空 vs 建物際) には十分です。
- **衛星は走行中静止しています。** 数分の SILS では実際の衛星も 1 度未満しか動かないので
  誤差は無視でき、静止させておく方が再現性が高くなります。
- 軸の約束: このシミュレータは ROS ENU で world を publish しており
  (`x = Unity z`, `y = -Unity x`)、**東は Unity +z、北は Unity -x** です。
  これは `GeoCoordinateConverter` が内部で使う対応とは異なります。

## コスト

1 更新あたり衛星数ぶんのレイ (既定 24 本) だけです。LiDAR が毎フレーム数万本打っている
のに比べれば無視できます。反射経路 (NLOS) を扱う段階になると数千本必要になるので、
そのときは `RaycastCommand` のバッチ API に移る想定です。
