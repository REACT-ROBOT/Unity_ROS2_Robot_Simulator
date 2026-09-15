# GNSS 天空視界センサ

`<sensor type="gnss_sky_view">` は、GNSS アンテナの位置から衛星ごとにレイを打ち、
**どの衛星が直達で届くか**と、**その集合の幾何がどれだけ良いか (DOP)** を publish します。

測位解も Fix/Float も誤差も出しません。それらは受信機の振る舞いに依存し、シミュレータを
再ビルドせずに調整・単体テストしたいからです。シミュレータでしか分からないのは
「建物が何を遮るか」であり、このセンサはそこだけを担当します。

このセンサを**同じリンクの `<sensor type="gnss">` が受信機モデルの入力として使います**。
受信機は劣化込みの `sensor_msgs/NavSatFix` と、NavSatFix が表現できない情報
（RTK Fix と Float の区別、誤差の内訳、wrong fix）を載せた
`simulation_extra_interfaces/GnssSolution` を publish します。
ROS ユーザは**ロボットを spawn するだけ**で、追加ノードなしに劣化入りの測位を得られます。
詳細は [GNSS-Receiver-ja.md](GNSS-Receiver-ja.md)。

## URDF

```xml
<sensor name="gnss_antenna_link" type="gnss_sky_view">
  <update_rate>5.0</update_rate>
  <satellite_count>24</satellite_count>
  <elevation_mask>15.0</elevation_mask>
  <max_range>500.0</max_range>
  <seed>1</seed>
  <hit_triggers>false</hit_triggers>
  <reflections>true</reflections>
  <reflection_spacing>2.5</reflection_spacing>
  <reflection_loss_db>-13.0</reflection_loss_db>
</sensor>
```

| 要素 | 既定 | 意味 |
|---|---|---|
| `satellite_count` | 24 | 仰角マスクより上に配置する衛星数 |
| `elevation_mask` | 15.0 | 仰角マスク [deg]。これより低い衛星は追尾しない |
| `max_range` | 500.0 | レイ長 [m]。シーンを覆う長さにする |
| `seed` | 1 | 星配置の乱数シード。同じ seed なら同じ空 |
| `hit_triggers` | false | トリガコライダで遮蔽するか |
| `reflections` | true | 1 バウンス反射 (NLOS) を探すか |
| `reflection_spacing` | 2.5 | 反射探索の掃引間隔 [deg]。**半分にするとレイ数は 4 倍** |
| `reflection_loss_db` | -13.0 | 反射経路の損失 [dB]

`name` はセンサを載せるリンク名です。**アンテナの高さと位置がそのまま結果を左右します**
(マストの上なら車体は影を作らない)。

**このセンサは ROS へ publish しません。** 同じリンクの `<sensor type="gnss">` が
受信機モデルの入力として直接読み、衛星の見え方は `gps_msgs/GPSFix` の `GPSStatus`
（used / visible / SNR）に載って出ます。

## 反射 (NLOS)

直達が遮られた衛星について、**上半球を掃いたレイの当たり面から「その経路ならどの衛星から
来たか」を逆算**して 1 バウンス経路を探します。Householder 反射が対合なので、打った方向 u と
法線 n から必要な衛星方向が `s = u - 2(u·n)n` で直接求まり、面を事前に特定する必要がありません。
反射点から衛星方向へ 2 本目を打って空が見えることを確認し、過剰行路長を `d(1 - u·s)` で出します。

過剰行路長は**反射体までの距離に比例**します（垂直な壁なら `2·d·cos²(仰角)`）。
幅 3 m の路地では 2 m 程度、幅 10 m の街路では 10 m 前後になります。

下半球は掃きません。垂直な壁の反射は仰角を変えないので、地平線より上の衛星は
地平線より上のレイでしか届かないためです。地面反射は別の機構で、しかもアンテナの
グラウンドプレーンが抑圧する対象です。

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

直達は衛星数ぶんのレイ (既定 24 本) だけ。反射探索は掃引が `2π/間隔²` 本で、
既定 2.5° なら約 3300 本です。確認レイは衛星あたり上位 4 候補だけなので 100 本程度です。

**反射探索はジョブ完了を待つコルーチンになっています。** 同期待ちにすると、基底クラスが
更新所要時間を周期に算入する仕様と相まって、1 更新が周期を超えた瞬間にセンサが連続実行に
入り主スレッドを占有します。そうなると同じスレッドで動く ROS サービスがタイムアウトし、
シミュレータ全体が応答しなくなります（実際に踏みました）。
