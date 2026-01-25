# 空力学モデリングガイド

このガイドでは、Unity ROS2 Robot Simulatorにおける航空機、UAV、羽ばたきロボットモデルの空力学的特性の設定方法を説明します。

## 概要

シミュレータは **Khan and Nahon 2015** 論文に基づいた空力学物理をAeroSurfaceシステムを通じてサポートしています：

1. **揚力係数曲線 (Cl-α)** - 通常飛行時は線形、失速時は非線形
2. **抗力係数曲線 (Cd-α)** - 寄生抗力、誘導抗力、失速誘導抗力
3. **失速モデル** - 通常飛行と失速状態間の滑らかな遷移
4. **制御面** - フラップ、エルロン、エレベーター、ラダー

このシステムは空気中と水中環境（ハイドロフォイル）の両方に対応しています。

## 目次

- [基本概念](#基本概念)
- [URDF設定](#urdf設定)
- [パラメータリファレンス](#パラメータリファレンス)
- [力の計算](#力の計算)
- [制御面の設定](#制御面の設定)
- [使用例](#使用例)

---

## 基本概念

### 空力

各空力学的サーフェスは3種類の力を生成します：

| 力 | 数式 | 説明 |
|----|------|------|
| 揚力 | L = Cl × q × S | 気流に垂直 |
| 抗力 | D = Cd × q × S | 気流に平行 |
| モーメント | M = Cm × q × S × c | コード周りのピッチングモーメント |

ここで：
- `q = 0.5 × ρ × V²` = 動圧
- `S = chord × span` = 翼面積
- `c` = コード長
- `ρ` = 流体密度

### 迎角 (AoA)

気流方向とサーフェスのコード線の間の角度。

- **低迎角** (< 失速角): 線形揚力、低抗力
- **失速** (> 失速角): 揚力低下、抗力急増
- **失速後**: 剥離流、平板的挙動

### アスペクト比補正

有限翼幅は翼端渦を生成し、有効揚力を減少させます。補正された揚力傾斜は：

```
Cl_corrected = Cl_slope × AR / (AR + 2×(AR+4)/(AR+2))
```

ここで `AR = span / chord` はアスペクト比です。

---

## URDF設定

### 基本構造

```xml
<robot name="my_aircraft">
  <!-- ロボット全体のデフォルト空力学パラメータ -->
  <aerodynamics>
    <fluid_density>1.225</fluid_density>  <!-- 海面レベルの空気 -->
  </aerodynamics>

  <link name="left_wing">
    <collision>
      <!-- リンク別の空力学サーフェス -->
      <aerodynamic_surface>
        <chord>1.5</chord>
        <span>2.0</span>
        <lift_slope>6.28</lift_slope>
        <zero_lift_aoa>-2</zero_lift_aoa>
        <stall_angle_high>15</stall_angle_high>
        <stall_angle_low>-15</stall_angle_low>
        <skin_friction>0.02</skin_friction>
        <flap_fraction>0.2</flap_fraction>

        <!-- 制御面設定 -->
        <control_surface type="roll" multiplier="1"/>
      </aerodynamic_surface>
      <geometry>
        <box size="1.5 0.1 2.0"/>
      </geometry>
    </collision>
  </link>
</robot>
```

### 完全なパラメータ構造

```xml
<aerodynamic_surface>
  <!-- ジオメトリ -->
  <chord>1.0</chord>              <!-- コード長（メートル） -->
  <span>1.0</span>                <!-- スパン長（メートル） -->
  <aspect_ratio>1.0</aspect_ratio> <!-- 手動AR（オプション） -->

  <!-- 揚力特性 -->
  <lift_slope>6.28</lift_slope>   <!-- dCl/dα（1/rad）（薄翼理論: 2π） -->
  <zero_lift_aoa>0</zero_lift_aoa> <!-- ゼロ揚力迎角（度） -->

  <!-- 失速角（レガシー単一値または媒質別） -->
  <stall_angle_high>15</stall_angle_high>      <!-- 両媒質（レガシー） -->
  <stall_angle_low>-15</stall_angle_low>       <!-- 両媒質（レガシー） -->
  <stall_angle_high_air>15</stall_angle_high_air>    <!-- 空気中のみ -->
  <stall_angle_low_air>-15</stall_angle_low_air>     <!-- 空気中のみ -->
  <stall_angle_high_water>12</stall_angle_high_water> <!-- 水中のみ -->
  <stall_angle_low_water>-12</stall_angle_low_water>  <!-- 水中のみ -->

  <!-- 抗力特性（レガシー単一値または媒質別） -->
  <skin_friction>0.02</skin_friction>          <!-- 両媒質（レガシー） -->
  <skin_friction_air>0.02</skin_friction_air>  <!-- 空気中のみ -->
  <skin_friction_water>0.01</skin_friction_water> <!-- 水中のみ -->

  <!-- 制御面 -->
  <flap_fraction>0.0</flap_fraction>   <!-- フラップのコード比率（0-0.5） -->
  <max_flap_angle>50</max_flap_angle>  <!-- 最大偏角（度） -->
  <control_surface type="pitch|roll|yaw|flap" multiplier="1"/>

  <!-- 流体特性 -->
  <fluid_medium>air</fluid_medium>     <!-- "air", "water", または "auto" -->

  <!-- 密度（レガシー単一値または媒質別） -->
  <fluid_density>1.225</fluid_density>   <!-- レガシー: 空気/水を自動判定 -->
  <air_density>1.225</air_density>       <!-- 空気密度（kg/m³） -->
  <water_density>1027</water_density>    <!-- 水密度（kg/m³） -->

  <!-- 動粘度（レイノルズ数計算用） -->
  <air_kinematic_viscosity>1.5e-5</air_kinematic_viscosity>   <!-- m²/s -->
  <water_kinematic_viscosity>1e-6</water_kinematic_viscosity> <!-- m²/s -->

  <!-- キャビテーション（水中のみ） -->
  <enable_cavitation>false</enable_cavitation>
  <cavitation_threshold>0.5</cavitation_threshold>
</aerodynamic_surface>
```

---

## パラメータリファレンス

### ジオメトリパラメータ

| パラメータ | デフォルト | 単位 | 説明 |
|-----------|-----------|------|------|
| `chord` | 1.0 | m | コード長（流れ方向の幅） |
| `span` | 1.0 | m | スパン長（流れに垂直） |
| `aspect_ratio` | 自動 | - | AR = span/chord（未指定時は自動計算） |

### 揚力パラメータ

| パラメータ | デフォルト | 範囲 | 説明 |
|-----------|-----------|------|------|
| `lift_slope` | 6.28 | 0-8 | 揚力曲線傾斜（dCl/dα）（1/rad） |
| `zero_lift_aoa` | 0 | -10〜5 | ゼロ揚力迎角（度） |
| `stall_angle_high` | 15 | 10-20 | 正の失速角 - 両媒質（レガシー） |
| `stall_angle_low` | -15 | -20〜-10 | 負の失速角 - 両媒質（レガシー） |
| `stall_angle_high_air` | 15 | 10-20 | 空気中の正の失速角 |
| `stall_angle_low_air` | -15 | -20〜-10 | 空気中の負の失速角 |
| `stall_angle_high_water` | 12 | 8-15 | 水中の正の失速角 |
| `stall_angle_low_water` | -12 | -15〜-8 | 水中の負の失速角 |

**代表的な揚力傾斜:**
- 薄翼理論: 2π ≈ 6.28
- NACA 0012: ~5.7
- 平板: ~3.5

**注意:** 水中の失速角はレイノルズ数が高いため、通常は空気中より低くなります。

### 抗力パラメータ

| パラメータ | デフォルト | 範囲 | 説明 |
|-----------|-----------|------|------|
| `skin_friction` | 0.02 | 0.01-0.05 | 表面摩擦係数 - 両媒質（レガシー） |
| `skin_friction_air` | 0.02 | 0.01-0.05 | 空気中の表面摩擦係数 |
| `skin_friction_water` | 0.01 | 0.005-0.03 | 水中の表面摩擦係数 |

### 制御面パラメータ

| パラメータ | デフォルト | 範囲 | 説明 |
|-----------|-----------|------|------|
| `flap_fraction` | 0 | 0-0.5 | フラップのコード比率 |
| `max_flap_angle` | 50 | 30-60 | 最大フラップ偏角（度） |

### 流体特性

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `fluid_medium` | air | "air", "water", または "auto"（自動検出） |
| `fluid_density` | 1.225 | レガシー: 単一密度値（kg/m³） |
| `air_density` | 1.225 | 空気密度（kg/m³） |
| `water_density` | 1027 | 水密度（kg/m³）- 海水 |
| `air_kinematic_viscosity` | 1.5e-5 | 空気の動粘度（m²/s） |
| `water_kinematic_viscosity` | 1e-6 | 水の動粘度（m²/s） |

### キャビテーションパラメータ（水中のみ）

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `enable_cavitation` | false | 高速時のキャビテーション効果を有効化 |
| `cavitation_threshold` | 0.5 | キャビテーション数閾値（0.2-1.0） |

---

## 力の計算

### 揚力係数（通常飛行）

線形領域（迎角 < 失速角）：

```
Cl = Cl_slope_corrected × (α - α_zero_lift)
```

### 誘導抗力

プラントルの揚力線理論より：

```
α_induced = Cl / (π × AR)
α_effective = α - α_zero_lift - α_induced
```

### 抗力係数

```
Cd = Cn × sin(α_eff) + Ct × cos(α_eff)
```

ここで：
- `Cn` = 法線力係数
- `Ct` = 接線（摩擦）係数 = Cf × cos(α_eff)

### 失速モデル

失速領域では、法線力係数は平板モデルに移行します：

```
Cn = Cf_90 × sin(α_eff) × (1/(0.56 + 0.44×|sin(α_eff)|) - 0.41×(1 - exp(-17/AR)))
```

### フラップ効果

制御面の偏角は以下を変化させます：
1. **ゼロ揚力角**: 正の偏角で下方にシフト
2. **最大揚力係数**: フラップ比率により変化
3. **失速角**: 揚力係数変化に基づき調整

大きな偏角では効果が減少（小角度で80%、±50°で40%）。

---

## 制御面の設定

### 制御入力タイプ

| タイプ | 説明 | 代表的なサーフェス |
|--------|------|-------------------|
| `pitch` | 機首上げ/下げ制御 | エレベーター、水平尾翼 |
| `roll` | 左/右バンク制御 | エルロン、差動翼 |
| `yaw` | 機首左/右制御 | ラダー、垂直尾翼 |
| `flap` | 揚力増強 | フラップ、スラット |

### 入力乗数

`multiplier`を使用して差動制御を作成：

```xml
<!-- 左エルロン: 正の制御で右ロール -->
<control_surface type="roll" multiplier="1"/>

<!-- 右エルロン: 負の制御で右ロール -->
<control_surface type="roll" multiplier="-1"/>
```

### プログラムによる制御

```csharp
// AerodynamicsControllerを取得
var aeroController = robot.GetComponent<AerodynamicsController>();

// 制御入力を設定（全て-1〜1、flapのみ0〜1）
aeroController.SetControlInputs(
    pitch: 0.5f,   // 機首上げ
    roll: -0.3f,   // 左バンク
    yaw: 0f,       // ニュートラル
    flap: 0f       // フラップなし
);

// 推力を設定
aeroController.ThrustPercent = 0.8f;
```

---

## 使用例

### 例1: シンプルな固定翼機

```xml
<?xml version="1.0"?>
<robot name="simple_aircraft">
  <!-- デフォルト空力学特性 -->
  <aerodynamics>
    <fluid_density>1.225</fluid_density>
  </aerodynamics>

  <!-- 主翼セクション -->
  <link name="left_wing">
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.1"/>
    </inertial>
    <collision>
      <aerodynamic_surface>
        <chord>1.5</chord>
        <span>3.0</span>
        <lift_slope>6.28</lift_slope>
        <zero_lift_aoa>-2</zero_lift_aoa>
        <stall_angle_high>15</stall_angle_high>
        <stall_angle_low>-15</stall_angle_low>
        <skin_friction>0.02</skin_friction>
        <flap_fraction>0.2</flap_fraction>
        <control_surface type="roll" multiplier="1"/>
      </aerodynamic_surface>
      <geometry>
        <box size="1.5 0.1 3.0"/>
      </geometry>
    </collision>
  </link>

  <link name="right_wing">
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.1"/>
    </inertial>
    <collision>
      <aerodynamic_surface>
        <chord>1.5</chord>
        <span>3.0</span>
        <lift_slope>6.28</lift_slope>
        <zero_lift_aoa>-2</zero_lift_aoa>
        <stall_angle_high>15</stall_angle_high>
        <stall_angle_low>-15</stall_angle_low>
        <skin_friction>0.02</skin_friction>
        <flap_fraction>0.2</flap_fraction>
        <control_surface type="roll" multiplier="-1"/>
      </aerodynamic_surface>
      <geometry>
        <box size="1.5 0.1 3.0"/>
      </geometry>
    </collision>
  </link>

  <!-- エレベーター付き水平尾翼 -->
  <link name="h_stabilizer">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.05"/>
    </inertial>
    <collision>
      <aerodynamic_surface>
        <chord>0.8</chord>
        <span>1.5</span>
        <lift_slope>6.28</lift_slope>
        <zero_lift_aoa>0</zero_lift_aoa>
        <stall_angle_high>15</stall_angle_high>
        <stall_angle_low>-15</stall_angle_low>
        <skin_friction>0.02</skin_friction>
        <flap_fraction>0.4</flap_fraction>
        <control_surface type="pitch" multiplier="1"/>
      </aerodynamic_surface>
      <geometry>
        <box size="0.8 0.05 1.5"/>
      </geometry>
    </collision>
  </link>

  <!-- ラダー付き垂直尾翼 -->
  <link name="v_stabilizer">
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.02"/>
    </inertial>
    <collision>
      <aerodynamic_surface>
        <chord>0.8</chord>
        <span>1.0</span>
        <lift_slope>6.28</lift_slope>
        <zero_lift_aoa>0</zero_lift_aoa>
        <stall_angle_high>15</stall_angle_high>
        <stall_angle_low>-15</stall_angle_low>
        <skin_friction>0.02</skin_friction>
        <flap_fraction>0.35</flap_fraction>
        <control_surface type="yaw" multiplier="1"/>
      </aerodynamic_surface>
      <geometry>
        <box size="0.8 1.0 0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- 胴体（抗力のみ） -->
  <link name="fuselage">
    <inertial>
      <mass value="20.0"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="5" iyz="0" izz="5"/>
    </inertial>
    <collision>
      <aerodynamic_surface>
        <chord>3.0</chord>
        <span>0.5</span>
        <aspect_ratio>0.5</aspect_ratio>
        <lift_slope>0</lift_slope>
        <stall_angle_high>0</stall_angle_high>
        <stall_angle_low>0</stall_angle_low>
        <skin_friction>0.04</skin_friction>
      </aerodynamic_surface>
      <geometry>
        <cylinder radius="0.25" length="3.0"/>
      </geometry>
    </collision>
  </link>
</robot>
```

### 例2: 水中グライダー（ハイドロフォイル）

```xml
<?xml version="1.0"?>
<robot name="underwater_glider">
  <!-- 水中環境 -->
  <aerodynamics>
    <fluid_medium>water</fluid_medium>
    <water_density>1027</water_density>
  </aerodynamics>

  <link name="main_wing">
    <collision>
      <aerodynamic_surface>
        <chord>0.3</chord>
        <span>1.0</span>
        <lift_slope>6.28</lift_slope>
        <zero_lift_aoa>0</zero_lift_aoa>
        <stall_angle_high_water>12</stall_angle_high_water>
        <stall_angle_low_water>-12</stall_angle_low_water>
        <skin_friction_water>0.01</skin_friction_water>
        <fluid_medium>water</fluid_medium>
      </aerodynamic_surface>
      <geometry>
        <box size="0.3 0.02 1.0"/>
      </geometry>
    </collision>
  </link>
</robot>
```

### 例3: 水陸両用車両（自動検出）

```xml
<?xml version="1.0"?>
<robot name="amphibious_vehicle">
  <!-- 二重媒質運用のデフォルト空力パラメータ -->
  <aerodynamics>
    <fluid_medium>auto</fluid_medium>
    <air_density>1.225</air_density>
    <water_density>1027</water_density>
  </aerodynamics>

  <link name="wing">
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.05"/>
    </inertial>
    <collision>
      <aerodynamic_surface>
        <chord>0.5</chord>
        <span>1.0</span>
        <lift_slope>6.28</lift_slope>
        <zero_lift_aoa>0</zero_lift_aoa>
        <!-- 媒質別の失速角 -->
        <stall_angle_high_air>15</stall_angle_high_air>
        <stall_angle_low_air>-15</stall_angle_low_air>
        <stall_angle_high_water>12</stall_angle_high_water>
        <stall_angle_low_water>-12</stall_angle_low_water>
        <!-- 媒質別の表面摩擦 -->
        <skin_friction_air>0.02</skin_friction_air>
        <skin_friction_water>0.01</skin_friction_water>
        <!-- 水面に基づいて媒質を自動検出 -->
        <fluid_medium>auto</fluid_medium>
        <!-- 水中時のキャビテーション効果を有効化 -->
        <enable_cavitation>true</enable_cavitation>
        <cavitation_threshold>0.5</cavitation_threshold>
        <control_surface type="roll" multiplier="1"/>
      </aerodynamic_surface>
      <geometry>
        <box size="0.5 0.05 1.0"/>
      </geometry>
    </collision>
  </link>
</robot>
```

---

## ベストプラクティス

1. **デフォルト値から開始:** lift_slope=6.28を使用し、観察された挙動に基づいて調整。

2. **ジオメトリをコリジョンに合わせる:** コードとスパンはビジュアル/コリジョンジオメトリを近似すべき。

3. **適切な失速角を使用:**
   - 従来の翼型: 12-16°
   - 薄翼: 8-12°
   - 水中フィン: 10-14°

4. **制御面のサイズ:**
   - エルロン: flap_fraction 0.2-0.3
   - エレベーター: flap_fraction 0.3-0.4
   - ラダー: flap_fraction 0.3-0.4

5. **水中使用時:**
   - `skin_friction_water`を使用（流れが滑らかなため低め）
   - `stall_angle_*_water`を使用（レイノルズ数が高いため空気中の約80%）
   - `water_density`を海水なら~1027 kg/m³に設定

6. **水陸両用/二重媒質車両:**
   - `fluid_medium`を"auto"に設定して自動検出
   - `*_air`と`*_water`サフィックスで空気と水のパラメータを別々に指定
   - 高速水中運用ではキャビテーションを有効化を検討
   - システムは遷移時（部分的に水没）にパラメータを自動補間

---

## トラブルシューティング

### 機体が揚力を発生しない
- lift_slope > 0 を確認
- 十分な対気速度があることを確認
- サーフェスの向きが正しいことを確認（揚力はコードに垂直）

### 飛行が不安定
- 制御面の感度を下げる
- 安定性のために尾翼面積を増やす
- 重心位置と空力中心の関係を確認

### 抗力が過大
- skin_frictionを減少
- アスペクト比を確認（高ARは誘導抗力が小さい）
- 失速角が低すぎないか確認

### 失速挙動が急激すぎる
- 失速遷移パディングは自動（5-15°）
- 翼型タイプに対して失速角が現実的か確認

---

## 参考文献

- Khan, W. & Nahon, M. (2015). Real-time modeling of agile fixed-wing UAV aerodynamics
- Anderson, J.D. (2016). Fundamentals of Aerodynamics
- Aircraft-Physics Unity Package by JoeBrow
