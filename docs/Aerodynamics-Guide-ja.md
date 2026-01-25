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
- [Blade Element理論 (BET)](#blade-element理論-bet)
- [非定常空力効果](#非定常空力効果)
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

  <!-- Blade Element理論（オプション - 精度向上用） -->
  <blade_element_theory>
    <num_elements>8</num_elements>           <!-- スパン方向要素数（1=従来方式） -->
    <induced_velocity_model>prandtl</induced_velocity_model>  <!-- none/prandtl/momentum/simpleWake -->
    <taper_ratio>1.0</taper_ratio>           <!-- 翼端コード/翼根コード -->
    <twist_root>0</twist_root>               <!-- 翼根のツイスト（度） -->
    <twist_tip>-3</twist_tip>                <!-- 翼端のツイスト（度） -->
    <sweep_angle>0</sweep_angle>             <!-- 後退角（度） -->
  </blade_element_theory>

  <!-- 非定常空力学（オプション - 羽ばたき/機動飛行用） -->
  <unsteady_effects enable="true">
    <enable_added_mass>true</enable_added_mass>
    <added_mass_coefficient>0.75</added_mass_coefficient>  <!-- 0.75=楕円翼、1.0=平板 -->
    <enable_wagner_lag>true</enable_wagner_lag>
    <wagner_time_constant>4.0</wagner_time_constant>
  </unsteady_effects>
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

### Blade Element理論パラメータ

| パラメータ | デフォルト | 範囲 | 説明 |
|-----------|-----------|------|------|
| `num_elements` | 1 | 1-32 | スパン方向の要素数（1 = 従来の準定常） |
| `induced_velocity_model` | prandtl | - | 誘導速度モデル: `none`, `prandtl`, `momentum`, `simpleWake` |
| `taper_ratio` | 1.0 | 0.1-1.5 | 翼端コード / 翼根コード（1.0 = 矩形翼） |
| `twist_root` | 0 | - | 翼根の幾何学的ツイスト（度） |
| `twist_tip` | 0 | - | 翼端の幾何学的ツイスト（度）（負 = ウォッシュアウト） |
| `sweep_angle` | 0 | - | 後退角（度）（正 = 後退） |

**誘導速度モデル:**

| モデル | 用途 | 説明 |
|--------|------|------|
| `none` | デバッグ用 | 誘導速度補正なし |
| `prandtl` | 固定翼 | プラントル翼端損失補正（航空機に推奨） |
| `momentum` | 回転翼 | プロペラ/ローター用運動量理論 |
| `simpleWake` | 羽ばたき | 時間遅れを伴う単純ウェイクモデル |

### 非定常空力パラメータ

| パラメータ | デフォルト | 範囲 | 説明 |
|-----------|-----------|------|------|
| `enable`（属性） | false | - | 非定常空力効果を有効化 |
| `enable_added_mass` | true | - | 付加質量（仮想質量）効果を有効化 |
| `added_mass_coefficient` | 0.75 | 0.5-1.5 | 付加質量係数（楕円翼:0.75、平板:1.0） |
| `enable_wagner_lag` | true | - | Wagner関数循環遅れを有効化 |
| `wagner_time_constant` | 4.0 | 1-10 | Wagner時定数乗数 |

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

## Blade Element理論 (BET)

Blade Element理論は翼を複数のスパン方向要素に分割し、各要素を独立して計算することで空力学的精度を向上させます。これにより以下が可能になります：

- **スパン方向荷重分布** - 翼根から翼端までの揚力の変化を捉える
- **翼端損失効果** - 有限翼の誘導抗力を正確にモデリング
- **幾何学的変化** - テーパー、ツイスト、後退角の効果
- **要素ごとの誘導速度** - より正確な渦/ウェイクモデリング

### BETを使用するタイミング

| 構成 | num_elements | モデル |
|------|--------------|--------|
| シンプルな航空機（高速シミュレーション） | 1 | - |
| 固定翼航空機（高精度） | 4-8 | `prandtl` |
| プロペラ/ローター | 8-16 | `momentum` |
| 羽ばたき翼 | 8-16 | `simpleWake` |

### テーパーとツイスト

**テーパー比** はスパン方向のコード変化を定義：
- `taper_ratio = 1.0`: 矩形翼（コード一定）
- `taper_ratio = 0.5`: 翼端コードは翼根コードの半分
- `taper_ratio < 1.0`: テーパー翼（航空機で一般的）

**ツイスト分布**（ウォッシュアウト）：
- `twist_root = 0°, twist_tip = -3°`: 典型的なウォッシュアウト
- ウォッシュアウトは翼端失速を遅らせ、操縦特性を改善
- 翼根と翼端間の線形補間

### 誘導速度モデル

#### Prandtl（固定翼航空機）

翼端損失補正を伴うプラントルの揚力線理論に基づく：

```
F = (2/π) × acos(exp(-f))
f = (AR/2) × (1 - r/R) / (r/R × |sin(φ)|)
```

最適な用途：
- 従来型固定翼航空機
- グライダー
- 高アスペクト比翼

#### Momentum（ローター/プロペラ）

アクチュエータディスク運動量理論に基づく：

```
v_i = √(T / (2 × ρ × A))
```

最適な用途：
- ヘリコプターローター
- マルチコプタープロペラ
- ダクテッドファン

#### SimpleWake（羽ばたき翼）

時間遅れを伴うウェイク効果の準定常近似：

```
v_i(t) = α × v_i_qs + (1-α) × v_i(t-Δt)
```

最適な用途：
- 羽ばたきロボット
- バイオインスパイアドスイマー
- 振動フォイル

---

## 非定常空力効果

急激に変化する飛行条件（羽ばたき、機動飛行）では、非定常効果が重要になります。2つの主要な効果がモデル化されています：

### 付加質量（仮想質量）

物体が流体中で加速すると、周囲の流体も加速しなければなりません。これにより「付加質量」効果が生じます：

```
F_added = -m_added × a
m_added = (π/4) × ρ × c² × b × k
```

ここで：
- `c` = コード長
- `b` = スパン長
- `k` = 付加質量係数（楕円翼:0.75、平板:1.0）
- `a` = 加速度

### Wagner関数（循環遅れ）

迎角が急激に変化すると、循環（および揚力）は瞬時には応答しません。Wagner関数はこの遅れをモデル化：

```
Φ(s) = 1 - 0.165×exp(-0.0455×s) - 0.335×exp(-0.3×s)
```

ここで `s = 2×V×t/c` は移動した半コード数。

**効果：**
- 急激な迎角増加 → 揚力は徐々に上昇（瞬時ではない）
- 急激な機動中の揚力オーバーシュートを低減
- 羽ばたき翼の解析で重要

### 非定常効果を有効にするタイミング

| アプリケーション | 付加質量 | Wagner遅れ |
|------------------|----------|-----------|
| 定常巡航飛行 | 不要 | 不要 |
| 積極的な機動飛行 | 必要 | オプション |
| 羽ばたき翼 | 必要 | 必要 |
| 突風応答 | オプション | 必要 |
| 水中振動フィン | 必要 | 必要 |

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

### 例4: Blade Element理論を使用した固定翼機

```xml
<?xml version="1.0"?>
<robot name="accurate_aircraft">
  <aerodynamics>
    <fluid_density>1.225</fluid_density>
  </aerodynamics>

  <link name="main_wing">
    <collision>
      <aerodynamic_surface>
        <chord>1.5</chord>
        <span>6.0</span>
        <lift_slope>6.28</lift_slope>
        <zero_lift_aoa>-2</zero_lift_aoa>
        <stall_angle_high>15</stall_angle_high>
        <stall_angle_low>-15</stall_angle_low>
        <skin_friction>0.02</skin_friction>
        <flap_fraction>0.2</flap_fraction>
        <control_surface type="roll" multiplier="1"/>

        <!-- 正確なスパン方向分布のためのBlade Element理論 -->
        <blade_element_theory>
          <num_elements>8</num_elements>
          <induced_velocity_model>prandtl</induced_velocity_model>
          <taper_ratio>0.6</taper_ratio>
          <twist_root>2</twist_root>
          <twist_tip>-2</twist_tip>
        </blade_element_theory>
      </aerodynamic_surface>
      <geometry>
        <box size="1.5 0.1 6.0"/>
      </geometry>
    </collision>
  </link>
</robot>
```

### 例5: 羽ばたき翼ロボット

```xml
<?xml version="1.0"?>
<robot name="flapping_wing">
  <aerodynamics>
    <fluid_medium>auto</fluid_medium>
    <air_density>1.225</air_density>
    <water_density>1027</water_density>
  </aerodynamics>

  <link name="left_wing">
    <collision>
      <aerodynamic_surface>
        <chord>0.08</chord>
        <span>0.15</span>
        <lift_slope>5.5</lift_slope>
        <zero_lift_aoa>0</zero_lift_aoa>
        <!-- 動的失速用の高い失速角 -->
        <stall_angle_high_air>25</stall_angle_high_air>
        <stall_angle_low_air>-25</stall_angle_low_air>
        <stall_angle_high_water>20</stall_angle_high_water>
        <stall_angle_low_water>-20</stall_angle_low_water>
        <skin_friction_air>0.015</skin_friction_air>
        <skin_friction_water>0.008</skin_friction_water>

        <!-- 羽ばたき用のBlade Element理論 -->
        <blade_element_theory>
          <num_elements>12</num_elements>
          <induced_velocity_model>simpleWake</induced_velocity_model>
          <taper_ratio>0.4</taper_ratio>
          <twist_root>0</twist_root>
          <twist_tip>-5</twist_tip>
        </blade_element_theory>

        <!-- 羽ばたきには非定常効果が重要 -->
        <unsteady_effects enable="true">
          <enable_added_mass>true</enable_added_mass>
          <added_mass_coefficient>0.75</added_mass_coefficient>
          <enable_wagner_lag>true</enable_wagner_lag>
          <wagner_time_constant>4.0</wagner_time_constant>
        </unsteady_effects>
      </aerodynamic_surface>
      <geometry>
        <box size="0.08 0.005 0.15"/>
      </geometry>
    </collision>
  </link>
</robot>
```

### 例6: 運動量理論を使用したプロペラ

```xml
<?xml version="1.0"?>
<robot name="quadcopter">
  <aerodynamics>
    <fluid_density>1.225</fluid_density>
  </aerodynamics>

  <link name="propeller_1">
    <collision>
      <aerodynamic_surface>
        <chord>0.02</chord>
        <span>0.127</span>  <!-- 5インチプロペラ半径 -->
        <lift_slope>5.7</lift_slope>
        <zero_lift_aoa>0</zero_lift_aoa>
        <stall_angle_high>12</stall_angle_high>
        <stall_angle_low>-12</stall_angle_low>
        <skin_friction>0.02</skin_friction>

        <!-- ローター用のBlade Element理論 -->
        <blade_element_theory>
          <num_elements>16</num_elements>
          <induced_velocity_model>momentum</induced_velocity_model>
          <taper_ratio>0.7</taper_ratio>
          <twist_root>25</twist_root>
          <twist_tip>5</twist_tip>
        </blade_element_theory>
      </aerodynamic_surface>
      <geometry>
        <cylinder radius="0.127" length="0.005"/>
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

7. **Blade Element理論:**
   - シンプルなテスト用に`num_elements=1`から開始し、精度向上のために増加
   - 固定翼航空機には4-8要素を使用（精度とパフォーマンスのバランス良好）
   - ローター/プロペラや羽ばたき翼には8-16要素を使用
   - 適切な誘導速度モデルを選択：
     - `prandtl`：従来型航空機用（翼端損失補正）
     - `momentum`：ローター/プロペラ用（アクチュエータディスク理論）
     - `simpleWake`：羽ばたき/振動運動用
   - 典型的なテーパー比：効率的な翼に0.4-0.8
   - 典型的なウォッシュアウト（ツイスト）：翼根に対して翼端-2°〜-5°

8. **非定常効果:**
   - 急激に変化する条件（羽ばたき、積極的な機動飛行）でのみ有効化
   - 付加質量は低速・高加速度時に最も重要
   - Wagner遅れは迎角が急激に変化する場合に重要
   - 高い`wagner_time_constant` = 遅い揚力応答（遅れが大きい）
   - `added_mass_coefficient`はほとんどの翼型で0.75を維持

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

### BET結果が単一要素と異なる
- これは正常 - BETは単一要素では捉えられないスパン方向効果を捕捉
- テーパー/ツイストのない矩形翼では結果は約5%以内であるべき
- 誘導速度モデルがアプリケーションに適切か確認

### 羽ばたき翼の揚力が遅れている
- Wagner遅れが正しく機能している - 揚力は数コード長にわたって増大
- 遅れが大きすぎる場合は`wagner_time_constant`を減少
- 高周波羽ばたきにはタイムステップが十分小さいことを確認

### プロペラ/ローターの推力が低すぎる
- 誘導速度モデルが`momentum`に設定されているか確認
- ツイスト分布が正しいか確認（翼根で大、翼端で小）
- 半径方向の変化を捕捉するのに十分な要素数を確保（8-16推奨）

---

## 参考文献

- Khan, W. & Nahon, M. (2015). Real-time modeling of agile fixed-wing UAV aerodynamics
- Anderson, J.D. (2016). Fundamentals of Aerodynamics
- Aircraft-Physics Unity Package by JoeBrow
- Leishman, J.G. (2006). Principles of Helicopter Aerodynamics（Blade Element理論）
- Wagner, H. (1925). Über die Entstehung des dynamischen Auftriebes von Tragflügeln
- Jones, R.T. (1938). Operational treatment of the non-uniform lift theory（Wagner関数近似）
- Theodorsen, T. (1935). General theory of aerodynamic instability and the mechanism of flutter
