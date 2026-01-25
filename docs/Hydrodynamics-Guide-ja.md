# 水力学モデリングガイド

Unity ROS2 Robot Simulatorで水中・水上ロボットモデルの水力学特性を設定する方法を解説します。

## 概要

本シミュレータは2段階の水中物理モデルをサポートしています：
1. **基本浮力** - `buoyancy_material`によるシンプルな浮力モデル（NaughtyWaterBuoyancy）
2. **完全水力学** - 粘性抵抗、圧力抗力、スラミング力、空気抵抗を含む高度な物理モデル（MARUS準拠）

## 目次

- [浮力材料の設定](#浮力材料の設定)
- [水力学パラメータ](#水力学パラメータ)
- [URDF設定例](#urdf設定例)
- [パラメータリファレンス](#パラメータリファレンス)
- [パラメータの導出方法](#パラメータの導出方法)

---

## 浮力材料の設定

### 基本概念

`buoyancy_material`タグは**相対密度（比重）**を定義します。これは材料密度と水密度の比率であり、浮力挙動を決定します：
- `density < 1.0` → 物体は浮く（水より軽い）
- `density = 1.0` → 中性浮力（水と同じ）
- `density > 1.0` → 物体は沈む（水より重い）

### URDF構文

ロボットレベルで材料を定義し、各コリジョン要素で参照します：

```xml
<robot name="my_underwater_robot">
  <!-- 浮力材料の定義（相対密度/比重） -->
  <buoyancy_material name="foam">
    <density value="0.05"/>  <!-- 水の5% - 強い浮力 -->
  </buoyancy_material>

  <buoyancy_material name="aluminum">
    <density value="2.7"/>   <!-- 水の2.7倍 - 沈む -->
  </buoyancy_material>

  <buoyancy_material name="plastic_hull">
    <density value="0.8"/>   <!-- 水の80% - 浮く -->
  </buoyancy_material>

  <link name="body">
    <collision>
      <!-- 材料を参照 -->
      <buoyancy_material name="plastic_hull"/>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>
  </link>
</robot>
```

### 一般的な材料の相対密度（比重）

| 材料 | 相対密度 | 水中での挙動 |
|------|---------|--------------|
| 発泡スチロール (EPS) | 0.015-0.03 | 強い浮力 |
| シンタクチックフォーム | 0.4-0.6 | 中程度の浮力 |
| プラスチック (HDPE) | 0.94-0.97 | わずかな浮力 |
| 水 | 1.0 | 中性浮力 |
| アルミニウム | 2.7 | 沈む |
| 鉄鋼 | 7.85 | 沈む |

**注意:** 相対密度は `材料密度 / 水密度` で計算されます。例えば、絶対密度2700 kg/m³のアルミニウムを水（1000 kg/m³）中で使用する場合、相対密度 = 2.7となります。

---

## 水力学パラメータ

### 水力学の有効化

URDFに`<hydrodynamics>`タグを追加して高度な水力学モデリングを有効にします：

```xml
<robot name="my_auv">
  <!-- ロボットレベルのデフォルト（全リンクに適用） -->
  <hydrodynamics>
    <water_density>1027</water_density>
    <!-- 追加パラメータ... -->
  </hydrodynamics>

  <link name="body">
    <collision>
      <!-- リンク固有のオーバーライド -->
      <hydrodynamics>
        <pressure_drag>
          <C_PD1>15</C_PD1>
        </pressure_drag>
      </hydrodynamics>
    </collision>
  </link>
</robot>
```

### 完全なパラメータ構造

```xml
<hydrodynamics>
  <!-- 流体特性 -->
  <water_density>1027</water_density>     <!-- kg/m³ -->
  <air_density>1.225</air_density>        <!-- kg/m³ -->
  <velocity_reference>1.0</velocity_reference>  <!-- m/s -->

  <!-- 圧力抗力 (Pressure Drag) -->
  <pressure_drag>
    <C_PD1>10</C_PD1>    <!-- 線形係数 -->
    <C_PD2>10</C_PD2>    <!-- 二次係数 -->
    <f_P>0.5</f_P>       <!-- 減衰指数 (0.1-1.0) -->
  </pressure_drag>

  <!-- 吸引抗力 (Suction Drag) -->
  <suction_drag>
    <C_SD1>10</C_SD1>    <!-- 線形係数 -->
    <C_SD2>10</C_SD2>    <!-- 二次係数 -->
    <f_S>0.5</f_S>       <!-- 減衰指数 (0.1-1.0) -->
  </suction_drag>

  <!-- スラミング力 (水面衝撃力) -->
  <slamming>
    <power>2</power>              <!-- 立ち上がり指数 (≥2) -->
    <max_acceleration>10</max_acceleration>  <!-- m/s² -->
    <multiplier>1.0</multiplier>  <!-- 力のスケーリング -->
  </slamming>

  <!-- 空気抵抗 -->
  <air_resistance>
    <coefficient>0.1</coefficient>
  </air_resistance>
</hydrodynamics>
```

---

## URDF設定例

### 例1: シンプルなROV

```xml
<?xml version="1.0"?>
<robot name="simple_rov">
  <!-- 浮力材料（フロートフォーム用） -->
  <buoyancy_material name="flotation_foam">
    <density value="0.2"/>  <!-- 水の20% - 浮く -->
  </buoyancy_material>

  <buoyancy_material name="rov_body">
    <density value="1.1"/>  <!-- 水の110% - わずかに負の浮力 -->
  </buoyancy_material>

  <!-- 低速ROV向けデフォルト水力学パラメータ -->
  <hydrodynamics>
    <water_density>1027</water_density>
    <velocity_reference>2.0</velocity_reference>
    <pressure_drag>
      <C_PD1>5</C_PD1>
      <C_PD2>5</C_PD2>
      <f_P>0.7</f_P>
    </pressure_drag>
    <suction_drag>
      <C_SD1>5</C_SD1>
      <C_SD2>5</C_SD2>
      <f_S>0.7</f_S>
    </suction_drag>
    <slamming>
      <power>2</power>
      <max_acceleration>5</max_acceleration>
      <multiplier>0.5</multiplier>
    </slamming>
  </hydrodynamics>

  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    <collision>
      <buoyancy_material name="rov_body"/>
      <geometry>
        <box size="0.4 0.3 0.2"/>
      </geometry>
    </collision>
    <visual>
      <geometry>
        <box size="0.4 0.3 0.2"/>
      </geometry>
    </visual>
  </link>

  <link name="flotation">
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <collision>
      <buoyancy_material name="flotation_foam"/>
      <geometry>
        <box size="0.4 0.3 0.05"/>
      </geometry>
    </collision>
  </link>
</robot>
```

### 例2: リンク別水力学パラメータを持つ水上艇

```xml
<?xml version="1.0"?>
<robot name="surface_vessel">
  <buoyancy_material name="hull_material">
    <density value="0.6"/>  <!-- 水の60% - 浮く -->
  </buoyancy_material>

  <!-- デフォルト水力学パラメータ -->
  <hydrodynamics>
    <water_density>1027</water_density>
    <velocity_reference>5.0</velocity_reference>
  </hydrodynamics>

  <link name="hull">
    <collision>
      <buoyancy_material name="hull_material"/>
      <!-- 船体固有: 滑走時の高い圧力抗力 -->
      <hydrodynamics>
        <pressure_drag>
          <C_PD1>15</C_PD1>
          <C_PD2>25</C_PD2>
          <f_P>0.3</f_P>
        </pressure_drag>
        <slamming>
          <power>3</power>
          <max_acceleration>20</max_acceleration>
          <multiplier>1.5</multiplier>
        </slamming>
      </hydrodynamics>
      <geometry>
        <mesh filename="package://my_robot/meshes/hull.dae"/>
      </geometry>
    </collision>
  </link>

  <link name="keel">
    <collision>
      <!-- キール固有: 流線型、低抗力 -->
      <hydrodynamics>
        <pressure_drag>
          <C_PD1>3</C_PD1>
          <C_PD2>5</C_PD2>
          <f_P>0.8</f_P>
        </pressure_drag>
      </hydrodynamics>
      <geometry>
        <box size="0.02 0.5 0.3"/>
      </geometry>
    </collision>
  </link>
</robot>
```

---

## パラメータリファレンス

### 力の計算式一覧

| 力の種類 | 数式 | 説明 |
|----------|------|------|
| 浮力 | F = ρ × g × V | ボクセルベース、波面追従 |
| 粘性水抵抗 | F = 0.5 × ρ × v² × S × Cf | Reynolds数ベースの摩擦抗力 |
| 圧力抗力 | F = -(C₁v + C₂v²) × A × cos^f(θ) × n | 流入角度依存 |
| スラミング力 | F = clamp(a/a_max)^p × cos(θ) × F_stop | 水面衝突時の衝撃 |
| 空気抵抗 | F = 0.5 × ρ_air × v² × A × Cd | 水上部分のみ |

### 流体特性

| パラメータ | デフォルト | 単位 | 説明 |
|-----------|-----------|------|------|
| `water_density` | 1027 | kg/m³ | 海水密度（淡水は1000を使用） |
| `air_density` | 1.225 | kg/m³ | 海面レベル、15℃での空気密度 |
| `velocity_reference` | 1.0 | m/s | 正規化抗力計算の基準速度 |

### 圧力抗力パラメータ

圧力抗力は、表面が流れの方向に面しているときに運動に抵抗します。

| パラメータ | デフォルト | 範囲 | 説明 |
|-----------|-----------|------|------|
| `C_PD1` | 10 | 0-100 | 線形圧力抗力係数 |
| `C_PD2` | 10 | 0-100 | 二次圧力抗力係数 |
| `f_P` | 0.5 | 0.1-1.0 | 減衰指数（低いほど角度依存性が急峻） |

**計算式:**
```
F_pressure = -(C_PD1 × v_norm + C_PD2 × v_norm²) × A × cos(θ)^f_P × n
```
ここで：
- `v_norm = |velocity| / velocity_reference`（正規化速度）
- `A` = 表面積
- `θ` = 速度と表面法線の間の角度
- `n` = 表面法線ベクトル

### 吸引抗力パラメータ

吸引抗力は後方を向いた表面（伴流側）で発生します。

| パラメータ | デフォルト | 範囲 | 説明 |
|-----------|-----------|------|------|
| `C_SD1` | 10 | 0-100 | 線形吸引抗力係数 |
| `C_SD2` | 10 | 0-100 | 二次吸引抗力係数 |
| `f_S` | 0.5 | 0.1-1.0 | 減衰指数 |

### スラミング力パラメータ

スラミング力は水面突入時の衝撃（波切り、水しぶき衝撃）をシミュレートします。

| パラメータ | デフォルト | 範囲 | 説明 |
|-----------|-----------|------|------|
| `power` | 2 | ≥2 | 立ち上がり指数（高いほど急激な衝撃） |
| `max_acceleration` | 10 | >0 | 想定される最大加速度 (m/s²) |
| `multiplier` | 1.0 | 0-5 | 力のスケーリング係数 |

**計算式:**
```
F_slamming = clamp(a / a_max)^p × cos(θ) × F_stop × multiplier
```
ここで：
- `a` = 現在の加速度の大きさ
- `a_max` = max_acceleration パラメータ
- `p` = power パラメータ
- `θ` = 速度と表面法線の間の角度
- `F_stop` = 運動量に基づく停止力の推定値

### 空気抵抗

| パラメータ | デフォルト | 範囲 | 説明 |
|-----------|-----------|------|------|
| `coefficient` | 0.1 | 0-2 | 空気抗力係数 (Cd) |

**計算式:**
```
F_air = 0.5 × ρ_air × v² × A × Cd
```
（速度と反対方向に適用）

---

## パラメータの導出方法

### 圧力抗力係数の推定

1. **CFD解析から:**
   - 基準速度でCFDシミュレーションを実行
   - 表面の圧力分布を抽出
   - `C_PD2 ≈ 2 × (測定抗力) / (ρ × v² × A)`

2. **経験データから:**
   - 平板: `C_PD2 ≈ 1.0-1.3`
   - 流線型ボディ: `C_PD2 ≈ 0.04-0.1`
   - 鈍頭物体（箱型）: `C_PD2 ≈ 1.0-2.1`

3. **経験則:**
   ```
   C_PD1 = C_PD2 / velocity_reference  （低速での線形挙動用）
   ```

### 減衰指数 (f_P, f_S) の選択

- **f = 0.3-0.5:** 急峻な角度依存性、平面に適切
- **f = 0.5-0.7:** 中程度、ほとんどの曲面船体に適切
- **f = 0.7-1.0:** 緩やかな減衰、流線型形状に適切

### スラミング力の調整

1. **power パラメータ:**
   - `power = 2`: 緩やかな立ち上がり、低速水面突入用
   - `power = 3-4`: 中程度、一般的なボート
   - `power = 5+`: 急激な衝撃、高速滑走艇

2. **max_acceleration:**
   - ROV/AUV: 5-10 m/s²
   - 水上艇: 10-20 m/s²
   - 高速ボート: 20-50 m/s²

3. **multiplier の調整:**
   - 1.0 から開始
   - 衝撃が弱すぎる場合は増加
   - シミュレーションが不安定な場合は減少

### 粘性抵抗（内蔵）

シミュレータはITTC 1957摩擦公式を使用して粘性抵抗を自動計算します：

**力の計算式:**
```
F_viscous = 0.5 × ρ × v² × S × Cf
```

**摩擦係数 (ITTC 1957):**
```
Cf = 0.075 / (log₁₀(Rn) - 2)²
Rn = v × L / ν
```

ここで：
- `ρ` = 水密度
- `v` = 流速
- `S` = 濡れ面積
- `Rn` = レイノルズ数
- `L` = 代表長さ
- `ν` = 動粘性係数（20℃の水で1×10⁻⁶ m²/s）

---

## ベストプラクティス

1. **シンプルから始める:** デフォルトパラメータから開始し、観察された挙動に基づいて調整する。

2. **複雑なロボットにはリンク別パラメータを使用:**
   - ロボットレベルのデフォルトを定義
   - 特定のリンクで異なるパラメータのみをオーバーライド

3. **実データで検証:**
   - 可能であれば、シミュレートされた抗力を実験データやCFDデータと比較
   - 測定された力に合わせて係数を調整

4. **運用条件を考慮:**
   - 淡水 vs 海水の密度
   - 温度は粘性に影響
   - 深度は浮力にわずかに影響（水の圧縮性）

5. **デバッグ可視化:**
   - `HydrodynamicFloatingObject`のInspectorで`showDebugForces`を有効化
   - `showDebugVoxels`でボクセルを可視化

---

## トラブルシューティング

### ロボットが速く沈みすぎる
- 浮力材料の体積を増やすか密度を下げる
- `buoyancy_material`が正しく参照されているか確認

### 水面での過度な振動
- ダンピングを増加（Inspectorで`linearDampingInWater`を調整）
- `slammingMultiplier`を減少
- ボクセル解像度（`normalizedVoxelSize`）を確認

### 非現実的な抗力
- `velocity_reference`が予想される運用速度と一致しているか確認
- 形状に基づいて`C_PD1`/`C_PD2`を調整
- 表面積の計算（メッシュ vs ボックス近似）を確認

### シミュレーションの不安定性
- `slammingMultiplier`と`slammingPower`を減少
- 圧力抗力係数を減少
- 物理タイムステップ解像度を上げる

---

## 参考文献

- MARUS (Marine Robotics Unity Simulator) - LABUST
- ITTC 1957 Friction Line - 国際水槽試験会議
- Fossen, T.I. (2011). Handbook of Marine Craft Hydrodynamics and Motion Control
- NaughtyWaterBuoyancy - Unity Asset Store
