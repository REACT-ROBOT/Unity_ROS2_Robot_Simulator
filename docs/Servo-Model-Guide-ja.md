# サーボモデル (摩擦・バックラッシ) ガイド

安価なサーボモータで顕著な、ギアボックスの摩擦 (Stribeck摩擦・静止摩擦) とバックラッシ (ガタ) をジョイント単位でモデル化し、URDFから設定する方法を解説します。

## 概要

`<servo_model>` タグをURDFの `<robot>` 直下に記述すると、対象ジョイントに `ServoJointModel` コンポーネントが付与され、以下がシミュレートされます:

1. **仮想モータ回転子** — サーボ内部のPD制御器とギア換算慣性をスクリプト内で高レートに数値積分
2. **Stribeck摩擦** — モータ/ギア側に静止摩擦 τ_s、クーロン摩擦 τ_c、Stribeck速度 ω_s、粘性係数 σ_v を持つ摩擦。速度ゼロ近傍はKarnopp法で正確に固着 (stick-slip) を再現
3. **バックラッシ** — モータ角 θm と関節角 θl の相対変位 Δ に対する連続なデッドゾーンばね (Modelica ElastoBacklash 相当):
   - 伝達トルク τ = K·d(Δ) + D·s(Δ)·(ωm − ωl)
   - d(Δ) = Δ − b·tanh(Δ/b)  (b = ガタ全幅の半分)
   - 場合分けのない連続関数のため、指令の符号反転だけでなく**負荷トルクの方向反転**(例: アームが鉛直を跨ぐ瞬間のガタ飛び移り)も自動的に再現されます

伝達ばねはPhysXの暗黙的 xDrive として実現されます。ただし回転子と関節は物理1ステップ遅れで状態を交換するため、**伝達剛性には離散安定条件による上限があります** (「制約・注意点」参照)。

## URDF構文

```xml
<robot name="my_robot">
  <joint name="arm_joint" type="revolute">
    <!-- 標準URDF: 負荷側の粘性・クーロン摩擦 (従来通り) -->
    <dynamics damping="0.001" friction="0.02"/>
    ...
  </joint>

  <!-- サーボモデル拡張 (単位はすべてSI: rad, N·m。prismaticの場合は m, N) -->
  <servo_model joint="arm_joint">
    <friction static="0.12" dynamic="0.08"
              stribeck_velocity="0.1" viscous="0.005"/>
    <backlash width="0.0175" stiffness="400" damping="0.5"/>
    <motor inertia="2e-3" p_gain="20" d_gain="0.5" torque_limit="2.0"/>
  </servo_model>
</robot>
```

`<servo_model>` が指定されたジョイントへの `/joint_states` 系トピックの指令は、xDrive へ直接書き込まれる代わりに仮想サーボ制御器への指令になります。

## パラメータリファレンス

### `<friction>` (モータ/ギア側のStribeck摩擦)

| 属性 | 意味 | 単位 | 既定値 |
|---|---|---|---|
| `static` | 静止摩擦 (すべり出し) トルク τ_s | N·m | 0 |
| `dynamic` | クーロン (動) 摩擦トルク τ_c | N·m | 0 |
| `stribeck_velocity` | Stribeck速度 ω_s (τ_s→τ_c の遷移速度) | rad/s | 0.1 |
| `viscous` | 粘性摩擦係数 σ_v | N·m/(rad/s) | 0 |

摩擦曲線: τ_f(ω) = τ_c + (τ_s − τ_c)·exp(−(ω/ω_s)²) + σ_v·ω

### `<backlash>` (バックラッシと伝達系)

| 属性 | 意味 | 単位 | 既定値 |
|---|---|---|---|
| `width` | ガタの全幅 (デッドバンド 2b) | rad | 0 |
| `stiffness` | 歯面係合時の伝達剛性 K | N·m/rad | 400 |
| `damping` | 係合時の伝達ダンピング D | N·m/(rad/s) | 0.5 |

`stiffness` の目安: 最大トルクでの歯面たわみ τ_max/K がガタ半幅 b の3〜5割になる程度。
ただし**ランタイム (ビルド済みアプリ) でスポーンしたロボットでは離散安定条件
√(K·(1/Jm + 1/J_load))·Δt ≲ 1 が優先されます** (Δt = Fixed Timestep)。
軽い負荷 (J_load ~ 10⁻³ kg·m² 程度) では K は数 N·m/rad が実用上限で、
これを超えると自励発振します。

注意: デッドゾーンは tanh による滑らかな連続関数のため、歯面の立ち上がりが緩やかで、**観測される実効デッドバンドは名目 `width` より2〜3割狭く**なります (Δ = b の位置で名目剛性の約24%が既に伝達されます)。実測のガタ幅に合わせたい場合は `width` をやや大きめに設定してください。

### `<motor>` (仮想モータ)

| 属性 | 意味 | 単位 | 既定値 |
|---|---|---|---|
| `inertia` | ギア換算した回転子慣性 (ギア比² × 回転子慣性) | kg·m² | 2e-3 |
| `p_gain` | サーボ内部PゲインKp | N·m/rad | URDF `<drive>` のstiffness値 |
| `d_gain` | サーボ内部DゲインKd | N·m/(rad/s) | URDF `<drive>` のdamping値 |
| `torque_limit` | モータトルク上限 | N·m | URDF `<limit effort>` 値 |

省略時の既定値は URDF に書かれた**生の数値**をそのままSI値として引き継ぎます。
ランタイム環境では `<drive>`/`<limit effort>` に後述の ×180/π 補正を掛けて
記述することが多いため、その場合は `p_gain`/`d_gain`/`torque_limit` を
**明示的に指定**してください。

## 典型的な設定例 (ホビーサーボ)

```xml
<servo_model joint="pan_joint">
  <!-- すべり出し 0.1 N·m, 動摩擦 0.06 N·m -->
  <friction static="0.10" dynamic="0.06" stribeck_velocity="0.1" viscous="0.003"/>
  <!-- ガタ 1° (0.0175 rad) -->
  <backlash width="0.0175" stiffness="300" damping="0.5"/>
  <motor inertia="1e-3" torque_limit="1.5"/>
</servo_model>
```

## 検証

単振子による検証結果 (プロット・数値つき) は [サーボモデル検証レポート](Servo-Model-Validation-ja.md) にまとまっています。テスト本体は `Assets/Tests/ServoModelTests/` にあります:

- **Hysteresis** — 無重力+負荷摩擦下の低速三角波指令で、指令-応答平面にガタ幅相当のヒステリシスループが現れること
- **GravityCrossing** — 振子が最下点を跨ぐとき (指令は単調のまま負荷トルクの符号が反転)、伝達たわみ Δ が −b から +b へ遷移すること
- **FrictionCurve** — 一定速度追従時の定常偏差から摩擦-速度曲線を逆算し、設定したStribeck曲線と一致すること
- **StictionHold** — 重力下の位置保持でチャタリング (stick-slip振動) が発生しないこと

## モデルの一般形 (他シミュレータへの移植)

本モデルはUnity/PhysXに依存しない次の連立系として表現できます
(MuJoCo・Gazebo等でも同じ式で実装可能):

**仮想回転子 (ギア出力軸換算):**

```
Jm·ω̇m = τ_servo − τ_f(ωm) − τ_t
τ_servo = clip( Kp·(θ_cmd − θm) + Kd·(ω_cmd − ωm), ±τ_max )
τ_f(ω)  = [τ_c + (τ_s − τ_c)·exp(−(ω/ω_s)²)]·sgn(ω) + σ_v·ω   (Karnopp固着付き)
```

**バックラッシ伝達 (回転子 → 負荷関節):**

```
Δ    = θm − θl,   b = width/2
d(Δ) = Δ − b·tanh(Δ/b)
s(Δ) = tanh²(Δ/b)
τ_t  = K·d(Δ) + D·max(s(Δ), 0.1)·(ωm − ωl)      ← 負荷関節に +τ_t を印加
```

Unity実装ではτ_tの印加をPhysXの暗黙的xDrive (stiffness=K, target=θl+d(Δ),
targetVelocity=ωm) に写像していますが、これは実現手段にすぎません。

MuJoCoへの対応の目安:
- 回転子慣性 Jm → `armature` (ただしバックラッシと組み合わせる場合は
  回転子を独立した1DOF jointとして立てる方が忠実)
- クーロン/静止摩擦 → `frictionloss` (Stribeck形状・Karnopp固着は
  MuJoCoの平滑化摩擦とは特性が異なるため、忠実度が必要なら上式を
  カスタム受動力・プラグインで実装)
- 伝達ばね τ_t → 回転子jointと負荷jointを結ぶカスタム受動力
  (`mjcb_passive` 等)。d(Δ), s(Δ) は上式のまま使えます

## 制約・注意点

- **伝達剛性の離散安定上限**: 回転子と負荷は物理1ステップ遅れで結合される
  ため、結合固有振動数 √(K·(1/Jm+1/J_load)) × Δt が ~1 を超えると自励発振
  します。50 Hz・軽負荷では K=数 N·m/rad が上限です。素の xDrive 位置サーボ
  (servo_model なし) も同じ制限を受けます
- **重力静止姿勢はジョイントリミットから離す**: リミット面上で静止した関節が
  ドライブトルクに応答しなくなるエンジン挙動があり、ランタイムでのリミット
  書き換えによる解除は位置スナップを誘発するため行っていません。仮想回転子は
  リミット範囲内へクランプされ、リミットへ押し付け続ける状態は防止されます
- **エディタとビルド済みプレイヤーの単位差**: Linuxプレイヤーでは回転xDriveの
  stiffness/damping/forceLimit が名目値の π/180 倍で作用することが実測されて
  います (エディタでは名目通り)。ServoJointModel は環境に応じて自動補償します
  が、**素のジョイントの URDF `<drive>`/`<limit effort>` はランタイムでは
  ×180/π を掛けた値で記述**してください
- ランタイムでスポーンしたロボットの `jointVelocity` は静止中でも重力起因の
  バイアス (~τ_g/J·Δt) を含むため、プレイヤーでは位置の有限差分で負荷速度を
  推定します (1ステップの遅延が入る点が上記の剛性上限の一因です)
- ギャップ内の20 ms未満の自由運動 (歯面衝突の高周波成分) は物理ステップで丸められます。より高忠実度が必要な場合は `Fixed Timestep` を小さくしてください
- `/joint_states` の effort には伝達トルク (`driveForce`) が報告されます。これは実機サーボの出力トルク推定値に近い値です
- 負荷側 (関節軸受) の摩擦は従来通り標準URDFの `<dynamics friction>` (PhysXの `jointFriction`) を使用してください
