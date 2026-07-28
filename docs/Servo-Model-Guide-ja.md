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

伝達ばねはPhysXの暗黙的 xDrive として実現されるため、50 Hz の物理ステップでも高剛性 K で安定です。

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

注意: デッドゾーンは tanh による滑らかな連続関数のため、歯面の立ち上がりが緩やかで、**観測される実効デッドバンドは名目 `width` より2〜3割狭く**なります (Δ = b の位置で名目剛性の約24%が既に伝達されます)。実測のガタ幅に合わせたい場合は `width` をやや大きめに設定してください。

### `<motor>` (仮想モータ)

| 属性 | 意味 | 単位 | 既定値 |
|---|---|---|---|
| `inertia` | ギア換算した回転子慣性 (ギア比² × 回転子慣性) | kg·m² | 2e-3 |
| `p_gain` | サーボ内部PゲインKp | N·m/rad | URDF `<drive>` のstiffnessをSI換算 |
| `d_gain` | サーボ内部DゲインKd | N·m/(rad/s) | URDF `<drive>` のdampingをSI換算 |
| `torque_limit` | モータトルク上限 | N·m | `<drive>` のforce_limit |

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

## 制約・注意点

- ギャップ内の20 ms未満の自由運動 (歯面衝突の高周波成分) は物理ステップで丸められます。より高忠実度が必要な場合は `Fixed Timestep` を小さくしてください
- `/joint_states` の effort には伝達トルク (`driveForce`) が報告されます。これは実機サーボの出力トルク推定値に近い値です
- 負荷側 (関節軸受) の摩擦は従来通り標準URDFの `<dynamics friction>` (PhysXの `jointFriction`) を使用してください
