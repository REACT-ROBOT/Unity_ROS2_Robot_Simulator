# Servo Joint Model

Friction, backlash and transmission model for `ArticulationBody` joints (`ServoJointModel`). Validated against a single pendulum; see `docs/Servo-Model-Guide.md` and `docs/Servo-Model-Validation.md` in the repository.

Part of [Unity_ROS2_Robot_Simulator](https://github.com/REACT-ROBOT/Unity_ROS2_Robot_Simulator).
Add to another project via `Packages/manifest.json`:

```json
"com.react-robot.servo-model": "https://github.com/REACT-ROBOT/Unity_ROS2_Robot_Simulator.git?path=/Packages/ServoModel#<commit>"
```

No additional dependencies.

---

`ArticulationBody` 関節向けの摩擦・バックラッシ・伝達系モデル(`ServoJointModel`)。単振子で検証済み。詳細はリポジトリの `docs/Servo-Model-Guide-ja.md` と `docs/Servo-Model-Validation-ja.md`。
