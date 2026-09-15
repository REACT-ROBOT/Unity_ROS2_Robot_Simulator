# URDF Properties

Applies URDF extension elements to imported robots (`CollisionMaterialApplier`): collision materials, magnetic guide tape and other simulator-specific tags. See `docs/URDF-Collision-Material.md`.

Part of [Unity_ROS2_Robot_Simulator](https://github.com/REACT-ROBOT/Unity_ROS2_Robot_Simulator).
Add to another project via `Packages/manifest.json`:

```json
"com.react-robot.urdf-properties": "https://github.com/REACT-ROBOT/Unity_ROS2_Robot_Simulator.git?path=/Packages/UrdfProperties#<commit>"
```

Requires [UnitySensors (hijimasa fork)](https://github.com/hijimasa/UnitySensors) in the same manifest; git dependencies cannot be declared in `package.json`:

```json
"com.frj.unity-sensors": "https://github.com/hijimasa/UnitySensors.git?path=/Packages/UnitySensors#<commit>"
```

---

URDF の拡張要素をインポート済みロボットへ適用する(`CollisionMaterialApplier`)。衝突マテリアル、磁気テープなど。詳細は `docs/URDF-Collision-Material-ja.md`。
