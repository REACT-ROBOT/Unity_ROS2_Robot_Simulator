# Hydrodynamics

Volume-based buoyancy and drag for `ArticulationBody` links (`HydrodynamicFloatingObject`). See `docs/Hydrodynamics-Guide.md`.

Part of [Unity_ROS2_Robot_Simulator](https://github.com/REACT-ROBOT/Unity_ROS2_Robot_Simulator).
Add to another project via `Packages/manifest.json`:

```json
"com.react-robot.hydrodynamics": "https://github.com/REACT-ROBOT/Unity_ROS2_Robot_Simulator.git?path=/Packages/Hydrodynamics#<commit>"
```

Requires [NaughtyWaterBuoyancy (hijimasa fork)](https://github.com/hijimasa/NaughtyWaterBuoyancy) in the same manifest; git dependencies cannot be declared in `package.json`:

```json
"com.naughtyattributes.waterbuoyancy": "https://github.com/hijimasa/NaughtyWaterBuoyancy.git?path=/Assets/NaughtyWaterBuoyancy#<commit>"
```

---

`ArticulationBody` リンク向けの体積ベース浮力と抗力(`HydrodynamicFloatingObject`)。詳細は `docs/Hydrodynamics-Guide-ja.md`。
