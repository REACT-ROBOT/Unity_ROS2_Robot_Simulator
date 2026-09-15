# Aerodynamics

Aero surfaces (`AeroSurface`) and blade-element propellers for `ArticulationBody` links. See `docs/Aerodynamics-Guide.md`.

Part of [Unity_ROS2_Robot_Simulator](https://github.com/REACT-ROBOT/Unity_ROS2_Robot_Simulator).
Add to another project via `Packages/manifest.json`:

```json
"com.react-robot.aerodynamics": "https://github.com/REACT-ROBOT/Unity_ROS2_Robot_Simulator.git?path=/Packages/Aerodynamics#<commit>"
```

Requires `com.react-robot.hydrodynamics` (same repository, `?path=/Packages/Hydrodynamics`) and, through it, NaughtyWaterBuoyancy in the same manifest.

---

`ArticulationBody` リンク向けの翼面(`AeroSurface`)と翼素理論プロペラ。詳細は `docs/Aerodynamics-Guide-ja.md`。
