# Unity_ROS2_Robot_Simulator
English | [日本語](README-ja.md)

A Unity-based robot simulator that integrates with ROS2 (Robot Operating System 2), providing a high-fidelity visual environment for robotics simulation and development.

A sample project for a typical two-wheeled mobile robot can be found [here](https://github.com/REACT-ROBOT/Unity_ROS2_sample).

## Overview
This project enables robot simulation in Unity with ROS2 communication capabilities, allowing robotic algorithms to be tested in realistic virtual environments before deployment on physical hardware.

## Features
- Realistic physics simulation using Unity's physics engine
- ROS2 integration for standard robotics communication
- Support for various robot models and sensor types
- Customizable environments for different testing scenarios

## Requirements
- Unity 6000.3.21f1 (verified). 6000.0.47f1 LTS or newer should work
- ROS 2 Humble (Ubuntu 22.04) or Jazzy (Ubuntu 24.04) — both verified
- [ROS-TCP-Connector (hijimasa fork)](https://github.com/hijimasa/ROS-TCP-Connector) — needs the publisher unregistration API
- [ROS-TCP-Endpoint (hijimasa fork)](https://github.com/hijimasa/ROS-TCP-Endpoint) — **upstream will not work**: it does not implement `__remove_subscriber` and drops the TCP connection on receiving it
- [URDF-Importer](https://github.com/Unity-Technologies/URDF-Importer)
- [UnitySensors](https://github.com/Field-Robotics-Japan/UnitySensors)
- [UnitySensorsROS](https://github.com/Field-Robotics-Japan/UnitySensors)
Note: UnitySensors and UnitySensorsROS are used in modifid version now.

## Installation
1. Clone this repository
2. Open the project in Unity
3. (Optional) Build the application.

## Usage
1. Run the built applicaion or the project from Editor.

2. Run TCP Connector from below command.
   ```bash
   ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
   ```

2. Set up your robot model and sensors
   ```bash
   ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{ name: '<YOURROBOTNAME>', allow_renaming: false, entity_resource: { uri: 'file:///your/urdf/path/robot.urdf', resource_string: '' }, entity_namespace: '', initial_pose: { header: { stamp: { sec: 0, nanosec: 0 }, frame_id: '' }, pose: { position: { x: 0.0, y: 0.0, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } } } }"
   ```

3. Start the simulation
   ```bash
   ros2 service call /set_simulation_state simulation_interfaces/srv/SetSimulationState "{ state: { state: 1 } }"
   ```

Note: The services are based on [simulation_interfaces](https://github.com/ros-simulation/simulation_interfaces) **2.1.0**.
Version 2.0.0 folded `uri` and `resource_string` into the `Resource` message (`entity_resource`),
so earlier call syntax will not work.

Ask the simulator what it supports with `get_simulator_features`:

```bash
ros2 service call /get_simulator_features simulation_interfaces/srv/GetSimulatorFeatures "{}"
```

Use `spawn_entities` to spawn several at once (`spawn_entity` is deprecated as of 2.0.0). When
spawning more than one entity from the same URDF, pass `entity_namespace` — the topic names baked
into the URDF would otherwise collide.

All 22 services defined in the srv directory are implemented, including entity queries and
edits, `step_simulation`, and world loading. For what each one means here, why worlds map to
scene JSON, and how to configure `get_spawnables` and named poses through
`simulation_resources.json`, see
[docs/Simulation-Interfaces-Services.md](docs/Simulation-Interfaces-Services.md).

## Simulated time

The simulator publishes its simulation clock on `/clock` (`rosgraph_msgs/msg/Clock`, up to
100 Hz; the actual cadence is capped by the application frame rate, which starts at 10 FPS and
can be raised in the UI), so ROS 2 nodes launched with `use_sim_time` follow the simulator. Publishing keeps
going while the simulation is stopped or paused — the value simply freezes, as in Gazebo — and
`reset_simulation` with `SCOPE_TIME` puts it back to zero. All message stamps published here
(`/joint_states`, `/ground_truth`, `/tf`) come from this same clock.

## Verifying the services

A conformance suite connects to a running simulator and checks that these services behave as
specified — in particular that `reset_simulation` restores the initial state and that the robot
keeps accepting commands afterwards.

```bash
# inside the Unity_ROS2_sample container
cd ~/colcon_ws && ./scripts/service_conformance_test.sh
```

See [docs/Service-Conformance-Test.md](docs/Service-Conformance-Test.md) for details.

## URDF extensions

Friction is set through the custom `<collision_material>` element. Syntax, the `combine`
pitfall, and why a robot slides at speed (with measurements) are in
[docs/URDF-Collision-Material.md](docs/URDF-Collision-Material.md).

## Known limitations

Work that is deliberately deferred, and the limits accepted as part of the design, are
collected in [docs/Known-Limitations.md](docs/Known-Limitations.md). It records why each
one stands and what you would need to decide before picking it up, so start there when a
behaviour looks surprising.

## License
This project is licensed under the Apache 2.0 License - see the LICENSE file for details.

## Acknowledgements

I would like to thank the following open-source projects that helped this repository.

* **[dbrizov/NaughtyWaterBuoyancy](https://github.com/dbrizov/NaughtyWaterBuoyancy)**: used for **buoyancy calculations** in this project.
* **[MARUSimulator/marus-core](https://github.com/MARUSimulator/marus-core)**: referenced for approaches on applying **hydrodynamic parameters beyond buoyancy**.
* **[gasgiant/Aircraft-Physics](https://github.com/gasgiant/Aircraft-Physics)**: referenced for approaches on applying **aerodynamic parameters**.
