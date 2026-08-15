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

### Spawning a robot from the GUI

Robots can also be spawned without ROS. The sidebar's **mesh file button** accepts `.urdf`
alongside the mesh formats: pick a mesh and it becomes a scenery object, pick a URDF and it
goes through the same internal path as `/spawn_entity`, so the robot is a regular entity — it
appears in `get_entities`, publishes `/joint_states` and the other per-robot topics, can be
removed with `delete_entity`, and is despawned by `reset_simulation` with `SCOPE_SPAWNED`.
The entity name comes from the file name (`_1`, `_2`, … appended on collision), with no
namespace. The robot spawns upright on the ground at the first free 1 m step along the ROS x
axis from the origin. `.xacro` files are rejected — expand them first
(`ros2 run xacro xacro robot.urdf.xacro > robot.urdf`). As with the service, spawning a robot
needs a loaded world. `package://` mesh references that do not resolve next to the URDF are
looked up through the search paths in `simulation_resources.json`; failure reasons are written
to the log.

### Entity list and joint sliders

The robot-icon button in the bottom-right button row (next to the add/save/load buttons)
slides open a right-edge panel listing every spawned entity
(same contents as `get_entities`; the list refreshes about once a second). Selecting an entity
shows one slider per movable joint (revolute/prismatic with their URDF limits, continuous
shown as -180…180 deg), labelled with the joint name and current position. Dragging a slider
commands the joint through the same path as a `/joint_states` command (drive target, or the
servo model when the URDF defines one), so manual sliders and ROS commands write the same
target — the last writer wins. While a slider is not being dragged it follows the actual joint
position, so externally commanded motion stays visible.

### Sensor visualization

Below the joint sliders the panel lists the selected robot's visualizable sensor outputs, each
with an On/Off button. Point-cloud outputs (LiDAR scans, depth and RGBD camera clouds) are
overlaid on the 3D view using the visualizers bundled with UnitySensors; camera outputs
(RGB/fisheye/panoramic images and the RGBD color stream) appear as a live preview inside the
panel. Point-cloud overlays stay on when the panel is closed or another entity is selected
(the button state is restored on reselect); image previews live in the panel, so they close
with it. The overlays are drawn on a dedicated layer that sensor cameras do not render, so
the images published to ROS are unaffected by what you visualize. The depth camera's raw
depth image is not previewed (it is a float texture in meters and would only show white) —
use its point cloud instead. For debugging, starting the simulator with
`SIM_AUTO_SENSOR_VIZ=N` (N ≥ 1) selects the first spawned entity, turns every visualization
on, and cycles them off and back on N times (ending enabled) — a headless way to exercise the
attach/detach path.

All 22 services defined in the srv directory are implemented, including entity queries and
edits, `step_simulation`, and world loading. For what each one means here, why worlds map to
scene JSON, and how to configure `get_spawnables` and named poses through
`simulation_resources.json`, see
[docs/Simulation-Interfaces-Services.md](docs/Simulation-Interfaces-Services.md).

### SDF worlds

`load_world` and the GUI's Load button also accept Gazebo SDF world files (`.sdf` /
`.world`). A **static subset** is converted into the same scenery the scene JSON produces:
models (box/cylinder/sphere/plane/mesh geometry, poses composed and converted from Z-up,
diffuse colours), `<include>` with `model://` URIs (resolved from the world file's
directory, a `models/` directory next to it, the `simulation_resources.json` search paths,
and `GZ_SIM_RESOURCE_PATH` / `GAZEBO_MODEL_PATH`), and lights. STL meshes keep the
ROS/Gazebo Z-up convention. Everything is placed as static scenery — dynamic models,
joints, actors, physics settings and plugins are skipped and reported in the response.
`get_available_worlds` lists `.sdf` / `.world` files found in `world_paths` alongside
scene JSON. Details in
[docs/Simulation-Interfaces-Services.md](docs/Simulation-Interfaces-Services.md).

## Simulated time

The simulator publishes its simulation clock on `/clock` (`rosgraph_msgs/msg/Clock`, up to
100 Hz; the actual cadence is capped by the application frame rate, which starts at 10 FPS and
can be raised in the UI), so ROS 2 nodes launched with `use_sim_time` follow the simulator. Publishing keeps
going while the simulation is stopped or paused — the value simply freezes, as in Gazebo — and
`reset_simulation` with `SCOPE_TIME` puts it back to zero. All message stamps published here
(`/joint_states`, `/ground_truth`, `/tf`) come from this same clock.

## Headless mode (CI / reinforcement learning)

The built player runs without a display:

```bash
./Unity_ROS2_Robot_Simulator.x86_64 -batchmode -nographics
```

Everything service- and physics-side works headless: all simulation_interfaces services and
the `simulate_steps` action, `/clock`, joint states/commands, ground truth and TF, and the
raycast-based sensors — lidar, the depth camera (its image is produced on the CPU, so no GPU
is needed), contact, GNSS and IMU. Rendering-dependent sensors (`camera`, `wideanglecamera`,
`panoramiccamera`, `rgbd_camera`) cannot work without a graphics device; they are skipped at
spawn with a warning, so their topics simply do not appear. The GUI point-cloud overlays are
likewise unavailable. On a machine that does have a GPU and a display server (or virtual
display such as Xvfb), run with `-batchmode` alone — no window is shown but rendering still
works, so the camera sensors stay available.

Pacing comes from `simulation_resources.json` (`SIMULATION_RESOURCES_CONFIG`):

```json
{ "settings": { "physics_hz": 100, "target_fps": 500, "time_scale": 10 } }
```

- `target_fps` lifts the 10 FPS default so sensors can publish at their URDF `update_rate`
  (a sensor never updates faster than the application frame rate).
- `time_scale` (0.1–100) multiplies simulated time against the wall clock for both `PLAYING`
  and stepping. Physics still advances in exact `1/physics_hz` increments — only the
  wall-clock speed changes, so `step_simulation` semantics are untouched. For RL, pause the
  simulation and drive it with `step_simulation` / `simulate_steps`; with the settings above,
  stepping runs an order of magnitude faster than real time (CPU permitting).

The conformance suite (below) accepts `--headless` to launch the simulator with
`-batchmode -nographics`, which is how to run it on a machine with no display at all.

## Verifying the services

A conformance suite connects to a running simulator and checks that these services behave as
specified — in particular that `reset_simulation` restores the initial state and that the robot
keeps accepting commands afterwards.

```bash
# inside the Unity_ROS2_sample container
cd ~/colcon_ws && ./scripts/service_conformance_test.sh
```

See [docs/Service-Conformance-Test.md](docs/Service-Conformance-Test.md) for details.

## Writing test scenarios (pytest)

The [simulation_ros2_utils](https://github.com/REACT-ROBOT/simulation_ros2_utils)
repository (a submodule of the companion workspace) ships `sim_test_utils`, a pytest plugin that wraps the
simulator services for scenario tests against a running simulator + endpoint:

```python
def test_avoidance(sim):                       # the `sim` fixture connects & cleans up
    sim.spawn("box", urdf=BOX_URDF, pose=(0, 0, 1))
    sim.play(); sim.pause()
    sim.step(50)                               # deterministic: PAUSED + step_simulation
    assert sim.position("box")[2] < 1.0
    sim.apply_wrench("box", force=(0, 0, 30), duration=0.5)   # disturbance injection
    assert not sim.collided("box")             # collision records (floor filtered out)
```

Build `simulation_extra_interfaces` and `sim_test_utils`
(`colcon build --packages-select simulation_extra_interfaces sim_test_utils`),
restart the endpoint (services register when Unity connects), then run
`python3 -m pytest src/simulation_ros2_utils/sim_test_utils/examples -v`. The fixture deletes spawned
entities, stops the simulation and resets time after every test. For
reproducible physics, spawn while paused and advance with `sim.step()` only.

## Disturbance injection

The custom service `/apply_link_wrench`
(`simulation_extra_interfaces/srv/ApplyLinkWrench`) applies a constant wrench
to a link's center of mass — wind gusts, pushes, impacts — for robustness
testing:

- `entity` / `link` (empty `link` targets the base link)
- `wrench`: force [N] and torque [N·m] in the **world frame, ROS axes**
- `duration`: simulated seconds; `0` means exactly one physics step. Time is
  counted in physics steps, so behaviour is identical under `time_scale` and
  `step_simulation`.

Repeated calls stack; `reset_simulation` clears active wrenches. The service
lives outside `simulation_interfaces`, so it is not listed in
`get_simulator_features` — this is also the practical route to "apply an
acceleration", which `set_entity_state` deliberately does not support.

## Collision monitoring

Every link of every spawned entity is monitored, and the custom service
`/get_contact_events` (`simulation_extra_interfaces/srv/GetContactEvents`)
returns records aggregated per (entity, link, other object): contact count,
first/last simulation time and the largest impulse. The "other" name is the
other entity's name, or the scenery object's name (the built-in floor is
`Plane`) — expected contacts like wheels on the ground are filtered by name on
the caller side (`sim.collided("robot")` in `sim_test_utils` ignores the floor
by default). Records are cleared by `reset_simulation` (SCOPE_STATE) or the
service's `clear` flag.

## Moving obstacles

Scenery objects can patrol a waypoint path as kinematic bodies — they push
robots and generate contacts, and pause/`step_simulation`/`time_scale` all
apply, so avoidance scenarios stay deterministic. Two ways to declare them:

- Scene JSON: an optional `motion` element on any object —
  `{"speed": 1.0, "loop": "loop"|"pingpong", "useTimes": false, "waypoints":
  [{"position": [x,y,z], "yawDeg": 0, "time": 0}, ...]}` (Unity coordinates,
  like the rest of the scene JSON; with `useTimes` the per-waypoint `time` is
  used instead of `speed`).
- SDF worlds: `<actor>` with a `<link>` shape and a
  `<script><trajectory>` — waypoint times and poses are interpolated and
  `<loop>` is honoured (skeletal skin/animation is not supported; the first
  visual shape follows the trajectory).

`reset_simulation` (SCOPE_STATE) puts moving obstacles back to their starting
waypoint so repeated test runs see the same phase.

## URDF extensions

Friction is set through the custom `<collision_material>` element. Syntax, the `combine`
pitfall, and why a robot slides at speed (with measurements) are in
[docs/URDF-Collision-Material.md](docs/URDF-Collision-Material.md).

A GNSS receiver is declared like the other sensors (imu, lidar, camera, contact, …) inside
`<simulation>`:

```xml
<simulation>
  <sensor type="gnss" name="gnss_link">
    <update_rate>10</update_rate>
    <origin_latitude>35.681236</origin_latitude>   <!-- optional; degrees -->
    <origin_longitude>139.767125</origin_longitude><!-- optional; degrees -->
    <origin_altitude>0.0</origin_altitude>         <!-- optional; meters -->
  </sensor>
</simulation>
```

The `origin_*` elements place the Unity world origin on the globe. They configure a single
scene-wide geodetic origin shared by every robot: the first spawned robot that declares a GNSS
sensor wins, and later robots that request a different origin get a warning and use the existing
one. The fix is published as `sensor_msgs/NavSatFix` on `/<robot>/<link>/fix`.

### Joint command modes (effort control)

Joint commands arrive as `sensor_msgs/JointState` on the robot's command topic
(`joint_commands_topic` in `<ros2_control><hardware>`). By default a joint is driven by a
PD drive: `position[]` sets the drive target and `velocity[]` the target velocity.

A joint whose `<ros2_control><joint>` entry declares
`<command_interface name="effort"/>` (and no position/velocity interface) switches to
**torque control**: the PD drive is disabled and the `effort[]` field of the command
message is applied directly as the joint's generalized force (N·m, or N for prismatic
joints), every physics step, until the next command. The value is clamped to the URDF
`<limit effort>` and, if present, the `<param name="max">` of the command interface.
`reset_simulation` zeroes the held torque. The `effort[]` field of the published joint
states echoes the applied torque (for PD-driven joints it reports the drive force as
before). A `<servo_model>` on an effort joint is ignored with a warning, and the GUI
slider for such a joint becomes a read-only position indicator.

The URDF `<limit velocity>` is now applied to every joint as the PhysX joint-velocity
cap; without it, PhysX's default (~12.6 rad/s) silently limits fast wheels and
torque-driven joints.

`<ros2_control><hardware><param name="command_timeout">0.5</param>` enables a
communication watchdog like the one in real motor drivers: if no `/joint_command`
arrives for that many seconds, velocity targets and held torques are zeroed (position
targets stay, matching servo behaviour) until commands resume. Off by default — without
the parameter the simulator keeps the last command forever, as before.

### Joint state and ground truth publishers

Joint state publishers/subscribers are attached only to URDFs that carry a
`<ros2_control>` tag. The ground truth publisher (`geometry_msgs/PoseStamped` plus
`/tf`) is attached only when `<ros2_control><hardware>` also declares
`ground_truth_topic`:

```xml
<ros2_control name="${name}" type="system">
  <hardware>
    <plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
    <param name="joint_commands_topic">/${name}/joint_command</param>
    <param name="joint_states_topic">/${name}/joint_states</param>
    <param name="ground_truth_topic">/${name}/ground_truth</param>
  </hardware>
  ...
</ros2_control>
```

URDFs without `<ros2_control>` — projectiles, props — get no publishers. These used to
be attached regardless of what the URDF contained, but the default topic names are
shared across entities, so spawning many such entities made all of them publish to the
same `/ground_truth`, `/tf` and `/joint_states`. That filled the per-topic ROS-TCP
outgoing queue and the drops took the robot's own state down with them (measured: over
1400 dropped messages per second with 20 such entities).

Declaring `ground_truth_topic` also keeps names apart. The default is shared by every
robot, so spawning several at once puts them all on one topic — give each its own.

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
