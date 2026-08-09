# ROS2 Service Conformance Test

An automated suite that connects to a running simulator and checks whether the features exposed
through `simulation_interfaces` services (`spawn_entity` / `set_simulation_state` /
`get_simulation_state` / `reset_simulation` / `step_simulation`) behave as specified.

It was built specifically to reproduce and isolate the reported class of bug where
**the robot stops accepting commands after `reset_simulation` is called**.

The suite lives in the [Unity_ROS2_sample](https://github.com/hijimasa/Unity_ROS2_sample)
repository under `colcon_ws/src/simulation_service_tests`. It drives a prebuilt simulator binary,
so nothing in this repository needs to change to run it.

## Running

Run it **inside** the Unity_ROS2_sample container. Both ROS 2 Humble and Jazzy work as-is —
the scripts read `${ROS_DISTRO}` rather than hardcoding a distro.

```bash
cd ~/colcon_ws
colcon build --packages-select simulation_service_tests simulation_ros2_utils
source install/setup.bash

./scripts/service_conformance_test.sh                       # default (diffbot)
./scripts/service_conformance_test.sh --profile servo_demo  # lighter robot
```

The script brings up the ROS-TCP-Endpoint and the simulator, runs the suite, and tears everything
down. Pass `--no-sim` to attach to an already running stack instead.

> **Note**: when reusing one `colcon_ws` across humble and jazzy, `rm -rf build install log`
> before `colcon build`. The Python versions differ (3.10 / 3.12), so leftovers from the other
> distro make the suite fail to start with
> `UnsupportedTypeSupport: Could not import 'rosidl_typesupport_c'`.

### Verifying your own changes

The suite drives a running player, so after changing this repository you need to rebuild the
player before running it.

```bash
# build a Linux player from this repository
~/Unity/Hub/Editor/6000.2.7f2/Editor/Unity -batchmode -nographics -quit \
  -projectPath . -executeMethod BuildLinuxPlayer.Build \
  -buildOutput /tmp/simbuild/Unity_ROS2_Robot_Simulator.x86_64 -logFile /tmp/build.log

# hand it to the container (docker run ... -v /tmp/simbuild:/home/unity/simbuild:ro) and test
./scripts/service_conformance_test.sh --sim-dir ~/simbuild
```

Exit codes: `0` = everything as expected, `1` = defects found, `2` = could not run.
Use `--junit PATH` to emit JUnit XML for CI.

Outputs (default `/tmp/service_conformance/`):

| File | Contents |
|---|---|
| `<profile>.console.log` | Run log |
| `<profile>.junit.xml` | JUnit XML for CI |
| `<profile>.report.json` | Machine-readable report |
| `Player.log` | The simulator's Unity log — where the actual exceptions land |

## Coverage

| ID | Area |
|----|------|
| A1–A3 | Reachability of the seven core services, initial state, `Result` code convention |
| B1–B3 | State transitions (start / same-state / rejection of invalid values) |
| C1–C6 | Spawn, baseline capture, `ground_truth`, **baseline proof that commands work**, pause/resume |
| D1–D10 | **All `reset_simulation` scopes**: entity survival, joint and pose restoration, command acceptance after reset, service liveness, despawn, respawn, time reset, repeat stability |
| E1–E2 | Despawn on `STATE_STOPPED`, and respawning afterwards |
| F1–F2 | How far `step_simulation` actually advances (`n` versus `2n` steps must come out 2:1), resetting an empty scene |
| G1–G6 | **simulation_interfaces 2.x**: **advertised features cross-checked against the services on the graph**, spawning through `Resource`, `spawn_entities` (batch spawn and partial-failure reporting), topic separation via `entity_namespace`, **spawning from `resource_string`** |
| H1–H9 | **Optional services**: `get_entities` / `get_entity_state` agreeing with `ground_truth`, `set_entity_state`, `entity_info`, `get_entity_bounds`, `EntityFilters`, `delete_entity`, `get_spawnables` / named poses, world lifecycle, **filtering worlds by tag** |

| I1–I2 | **The `simulate_steps` action**: per-step feedback, cancellation mid-run |

The starred scenarios (D5 / D8 / D10) are the direct checks for the reported bug.

Verdicts are `PASS` / `FAIL` / `KNOWN_GAP` (a gap known to be unimplemented; not counted toward
the exit code by default) / `SKIP` / `ERROR`.

## Defects found, and their fixes

Run against the released v0.9.3 binary with both profiles:

```
PASS 19  FAIL 4  KNOWN_GAP 3  SKIP 0  ERROR 0   (diffbot)
PASS 19  FAIL 3  KNOWN_GAP 3  SKIP 1  ERROR 0   (servo_demo)
```

With the fixes below plus the simulation_interfaces 2.1.0 work, current HEAD passes everything on
both profiles (the single `servo_demo` skip is C5, which is meaningless for a fixed-base robot).

```
PASS 31  FAIL 0  KNOWN_GAP 0  SKIP 0  ERROR 0   (diffbot)
PASS 30  FAIL 0  KNOWN_GAP 0  SKIP 1  ERROR 0   (servo_demo)
```

Same results on both ROS 2 Humble (Ubuntu 22.04) and Jazzy (Ubuntu 24.04).

After implementing the remaining 15 services plus `WORLD_TAGS` and the `SimulateSteps`
action, and adding the H group (9 scenarios) and the I group (2 scenarios):

```
PASS 43  FAIL 0  KNOWN_GAP 0  SKIP 0  ERROR 0   (diffbot)
PASS 42  FAIL 0  KNOWN_GAP 0  SKIP 1  ERROR 0   (servo_demo)
```

Confirmed on both ROS 2 Jazzy (Ubuntu 24.04) and Humble (Ubuntu 22.04).

Fewer `simulate_steps` feedback messages may arrive than steps were requested. Feedback
published just before the goal finishes can be overtaken by the result, and the rclpy client
drops that goal's feedback subscription as soon as it has the result. How far the simulation
actually advanced is checked against the sim clock instead, so I1 asserts that progress is
reported incrementally rather than asserting a count.

### FAIL, now fixed: a respawned robot ignores commands (D8 / D10 / E2)

This is where the reported bug reproduced.

```
FAIL  D8   Respawn after despawn, then accept commands
      left_wheel_joint: cmd_vel=+3.000 obs_vel=-0.000 dpos=+0.000 -> NG
      right_wheel_joint: cmd_vel=+3.000 obs_vel=-0.000 dpos=+0.000 -> NG
FAIL  D10  Command acceptance survives repeated resets
      commands stopped working on cycle 1 of 3
```

`Player.log` shows the cause:

```
Publisher for topic /diffbot/joint_states registered twice!
NullReferenceException: Object reference not set to an instance of an object.
  at UnityEngine.ArticulationBody.get_xDrive ()
  at JointStateSub.Callback (RosMessageTypes.Sensor.JointStateMsg msg)
  at Unity.Robotics.ROSTCPConnector.RosTopicState.OnMessageReceived (System.Byte[] data)
```

`SimulationControl.DespawnAllEntities()` only calls `GameObject.Destroy()`; it never unsubscribes
the callback that `JointStateSub.Start()` registered via `ROSConnection.Subscribe()`.
`RosTopicState` accumulates callbacks in a `List` and dispatches with `List.ForEach`, so:

1. the destroyed `JointStateSub`'s callback stays in the list,
2. respawning appends the new callback *behind* it,
3. an incoming command invokes the stale callback first, which throws
   `NullReferenceException` reading `xDrive` on a destroyed `ArticulationBody`,
4. `ForEach` aborts there and **never reaches the new robot's callback**.

**Fix**: `JointStateSub` gained a `DetachFromRos()` that `DespawnAllEntities()` calls
*before* `Destroy()`. It unsubscribes through `ROSConnection.Unsubscribe()` and also sets a flag,
so messages arriving before the unsubscription reaches the endpoint are dropped too.
`LinkThruster`, which subscribes the same way, got the same treatment.

`OnDestroy()` alone is not enough: Unity defers `Destroy()` to the end of the frame, so a
destroyed callback survives for one more frame. Calling it explicitly from the despawn path
makes the ordering deterministic.

> **Endpoint requirement**: `Unsubscribe()` sends the `__remove_subscriber` sys command to the
> ROS-TCP-Endpoint. Upstream ROS-TCP-Endpoint (as of v0.7.0) does not implement it —
> `handle_syscommand`'s `getattr` raises `AttributeError` and **takes down the whole TCP
> connection** (verified: every service goes unresponsive).
> [hijimasa/ROS-TCP-Endpoint](https://github.com/hijimasa/ROS-TCP-Endpoint) implements all four
> unregistration commands and no longer drops the connection on an unknown or malformed command.
> This simulator cannot be paired with the upstream endpoint.

The `registered twice!` warnings from `JointStatePub` / `GroundTruthPub` come from
`ROSConnection` having no publisher-unregister API. `RegisterPublisher` just early-returns on a
duplicate and publishing keeps working, so there is no functional harm — but the warnings bury
real problems in the log, so registration now checks `GetTopic(...).IsPublisher` first.
Publisher release is covered below under "Publisher unregistration".

### FAIL, now fixed: root pose not fully restored by SCOPE_STATE (D4, diffbot only)

```
FAIL  D4   SCOPE_STATE restores the root pose
      baseline (-0.000, -0.000, +0.000) -> now (+0.080, +0.001, -0.000), off by 0.080 m
```

`ResetAllEntitiesState()` restored position via `TeleportRoot()` but left joint velocities
untouched, so the wheels were still spinning when the robot teleported and it immediately drove
away again. (The test reports the post-reset joint velocity — +2.9 rad/s — alongside the offset,
which is what distinguishes "never restored" from "restored, then drove off".) Same root cause as
D3 below.

**Fix**: covered by the D3 fix, plus zeroing the root's `linearVelocity` / `angularVelocity`
after `TeleportRoot()`.

### KNOWN_GAP, now fixed: joint state not restored by SCOPE_STATE (D3)

```
KNOWN_GAP  D3  worst offset right_wheel_joint by 26.110 rad
```

`ResetAllEntitiesState()` touched only the root `transform` and `TeleportRoot()`; it never
reinitialized `jointPosition` / `jointVelocity` / `jointForce` on the child `ArticulationBody`
components.

**Fix**: the zeroing `SpawnEntity()` already did at spawn time is now factored out as
`ResetArticulationState()` and called from both spawn and reset. It additionally:

- resets `xDrive.target` / `targetVelocity` to 0 — otherwise the joint snaps back to the last
  commanded position on the physics step right after being zeroed;
- calls `ServoJointModel.ResetState()` to clear rotor angle, transmission deflection and
  command — otherwise wound-up torque is released the instant the reset lands.

Joints are restored *before* the root is teleported; the other order lets the solver run one step
with the joints already displaced.

### KNOWN_GAP, now fixed: SCOPE_TIME unimplemented (D9)

The `SCOPE_TIME` branch in `SimulationControl.ResetSimulation()` was still `// TODO: Reset Time`.

**Fix**: `Time.timeAsDouble` cannot be rewound, so `Clock` now carries an origin
(`s_TimeOrigin`) that `Clock.ResetTime()` advances to the current time. Every published sim
timestamp subtracts it.

### KNOWN_GAP, now fixed: Result code convention (A3)

`Result.msg` defines success as `RESULT_OK = 1`; `0` means `RESULT_FEATURE_UNSUPPORTED`.
Every service returned the default `0` on success.

**Fix**: `get_simulation_state` / `set_simulation_state` / `reset_simulation` / `spawn_entity`
now return `RESULT_OK`. The client side (`simulation_ros2_utils`) accepts both `0` and `1` so it
still works against older simulators.

## Publisher unregistration

Publishers are now released on despawn, the same as subscribers. Before the fix, a despawned
robot's topics stayed in `ros2 topic list` for the lifetime of the connection.

```
                            after despawn (before)   after despawn (after)
/diffbot/joint_command      gone                     gone
/diffbot/joint_states       still there              gone
/diffbot/lidar_link/scan    still there              gone
/diffbot/camera_link/...    still there              gone
/ground_truth               still there              gone
```

It took changes in three places.

**1. A public API in ROS-TCP-Connector**

`InternalAPI.SendPublisherUnregistration()` already existed but nothing called it, and neither
`ROSConnection` nor `RosTopicState` exposed an entry point — there was no counterpart to the
subscriber's `Unsubscribe()`. Added `RosTopicState.UnregisterPublisher()` and
`ROSConnection.UnregisterPublisher(topic)`.

**2. `__remove_publisher` in ROS-TCP-Endpoint**

Implemented alongside the other three unregistration commands.

**3. Per-entity topic bookkeeping in the simulator**

The sensor publishers are UnitySensors classes, so they cannot be given a release method. Instead
`SimulationControl` records the topics it registers for each entity at spawn time
(`TrackPublishedTopic`) and releases them on despawn (`ReleasePublishedTopics`). `/tf` and
`/ground_truth` are published under the same name by every robot, so they are reference counted
and only released when the last entity using them goes away.

Despawn calls `entity.SetActive(false)` before releasing, to stop `Update()` on the subtree.
Skipping that lets UnitySensors' `RosMsgPublisher.Update()` — which lazily registers the topic if
it is not registered — re-register in the same frame, because `Destroy()` is deferred to the end
of the frame and the components are still alive.

## simulation_interfaces 2.1.0

Updated from 1.0.0. The jump includes breaking changes, so clients built against anything before
2.0.0 will not interoperate.

### What changed in the interfaces

| Change | Detail |
|---|---|
| `Resource` message | `SpawnEntity`'s `uri` / `resource_string` folded into `entity_resource` (2.0.0, breaking) |
| `SpawnEntities` added | Spawn several entities in one call; `SpawnEntity` is deprecated |
| `SpawnResult` added | Per-entity spawn outcome with detail codes 101-109 |
| World services | `LoadWorld` / `UnloadWorld` / `GetCurrentWorld` / `GetAvailableWorlds` |
| `SimulationState` | Added `STATE_NO_WORLD` (4) and `STATE_LOADING_WORLD` (5) |
| `SetEntityState` | Added `set_pose` / `set_twist` / `set_acceleration` flags |

### What the simulator does about it

- **Regenerating the C# messages**: `Assets/Editor/GenerateRosMessages.cs` rebuilds
  `Assets/RosMessages` from the definitions without walking the editor menus.

  ```bash
  Unity -batchmode -nographics -quit -projectPath . \
    -executeMethod GenerateRosMessages.Generate \
    -messageInput ../Unity_ROS2_sample/colcon_ws/src/simulation_interfaces
  ```

- **`spawn_entity`** now reads `entity_resource.uri`, and its result codes follow the spec:
  no resource at all gives `NO_RESOURCE`, a missing file gives `MISSING_ASSETS`, and asking for
  `resource_string` gives `UNSUPPORTED_FORMAT`.
- **`spawn_entities`** is implemented. A single failure makes the overall result
  `ENTITIES_SPAWN_FAILED` while each outcome lands in `results[i]`; remaining requests are still
  attempted, so the caller can see exactly what got created.
- **Entity name uniqueness**: the requested name used to be ignored in favour of the URDF's robot
  name, so two spawns of the same URDF produced two entities with the same name publishing to the
  same topics. The spec's rule now applies — the resource's name is only the fallback for an empty
  request — and a collision returns `NAME_NOT_UNIQUE` unless `allow_renaming` is set, in which case
  a suffix is appended.
- **`entity_namespace`** is implemented. Topic names in a URDF's `ros2_control` block are fixed by
  the resource, so without a namespace two entities from one URDF collide.
- **`get_simulator_features`** is implemented and advertises **only what is actually implemented**
  (G1 fails the run if an unimplemented feature is advertised).

### Not supported

Spawning from `resource_string` (`SPAWNING_RESOURCE_STRING`) is unsupported: a URDF's mesh
references resolve relative to the URDF file, so a bare string leaves the assets unfindable.

> **Since changed**: the four world services (`LoadWorld` / `UnloadWorld` / `GetCurrentWorld` /
> `GetAvailableWorlds`) and `step_simulation`, both listed here as unimplemented, are now
> implemented. The entity services (`GetEntities` / `GetEntityState` / `SetEntityState` /
> `DeleteEntity` and friends), `GetSpawnables` and the named-pose pair went in at the same time,
> so all 22 services defined in the srv directory are covered. See
> [docs/Simulation-Interfaces-Services.md](Simulation-Interfaces-Services.md) for the semantics
> and configuration.
>
> The suite in Unity_ROS2_sample has been updated to match. **F1** now measures how far
> stepping actually advances the clock (`n` versus `2n` steps must come out 2:1), and **G1**
> no longer hard-codes a list of "features that should be unimplemented" — it cross-checks
> the advertised features against the services present on the ROS graph. The 15 newly
> implemented services are covered by **H1–H9**.

## Assumptions to know when writing tests

Simulator behaviors that shape how the suite is built:

- **Topics stop entirely while paused.** Unity's `FixedUpdate` does not run while
  `Time.timeScale = 0`, so neither `joint_states` nor `ground_truth` publishes in `STOPPED` or
  `PAUSED`. Any check of "is the entity still alive?" via topic traffic must first switch to
  `PLAYING` — that is why the despawn checks (D7 / E1) call `play()` first.
- **`joint_states` timestamps are sim time.** They freeze while paused, so they cannot drive
  timeouts. The harness uses wall clock (`time.monotonic()`) throughout.
- **Commands must be sent continuously.** `JointStateSub` merely copies incoming values into
  `xDrive.target` / `targetVelocity`, so the harness does the periodic publishing that
  ros2_control would normally do.
- **ros2_control is bypassed.** The harness publishes straight to `joint_command` and reads
  `joint_states` directly. Without controller_manager and spawners in the path, a failure points
  at the simulator.

## Reproducing by hand

```bash
# spawn a robot and drive it
ros2 launch unity_diffbot_sim diffbot_spawn.launch.py

# reset (scope: default | time | state | spawned | all)
ros2 run simulation_ros2_utils reset_simulation --ros-args -p scope:=state

# resume and see whether commands get through
ros2 run simulation_ros2_utils set_sim_state --ros-args -p set_state:=start
```
