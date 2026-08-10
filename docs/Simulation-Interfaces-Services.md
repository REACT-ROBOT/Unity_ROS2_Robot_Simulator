# simulation_interfaces service reference

The services this simulator exposes from
[simulation_interfaces](https://github.com/ros-simulation/simulation_interfaces) **2.1.0**,
and how they are interpreted here. What is implemented always matches what
`get_simulator_features` advertises (the conformance test's G1 check fails the build if the
two drift apart).

```bash
ros2 service call /get_simulator_features simulation_interfaces/srv/GetSimulatorFeatures "{}"
```

## Coverage

All 22 services defined in the srv directory are implemented, plus the `SimulateSteps`
action and every feature flag in `SimulatorFeatures`.

| Group | Service | Notes |
|---|---|---|
| Spawning | `spawn_entity` / `spawn_entities` | Both `uri` and `resource_string`; `spawn_entity` is deprecated since 2.0.0 |
| | `delete_entity` | Despawns a single entity |
| | `get_spawnables` | Scans `spawnable_paths` from the config file |
| Entities | `get_entities` / `get_entities_states` | Filter by name, category, tags, bounds (box / sphere / convex hull) |
| | `get_entity_state` / `set_entity_state` | Pose and twist |
| | `get_entity_info` / `set_entity_info` | Category, description, tags |
| | `get_entity_bounds` | AABB in the canonical link frame |
| Named poses | `get_named_poses` / `get_named_pose_bounds` | Read from the config file |
| Simulation | `get_simulation_state` / `set_simulation_state` | |
| | `reset_simulation` | SCOPE_TIME / STATE / SPAWNED |
| | `step_simulation` | Only while paused |
| | `get_simulator_features` | |
| Worlds | `load_world` / `unload_world` | Scene JSON |
| | `get_current_world` / `get_available_worlds` | Filterable by tag |
| Actions | `simulate_steps` | Only while paused; feedback per step, cancellable |

### Not supported

Every service in the srv directory and every flag in `SimulatorFeatures` is implemented.
- **`acceleration` on `set_entity_state`** — ignored (reading it works). Unity offers no
  direct way to impose an acceleration, and turning one into a force means deciding how to
  treat mass. `EntityState.msg` explicitly allows simulators to ignore the field, so when it
  is requested the result stays `RESULT_OK` and `error_message` records that it was ignored.

## What counts as an entity

Entities are **only the things created through `spawn_entity` / `spawn_entities`**. Floors,
obstacles and lights placed through the UI are world scenery and do not appear in
`get_entities`.

Otherwise `delete_entity` and `set_entity_state` would straddle two populations — objects
created from ROS and objects placed in the GUI — which does not fit `load_world` replacing
the scenery wholesale.

Default categories are `CATEGORY_ROBOT` for anything built from a URDF and
`CATEGORY_OBJECT` for a bare mesh. `set_entity_info` overrides them at any time.

### Frames

Poses, twists and bounds are all exchanged in ROS convention (right-handed, Z up, metres).
The mapping to Unity's left-handed, Y-up convention is the same one used when publishing
`/ground_truth`.

`acceleration` is a first difference of the root body's velocity, taken every `FixedUpdate`
because Unity exposes no acceleration to read. It is not smoothed, so contacts make it spike.
While `timeScale` is 0 no `FixedUpdate` runs, so a paused simulation keeps the last value.

For a URDF robot, `get_entity_state` reports the pose of the **base link**. The root
GameObject does not follow the ArticulationBody solver, so reading it would make the robot
look stuck at its spawn pose.

### Bounds filtering

`EntityFilters.bounds` supports `TYPE_BOX`, `TYPE_SPHERE` and `TYPE_CONVEX_HULL`
(`TYPE_EMPTY` means no filter). Entities are treated as their AABB, and those overlapping the
bounds are returned.

Convex hulls are tested with **GJK**. The separating axis test would need the hull's face
normals, but `Bounds.msg` carries only vertices — the faces are not in the message. GJK sees a
shape purely as "given a direction, return the furthest point", so it decides the question
without reconstructing any faces. Hulls whose points are all coplanar (a triangle or a quad) or
even collinear work as-is, as degenerate convex sets.

`TYPE_CONVEX_HULL` needs **at least three points**, as `Bounds.msg` specifies; fewer returns
`RESULT_OPERATION_FAILED`.

### Name filter

`EntityFilters.filter` is matched against the **whole** name, following the
simulation_interfaces reference implementation's use of `std::regex_match`. Use `.*foo.*`
for a substring match. The spec calls for POSIX extended regular expressions; evaluation
happens through .NET's engine, which accepts ordinary expressions unchanged.

## Worlds are scene JSON

A "world" here is the scene JSON that the GUI's Save Scene writes (the `SavedSceneData`
format). Switching Unity scenes was the alternative, but that would require rebuilding the
player for every new world. JSON stays entirely at runtime, so scenery assembled in the GUI
can be handed straight to `load_world`.

The **built-in stage — ground plane, default lighting — is always present**; a world is
scenery layered on top of it. At startup the built-in scene counts as the loaded world, so
the long-standing behaviour of starting in `STATE_STOPPED` is unchanged.

```bash
# Swap the scenery (all entities are removed, state returns to stopped, time is reset)
ros2 service call /load_world simulation_interfaces/srv/LoadWorld \
  "{ world_resource: { uri: 'file:///home/user/worlds/warehouse.json', resource_string: '' },
     fail_on_unsupported_element: false, ignore_missing_or_unsupported_assets: false }"

# Inspect the current world
ros2 service call /get_current_world simulation_interfaces/srv/GetCurrentWorld "{}"

# Unload it
ros2 service call /unload_world simulation_interfaces/srv/UnloadWorld "{}"
```

After `unload_world` the state is `STATE_NO_WORLD`. In that state:

- `set_simulation_state` rejects everything except `STATE_QUITTING` with
  `INCORRECT_TRANSITION`
- `spawn_entity` / `spawn_entities` / `reset_simulation` / `step_simulation` return
  `RESULT_INCORRECT_STATE`

`load_world` brings it back. **The built-in scene can be restored too**, even though it
cannot be exported as a file: its `uri` is empty, but `world_resource.resource_string` holds
a scene JSON describing "no scenery loaded". Hand the `Resource` from `get_current_world`
straight back to `load_world` and you are where you started. A world loaded from a string
keeps its `resource_string` for the same reason.

### World name, description and tags

These go at the top of the scene JSON. All three are optional; without them the name falls
back to the file name, the description to the object count, and the tags to empty. Keeping
the metadata inside the world file means shipping the file ships its tags too — there is no
separate index to keep in sync.

```json
{
  "name": "warehouse",
  "description": "Indoor environment with shelving and pallets",
  "tags": ["indoor", "warehouse"],
  "objects": [ ... ]
}
```

Whatever is written here shows up both in the `get_available_worlds` listing and in
`get_current_world`. The GUI's Save Scene carries the loaded world's metadata back out, so
opening a world and saving it again does not drop its tags.

`get_available_worlds` can filter on those tags (`WORLD_TAGS`):

```bash
# Worlds carrying either indoor or outdoor (FILTER_MODE_ANY)
ros2 service call /get_available_worlds simulation_interfaces/srv/GetAvailableWorlds \
  "{ filter: { tags: ['indoor', 'outdoor'], filter_mode: 0 } }"

# Worlds carrying both indoor and warehouse (FILTER_MODE_ALL)
ros2 service call /get_available_worlds simulation_interfaces/srv/GetAvailableWorlds \
  "{ filter: { tags: ['indoor', 'warehouse'], filter_mode: 1 } }"
```

- Asking for tags **excludes worlds that carry none**.
- No match returns an empty list with `RESULT_OK`. Finding nothing is not an error.
- A `filter_mode` other than `0` or `1` returns `RESULT_OPERATION_FAILED`. Silently treating
  it as `ANY` would return a differently-filtered result and call it success. For the same
  reason the tag filters on `get_entities`, `get_entities_states` and `get_named_poses`
  reject an unknown `filter_mode` too.

### Result codes when loading

`load_world` validates the JSON before clearing anything, so a file that cannot be read
never leaves you with the scenery deleted and nothing in its place.

| Situation | Result code |
|---|---|
| `uri` is not a file URI, or does not end in `.json` | `UNSUPPORTED_FORMAT` (101) |
| Both `uri` and `resource_string` are empty | `NO_RESOURCE` (102) |
| Not readable as JSON | `RESOURCE_PARSE_ERROR` (103) |
| File missing, or a mesh could not be loaded | `MISSING_ASSETS` (104) |
| An element has an unknown `type` | `UNSUPPORTED_ELEMENTS` (106) |

`ignore_missing_or_unsupported_assets` and `fail_on_unsupported_element` suppress
`MISSING_ASSETS` and `UNSUPPORTED_ELEMENTS` respectively. Skipped elements are listed in
`error_message` either way.

## step_simulation

Only while paused: advance the requested number of steps and stop again.

```bash
ros2 service call /set_simulation_state simulation_interfaces/srv/SetSimulationState "{ state: { state: 2 } }"
ros2 service call /step_simulation simulation_interfaces/srv/StepSimulation "{ steps: 10 }"
```

- Returns `RESULT_OPERATION_FAILED` when the simulation is not paused, as specified.
- Does not respond until stepping finishes. A single call may advance at most 100000 steps;
  beyond that it returns `RESULT_OPERATION_FAILED`. The cap keeps a mistyped request from
  taking the simulator away for hours.
- A `set_simulation_state`, a `reset_simulation` or the on-screen button arriving mid-run
  aborts the stepping and returns `RESULT_OPERATION_FAILED`.

Stepping restores `Time.timeScale` rather than calling `Physics.Simulate()`, because the
latter does not run `FixedUpdate`: physics would advance while `ServoJointModel`,
`JointStateSub` and the rest of the control path stood still.

## Spawning from resource_string

Leave `entity_resource.uri` empty and put the URDF itself in `resource_string`
(`SPAWNING_RESOURCE_STRING`) — the point being to hand over an expanded xacro without
writing it to disk first.

The description is written to a temporary file and then joins the normal path. Both the URDF
Importer and the XML parsing here take a path, so funnelling everything through one entry
point avoids a second, string-only code path. The temporary file is removed once the spawn
finishes.

### Where meshes are looked up

Passing a string removes the "next to the URDF" anchor, so mesh references need somewhere
else to resolve from. **`spawnable_paths` in `simulation_resources.json` doubles as the
search path** — the equivalent of Gazebo's `GZ_SIM_RESOURCE_PATH`. **`AMENT_PREFIX_PATH`** is
consulted too, so `package://` resolves with no extra configuration when the simulator is
launched from a sourced ROS 2 environment.

For each root, `package://<pkg>/<rest>` is tried in this order:

| Candidate | What the root points at |
|---|---|
| `<root>/<pkg>/<rest>` | A directory holding packages side by side |
| `<root>/share/<pkg>/<rest>` | An ament install prefix |
| `<root>/<pkg>/share/<pkg>/<rest>` | A colcon install with isolated prefixes |
| `<root>/<rest>` | That package's own directory |

The search runs **only when the asset is not found next to the URDF**, so URDFs that already
loaded behave exactly as before. Absolute `file://` references — the form this repository's
descriptions use — never depended on the anchor and resolve without any search path.

The search paths live in the URDF Importer, so `package://` now also resolves for ordinary
`uri` spawns.

## The simulate_steps action

The same thing `step_simulation` does, with **per-step progress** and **cancellation**. It
is the better fit when advancing a large number of steps.

```bash
ros2 service call /set_simulation_state simulation_interfaces/srv/SetSimulationState "{ state: { state: 2 } }"
ros2 action send_goal -f /simulate_steps simulation_interfaces/action/SimulateSteps "{ steps: 100 }"
```

- Goals are **always accepted**. Failures such as "not paused" come back as
  `OPERATION_FAILED` in the result with the goal status set to `ABORTED`. Rejecting a goal
  would need another round trip before execution starts, and `SimulateSteps.action` itself
  specifies reporting that case through the result.
- One feedback message per physics step, carrying `completed_steps` and `remaining_steps`.
- Cancelling stops at that point and the status becomes `CANCELED`. The steps already taken
  are valid, so the result's own `result` stays `RESULT_OK`.
- The step limit and the cancellation path are shared with `step_simulation`. The service and
  the action never run at the same time; whichever arrives second gets `OPERATION_FAILED`.

### Endpoint requirement

Actions required changes to **both** ROS-TCP-Connector and ROS-TCP-Endpoint, neither of which
had any action support upstream: `ROSConnection.ImplementAction<TGoal, TResult>` on the
connector, and the `__unity_action` / `__action_goal` / `__action_cancel` /
`__action_feedback` / `__action_result` system commands on the endpoint. **Both hijimasa
forks must be at a version that carries them.**

## simulation_resources.json

`get_spawnables`, `get_named_poses`, `get_named_pose_bounds` and `get_available_worlds` all
draw their contents from this config file. A Unity player has nothing equivalent to ament's
package search path, so what the simulator "can see" has to be stated explicitly.

The search order is below; if none exist, the services return empty lists with `RESULT_OK`
(starting without any configuration is the priority, so a missing file is not an error).

1. The path in the `SIMULATION_RESOURCES_CONFIG` environment variable
2. `simulation_resources.json` next to the player executable (the project root in the Editor)
3. `Application.persistentDataPath/simulation_resources.json`

```json
{
  "spawnable_paths": [
    "/root/colcon_ws/install/diffbot_description/share",
    "/root/models"
  ],
  "world_paths": [
    "/root/worlds"
  ],
  "named_poses": [
    {
      "name": "charger",
      "description": "In front of the charging station",
      "tags": ["spawn", "parking"],
      "position": [1.0, 2.0, 0.0],
      "rpy": [0.0, 0.0, 1.5708],
      "bounds": {
        "type": "box",
        "min": [-0.3, -0.3, 0.0],
        "max": [0.3, 0.3, 0.5]
      }
    },
    {
      "name": "gate",
      "position": [-4.0, 0.0, 0.0],
      "orientation": [0.0, 0.0, 0.0, 1.0],
      "bounds": { "type": "sphere", "center": [0.0, 0.0, 0.5], "radius": 1.0 }
    }
  ]
}
```

- All coordinates are ROS convention (right-handed, Z up, metres, radians).
- Give an orientation either as `orientation` (quaternion `[x, y, z, w]`) or as `rpy`
  (`[roll, pitch, yaw]`). If both are present, `orientation` wins.
- `bounds` is optional (it becomes `TYPE_EMPTY`). `type` may only be `box` or `sphere`.
- `spawnable_paths` is walked recursively, collecting `.urdf`, `.obj`, `.stl`, `.dae`,
  `.fbx` and `.ply`.
- From `world_paths`, only `.json` files that **parse as a scene** become candidates, since
  unrelated JSON such as config files will be sitting in the same directories.
- Scanning stops after 2000 files in total. When it does, the truncation is reported in
  `error_message` rather than silently dropped.

`sources` on `get_spawnables` and `additional_sources` on `get_available_worlds` add
one-off search locations. Passing something that does not exist is not a failure; the
`error_message` names what could not be read (`get_available_worlds` does return
`DEFAULT_SOURCES_FAILED` unless `continue_on_error` is set).

## Examples

```bash
# List the current entities
ros2 service call /get_entities simulation_interfaces/srv/GetEntities "{ filters: { filter: '' } }"

# Robots only, with their states
ros2 service call /get_entities_states simulation_interfaces/srv/GetEntitiesStates \
  "{ filters: { categories: [{ category: 1 }] } }"

# Only what overlaps a 3 m sphere
ros2 service call /get_entities simulation_interfaces/srv/GetEntities \
  "{ filters: { bounds: { type: 3, points: [{x: 0.0, y: 0.0, z: 0.0}, {x: 3.0, y: 0.0, z: 0.0}] } } }"

# Move an entity without touching its twist or acceleration
ros2 service call /set_entity_state simulation_interfaces/srv/SetEntityState \
  "{ entity: 'diffbot', set_pose: true, set_twist: false, set_acceleration: false,
     state: { pose: { position: {x: 1.0, y: 0.0, z: 0.0},
                      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0} } } }"

# Tag it
ros2 service call /set_entity_info simulation_interfaces/srv/SetEntityInfo \
  "{ entity: 'diffbot', info: { category: { category: 1 }, description: 'Differential drive base', tags: ['agv'] } }"

# Remove just this one
ros2 service call /delete_entity simulation_interfaces/srv/DeleteEntity "{ entity: 'diffbot' }"
```
