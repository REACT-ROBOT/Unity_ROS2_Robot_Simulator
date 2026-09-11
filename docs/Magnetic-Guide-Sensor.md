# Magnetic guide sensor and magnetic tape

An AGV-style magnetic guide (line) sensor — Roboteq MGS1600, Accurate Systems UDS-1213 —
is a bar of Hall elements mounted a few centimetres above the floor. It reports where a
magnetic tape lies under the bar, whether any tape is there, and "markers": short pieces
of tape of the opposite polarity laid beside the track. This simulator models both the
sensor (`<sensor type="magnetic_guide">`) and the tape (`<magnetic_tape>` on a collision
shape).

## The tape

Tape is part of a URDF entity, so it is spawned, listed, deleted and reset like anything
else. Each piece is a thin `<collision>` box whose `<collision_material>` carries
`<magnetic_tape polarity="track"/>` or `polarity="marker"`:

```xml
<robot name="course">
  <collision_material name="track"><magnetic_tape polarity="track"/></collision_material>
  <collision_material name="marker"><magnetic_tape polarity="marker"/></collision_material>

  <link name="world"/>
  <link name="tape_link">
    <collision>
      <origin xyz="5.0 0 0.0005"/><geometry><box size="10.0 0.025 0.001"/></geometry>
      <collision_material name="track"/>
    </collision>
    <collision>
      <origin xyz="3.15 0.035 0.0005"/><geometry><box size="0.3 0.025 0.001"/></geometry>
      <collision_material name="marker"/>
    </collision>
  </link>
  <joint name="fix" type="fixed"><parent link="world"/><child link="tape_link"/></joint>
</robot>
```

- `magnetic_tape` implies `sensor_only`: the boxes become triggers, so the robot drives
  over them and `get_contact_events` ignores them
  ([URDF-Collision-Material.md](URDF-Collision-Material.md)).
- The lidar is a raycast on layer 0 and *can* hit a trigger. A 1 mm strip on the floor is
  below any 2D scan plane; a 3D lidar sees it as part of the floor, which is what a real
  tape looks like too.
- Keep the whole course in **one link** (every link is an articulation body; PhysX allows
  64 per articulation). Hang it from a `world` link so it stays put.
- Markers sit 15–30 mm outside the track edge (the sensor's datasheet rule), so with 25 mm
  tape the marker centre is at ±35 to ±40 mm from the track centre.
- Polyline courses are easier to write as JSON and convert; the companion workspace's
  `tools/magnetic_line/gen_tape_urdf.py` does exactly this.

## The sensor

```xml
<simulation>
  <sensor name="uds_link" type="magnetic_guide">
    <update_rate>50</update_rate>
    <width>0.16</width>          <!-- bar width [m] -->
    <pitch>0.001</pitch>         <!-- element pitch / reported resolution [m] -->
    <min_height>0.01</min_height><!-- tape closer than this is not read (dragging) -->
    <max_height>0.06</max_height><!-- tape further than this is not read (field too weak) -->
    <fork>nearest</fork>         <!-- nearest | left | right: which track to report at a fork -->
    <noise><stddev>0.0</stddev></noise>  <!-- gaussian noise on the position [m] -->
  </sensor>
</simulation>
```

`name` is the link the bar is attached to, as for every other sensor. The link frame is the
sensor frame: origin at the centre of the bar's face, **+x forward, +y left, -z is the
direction the bar looks in**. Mount it at the real sensor's height (20–40 mm above the
floor is the datasheet's sweet spot).

Each element casts a ray down `max_height` and is "over tape" when the nearest thing it
hits carries `MagneticTape` at a distance of at least `min_height`. Anything solid between
the bar and the tape shields it. Consecutive track elements form a track and its centre is
the reported position; a single missing element does not split a track.

Topic: `/<entity>/<link>/magnetic_guide`, type `simulation_extra_interfaces/MagneticGuide`:

| field | meaning |
|---|---|
| `track_detected` | a track-polarity tape is under the bar |
| `position` | lateral position of the selected track [m], **+left**, quantised to `pitch`; 0 when none |
| `left_marker` / `right_marker` | marker-polarity tape on that side of the selected track (of the bar centre when no track) |
| `track_positions` | every track under the bar, left to right; a fork shows two |

The sensor works headless (it is a physics query, not a render).

## Checking it

- Solver tests (samples → reading): `UnitySensors/Packages/UnitySensors/Tests/Editor/MagneticGuideSolverTests.cs`
- Sensor geometry tests (raycast, sides, height, shielding): `Assets/Tests/MagneticGuideTests`
  (playmode, `-assemblyNames MagneticGuideTests`)
- Scenario tests: `sim_test_utils/examples/test_magnetic_guide.py` in the companion
  workspace spawns a course and a sensor and checks the topic.
