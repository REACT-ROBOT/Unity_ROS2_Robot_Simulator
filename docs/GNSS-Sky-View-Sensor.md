# GNSS Sky View Sensor

`<sensor type="gnss_sky_view">` casts one ray per satellite from a GNSS antenna and
publishes **which satellites reach it directly** and **how good the geometry of that
set is** (DOP).

It produces no position, no fix grade and no error. Those depend on receiver
behaviour that is far easier to tune and unit-test outside the simulator. What
cannot be known outside the simulator is what the buildings block, and that is all
this sensor does.

In the companion workspace, `hardware_emulator/gps_emulator` subscribes to this
topic, decides RTK fix / float / single, generates the matching position error and
emits it as NMEA.

## URDF

```xml
<sensor name="gnss_antenna_link" type="gnss_sky_view">
  <update_rate>5.0</update_rate>
  <satellite_count>24</satellite_count>
  <elevation_mask>15.0</elevation_mask>
  <max_range>500.0</max_range>
  <seed>1</seed>
  <hit_triggers>false</hit_triggers>
</sensor>
```

| Element | Default | Meaning |
|---|---|---|
| `satellite_count` | 24 | Satellites placed above the elevation mask |
| `elevation_mask` | 15.0 | Elevation mask [deg]; lower satellites are not tracked |
| `max_range` | 500.0 | Ray length [m]; make it cover the scene |
| `seed` | 1 | Constellation seed; the same seed is the same sky |
| `hit_triggers` | false | Whether trigger colliders shadow a satellite |

`name` is the link the sensor rides. **The antenna's height and placement decide the
result** — on top of a mast the robot body casts no shadow.

The topic is `/<robot>/<link>/sky_view`, of type
`simulation_extra_interfaces/GnssSkyView`.

## Things to know

- **Trigger colliders are ignored by default.** The simulator uses triggers for
  things sensors should see but that should not block movement (weeds, magnetic
  tape, see [URDF-Collision-Material.md](URDF-Collision-Material.md)), and none of
  those should shadow a satellite. Buildings are solid colliders and block normally.
- **The constellation is synthetic, not an almanac.** It is a deterministic
  golden-angle lattice, uniform in solid angle above the mask. A real mid-latitude
  GNSS sky is denser at middle elevations and has a hole towards the pole, because
  the orbits are inclined. **Do not read absolute DOP values as if they were a site
  survey**; they are good for comparisons (open sky against a wall).
- **Satellites are held still for the run.** Over a SILS run of minutes the real
  geometry moves well under a degree, so the error is negligible and a frozen sky
  keeps runs repeatable.
- Axis convention: this simulator publishes its world as ROS ENU with
  `x = Unity z` and `y = -Unity x`, so **East is Unity +z and North is Unity -x**.
  That is not the mapping `GeoCoordinateConverter` uses internally.

## Cost

One ray per satellite per update (24 by default), which is nothing next to the
LiDARs' tens of thousands per frame. Reflected (NLOS) paths will need thousands of
rays; that stage will want the `RaycastCommand` batch API.
