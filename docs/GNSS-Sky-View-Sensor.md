# GNSS Sky View Sensor

`<sensor type="gnss_sky_view">` casts one ray per satellite from a GNSS antenna and
publishes **which satellites reach it directly** and **how good the geometry of that
set is** (DOP).

It produces no position, no fix grade and no error. Those depend on receiver
behaviour that is far easier to tune and unit-test outside the simulator. What
cannot be known outside the simulator is what the buildings block, and that is all
this sensor does.

The `<sensor type="gnss">` on the same link consumes this as the input to its
receiver model. That receiver publishes a degraded `sensor_msgs/NavSatFix` plus a
`simulation_extra_interfaces/GnssSolution` carrying what NavSatFix cannot express
(RTK fix against float, the error breakdown, wrong fixes), so a ROS user gets a
realistic fix by spawning a robot and nothing else. See
[GNSS-Receiver.md](GNSS-Receiver.md).

## URDF

```xml
<sensor name="gnss_antenna_link" type="gnss_sky_view">
  <update_rate>5.0</update_rate>
  <satellite_count>24</satellite_count>
  <elevation_mask>15.0</elevation_mask>
  <max_range>500.0</max_range>
  <seed>1</seed>
  <hit_triggers>false</hit_triggers>
  <reflections>true</reflections>
  <reflection_spacing>2.5</reflection_spacing>
  <reflection_loss_db>-13.0</reflection_loss_db>
</sensor>
```

| Element | Default | Meaning |
|---|---|---|
| `satellite_count` | 24 | Satellites placed above the elevation mask |
| `elevation_mask` | 15.0 | Elevation mask [deg]; lower satellites are not tracked |
| `max_range` | 500.0 | Ray length [m]; make it cover the scene |
| `seed` | 1 | Constellation seed; the same seed is the same sky |
| `hit_triggers` | false | Whether trigger colliders shadow a satellite |
| `reflections` | true | Search for one-bounce (NLOS) paths |
| `reflection_spacing` | 2.5 | Angular spacing of the reflection sweep [deg]; **halving it quadruples the rays** |
| `reflection_loss_db` | -13.0 | Loss along a reflected path [dB] |

`name` is the link the sensor rides. **The antenna's height and placement decide the
result** — on top of a mast the robot body casts no shadow.

**This sensor publishes nothing to ROS.** The `<sensor type="gnss">` on the same
link reads it in process as the input to its receiver model, and the satellite
view reaches ROS inside `gps_msgs/GPSFix`'s `GPSStatus` (used / visible / SNR).

## Reflections (NLOS)

For a satellite whose direct path is blocked, the sensor sweeps the upper
hemisphere and works backwards from each hit: the Householder reflection is its
own inverse, so the launch direction u and the hit normal n give the satellite
that path would have come from as `s = u - 2(u.n)n`, with no surface identified
in advance. A second ray from the reflection point confirms that satellite is
visible from there, and the extra path length is `d(1 - u.s)`.

The excess path scales with the distance to the reflector -- `2*d*cos(elevation)^2`
for a vertical wall -- so a 3 m alley gives a couple of metres while a 10 m
street gives around ten.

The lower hemisphere is not swept. A vertical wall leaves the elevation
unchanged, so a satellite above the horizon can only be reached by a ray above
the horizon; ground bounce is a different mechanism and one the antenna's ground
plane is built to suppress.

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

The direct pass is one ray per satellite (24 by default). The reflection sweep is
`2*pi/spacing^2` rays -- about 3300 at the default 2.5 degrees -- and the confirm
pass only the best four candidates per satellite, so about a hundred.

**The reflection search yields on its job handles rather than blocking.** Blocking
starves the main thread: the base class folds the update time back into the
period, so as soon as one update takes longer than the period the sensor runs
back to back, and the ROS services sharing that thread time out. This was not
theoretical -- it made the simulator unresponsive.
