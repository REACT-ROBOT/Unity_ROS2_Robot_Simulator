# GNSS Receiver Sensor

`<sensor type="gnss">` simulates a GNSS receiver. With a
[`gnss_sky_view`](GNSS-Sky-View-Sensor.md) sensor on the same link, the sky it
traces drives the solution grade (RTK fix / float / single-point) and the position
error. Without one, the reported position is the exact truth.

**The receiver model runs in the simulator.** A ROS user gets a realistic fix by
spawning a robot, with nothing extra to launch.

## Topics

| Topic | Type | Contents |
|---|---|---|
| `/<robot>/<link>/fix` | `sensor_msgs/NavSatFix` | The degraded fix; status and position_covariance follow the grade |
| `/<robot>/<link>/solution` | `simulation_extra_interfaces/GnssSolution` | What NavSatFix cannot express |

**`NavSatFix.status` cannot separate an RTK fix from an RTK float.** It has four
values (`NO_FIX`, `FIX`, `SBAS_FIX`, `GBAS_FIX`) and both RTK grades are
ground-based augmentation. That is a limitation of the message, not of this
sensor, and every real driver has it. The quality therefore travels in
`position_covariance` -- which is what consumers such as `robot_localization`'s
`navsat_transform_node` actually read. It is built from the grade's own sigma and
published as `COVARIANCE_TYPE_DIAGONAL_KNOWN`.

Consumers that need the distinction -- an NMEA bridge emitting GGA quality 4
against 5, a localiser weighting by grade -- read `GnssSolution`. It also carries
the error broken down (stochastic bias, reflection-driven, wrong fix), so an
evaluation can use the exact error instead of inferring it.

## URDF

```xml
<sensor name="gnss_antenna_link" type="gnss">
  <update_rate>5.0</update_rate>
  <origin_latitude>36.549774</origin_latitude>
  <origin_longitude>139.928772</origin_longitude>
  <origin_altitude>100.0</origin_altitude>
  <reconvergence_sec>8.0</reconvergence_sec>
  <lock_seconds_for_fix>8.0</lock_seconds_for_fix>
  <wrong_fix_probability>0.01</wrong_fix_probability>
  <stochastic_error>true</stochastic_error>
  <seed>20260914</seed>
</sensor>
```

| Element | Default | Meaning |
|---|---|---|
| `origin_*` | Tokyo | Geodetic origin. **Must match the base given to gpsd.** |
| `reconvergence_sec` | 8.0 | [s] to re-fix when per-satellite lock is unavailable |
| `lock_seconds_for_fix` | 8.0 | [s] of unbroken carrier before a satellite can help fix |
| `wrong_fix_probability` | 0.01 | Chance each re-fix resolves the WRONG integers |
| `stochastic_error` | true | false leaves only the error the geometry produces |
| `seed` | 20260914 | Random seed |

The sky source is resolved lazily from the same link, so the order of the two
sensor entries in the URDF does not matter.

## What the model does

| Piece | Behaviour |
|---|---|
| Grade ladder | Usable satellites and HDOP set the ceiling; ambiguity resolution decides what is actually reached |
| Stochastic error | First-order Gauss-Markov: **a bias correlated over tens of seconds, not white noise**, so averaging does not remove it. Continuous across a grade change |
| Reflections (NLOS) | Excess path length enters the least-squares solution as a pseudorange bias, so **the error points somewhere the map explains** and repeats at the same place |
| Carrier lock | Counted per satellite. Brushing a pole and driving under a bridge are not the same event |
| Wrong fix | Wrong integers: **reported as a healthy RTK fix while sitting a decimetre or more out.** Nothing downstream can tell |
| Cap while fixed | Carrier multipath cannot exceed a quarter wavelength (4.8 cm on L1). Geometry sets the direction, physics the bound |

## Things to know

- **Axis convention.** This simulator publishes its world as ROS ENU (east is
  Unity +z, north is Unity -x), which is ninety degrees from the convention
  `GeoCoordinateConverter` uses internally (east is Unity x). This sensor states
  its ENU offset explicitly and so agrees with the ground truth; calling
  `GeoCoordinateSystem.GetCoordinate()` directly gives a fix **rotated by ninety
  degrees, wrong by a metre for every metre driven.**
- The reflection loss is a constant, independent of incidence angle and material.
  Tune it against measured C/N0.
- The constellation is synthetic, not an almanac. **Do not read absolute DOP
  values as if they were a site survey.**
