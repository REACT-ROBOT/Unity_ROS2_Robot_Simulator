# GNSS Receiver Sensor

`<sensor type="gnss">` simulates a GNSS receiver. With a
[`gnss_sky_view`](GNSS-Sky-View-Sensor.md) sensor on the same link, the sky it
traces drives the solution grade (RTK fix / float / single-point) and the position
error. Without one, the reported position is the exact truth.

**The receiver model runs in the simulator.** A ROS user gets a realistic fix by
spawning a robot, with nothing extra to launch.

## Topics

**No simulator-specific messages.** Everything is a type an existing GNSS package
already defines.

| Topic | Type | Contents |
|---|---|---|
| `/<robot>/<link>/fix` | `sensor_msgs/NavSatFix` | The degraded fix; status and position_covariance follow the grade |
| `/<robot>/<link>/extended_fix` | `gps_msgs/GPSFix` | **RTK fixed against float**, per-satellite azimuth/elevation/SNR, every DOP |
| `/<robot>/<link>/nmea` | `nmea_msgs/Sentence` | GGA / RMC, so the NMEA driver you run on hardware works here unchanged |

`NavSatFix.status` has four values (`NO_FIX`, `FIX`, `SBAS_FIX`, `GBAS_FIX`) and
**cannot separate an RTK fix from an RTK float** -- both are ground-based
augmentation. That is a limitation of the message, not of this sensor, and every
real driver has it. On NavSatFix the quality therefore travels in
`position_covariance`, which is what `robot_localization`'s `navsat_transform_node`
reads; it comes from the grade's own sigma, published as
`COVARIANCE_TYPE_DIAGONAL_KNOWN`.

**For the distinction, read `gps_msgs/GPSFix`**: its `GPSStatus` has
`STATUS_RTK_FIX` (19) and `STATUS_RTK_FLOAT` (20).

Satellites are reported the way a real receiver reports them:

| Path | In GPSStatus |
|---|---|
| Direct | in `satellite_used_prn` and `satellite_visible_*` (SNR at the clean 45 dB-Hz) |
| Reflected (NLOS) | in `satellite_visible_*` only, not used -- and **weaker, by the reflection loss** |
| Blocked | in neither |

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

## Visualising the received paths

The entity panel's `<link> gnss rays` toggle draws **how each satellite's signal
actually reached the antenna**. Starting with `SIM_AUTO_SENSOR_VIZ=1` turns it on
automatically for the first entity.

| Colour | Meaning |
|---|---|
| Green (thin) | Direct line of sight; a usable signal |
| **Amber (thick)** | **A reflected path (NLOS): antenna -> wall -> on towards the sky** |
| Red (short, faint) | Blocked, nothing gets through |

Only the reflected paths are drawn heavy. Among twenty clear signals it is the two
or three that bounced which move the solution, and they are the only thing in the
picture worth looking at. The kink itself is the detour the receiver measures as
range.

**A deep canyon cannot be seen into from the side.** Looking into a 10 m slot
between 40 m walls means looking almost straight down. For a recording, use
shallower walls (16 m apart, 14 m tall works well) and film along the street axis.

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
