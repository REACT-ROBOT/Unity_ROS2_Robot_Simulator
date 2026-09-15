using UnityEngine;
using RosMessageTypes.Gps;

using UnitySensors.Attribute;
using UnitySensors.DataType.Sensor;
using UnitySensors.Sensor.GNSS;
using UnitySensors.ROS.Serializer;
using UnitySensors.ROS.Serializer.Std;

/// <summary>
/// 受信機の解を gps_msgs/GPSFix に詰める。
/// </summary>
/// <remarks>
/// NavSatFix が表現できない RTK Fix / Float の区別は、この型の GPSStatus が
/// STATUS_RTK_FIX / STATUS_RTK_FLOAT として持っている。衛星ごとの方位・仰角・
/// SNR と各種 DOP も入るので、シミュレータ固有のメッセージは要らない。
///
/// 衛星の対応付け:
///   直達  -> used かつ visible (SNR は基準値)
///   反射  -> visible だが used ではない (SNR は反射損失ぶん低い)
///   遮蔽  -> どちらにも入らない
/// これは実機の受信機が報告する内容とそのまま同じ意味になる。
/// </remarks>
[System.Serializable]
public class GpsFixMsgSerializer : RosMsgSerializer<GPSFixMsg>
{
    /// <summary>[dB-Hz] C/N0 reported for a clean direct signal.</summary>
    const int CleanSnr = 45;

    [SerializeField, Interface(typeof(UnitySensors.Interface.Sensor.IGnssSolutionInterface))]
    private Object _source;
    [SerializeField]
    private HeaderSerializer _header;

    private GNSSSensor _sensor;

    public void Configure(GNSSSensor sensor, HeaderSerializer header)
    {
        _source = sensor;
        _sensor = sensor;
        _header = header;
    }

    public override void Init()
    {
        base.Init();
        _header.Init();
        _sensor = _source as GNSSSensor;
        _msg.status = new GPSStatusMsg();
        _msg.position_covariance = new double[9];
    }

    public override GPSFixMsg Serialize()
    {
        GnssSolution solution = _sensor.solution;
        _msg.header = _header.Serialize();
        _msg.status.header = _msg.header;
        _msg.status.status = ToStatus(solution.state);
        _msg.status.position_source = GPSStatusMsg.SOURCE_GPS;
        _msg.status.motion_source = GPSStatusMsg.SOURCE_POINTS;
        _msg.status.orientation_source = GPSStatusMsg.SOURCE_NONE;

        FillSatellites(_sensor.sky);

        _msg.latitude = _sensor.coordinate.latitude;
        _msg.longitude = _sensor.coordinate.longitude;
        _msg.altitude = _sensor.coordinate.altitude;
        _msg.track = _sensor.courseDegrees;
        _msg.speed = _sensor.groundSpeed;

        _msg.hdop = solution.hdop;
        _msg.pdop = solution.pdop;
        // The model does not separate the vertical and time components, so saying
        // "unknown" (<= 0, as the message defines it) is the honest answer rather
        // than inventing numbers a consumer might weight by.
        _msg.vdop = -1.0;
        _msg.tdop = -1.0;
        _msg.gdop = -1.0;

        // err_* are 95% confidence per the message; the model works in 1 sigma.
        double sigma = solution.horizontalSigma;
        _msg.err_horz = 1.96 * sigma;
        _msg.err_vert = 1.96 * sigma * 2.0;
        _msg.err = 1.96 * sigma * System.Math.Sqrt(5.0);
        _msg.err_track = -1.0;
        _msg.err_speed = -1.0;
        _msg.err_climb = -1.0;
        _msg.err_time = -1.0;
        _msg.err_pitch = -1.0;
        _msg.err_roll = -1.0;
        _msg.err_dip = -1.0;

        double variance = sigma * sigma;
        for (int i = 0; i < 9; i++) _msg.position_covariance[i] = 0.0;
        _msg.position_covariance[0] = variance;
        _msg.position_covariance[4] = variance;
        _msg.position_covariance[8] = variance * 4.0;
        _msg.position_covariance_type = GPSFixMsg.COVARIANCE_TYPE_DIAGONAL_KNOWN;
        return _msg;
    }

    private void FillSatellites(GnssSkyView sky)
    {
        SatelliteObservation[] satellites = sky.satellites;
        if (satellites == null)
        {
            _msg.status.satellites_used = 0;
            _msg.status.satellites_visible = 0;
            _msg.status.satellite_used_prn = System.Array.Empty<int>();
            _msg.status.satellite_visible_prn = System.Array.Empty<int>();
            _msg.status.satellite_visible_z = System.Array.Empty<int>();
            _msg.status.satellite_visible_azimuth = System.Array.Empty<int>();
            _msg.status.satellite_visible_snr = System.Array.Empty<int>();
            return;
        }

        int used = 0, visible = 0;
        for (int i = 0; i < satellites.Length; i++)
        {
            if (satellites[i].visibility == SatelliteVisibility.LineOfSight) used++;
            if (satellites[i].visibility == SatelliteVisibility.LineOfSight ||
                satellites[i].visibility == SatelliteVisibility.Nlos) visible++;
        }

        if (_msg.status.satellite_used_prn == null || _msg.status.satellite_used_prn.Length != used)
        {
            _msg.status.satellite_used_prn = new int[used];
        }
        if (_msg.status.satellite_visible_prn == null || _msg.status.satellite_visible_prn.Length != visible)
        {
            _msg.status.satellite_visible_prn = new int[visible];
            _msg.status.satellite_visible_z = new int[visible];
            _msg.status.satellite_visible_azimuth = new int[visible];
            _msg.status.satellite_visible_snr = new int[visible];
        }

        int u = 0, v = 0;
        for (int i = 0; i < satellites.Length; i++)
        {
            SatelliteObservation satellite = satellites[i];
            bool isLos = satellite.visibility == SatelliteVisibility.LineOfSight;
            if (!isLos && satellite.visibility != SatelliteVisibility.Nlos)
            {
                continue;
            }
            if (isLos)
            {
                _msg.status.satellite_used_prn[u++] = satellite.prn;
            }
            _msg.status.satellite_visible_prn[v] = satellite.prn;
            _msg.status.satellite_visible_z[v] = Mathf.RoundToInt(satellite.elevation * Mathf.Rad2Deg);
            _msg.status.satellite_visible_azimuth[v] =
                Mathf.RoundToInt(Mathf.Repeat(satellite.azimuth * Mathf.Rad2Deg, 360.0f));
            // A reflected signal really does arrive weaker; reporting the loss in
            // the SNR field is how a real receiver would show the same thing.
            _msg.status.satellite_visible_snr[v] = CleanSnr + Mathf.RoundToInt(satellite.relativePowerDb);
            v++;
        }
        _msg.status.satellites_used = (ushort)used;
        _msg.status.satellites_visible = (ushort)visible;
    }

    private static short ToStatus(RtkState state)
    {
        switch (state)
        {
            case RtkState.Fix: return GPSStatusMsg.STATUS_RTK_FIX;
            case RtkState.Float: return GPSStatusMsg.STATUS_RTK_FLOAT;
            case RtkState.Single: return GPSStatusMsg.STATUS_FIX;
            default: return GPSStatusMsg.STATUS_NO_FIX;
        }
    }
}
