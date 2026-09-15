using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nmea;
using RosMessageTypes.Std;

using UnitySensors.DataType.Sensor;
using UnitySensors.Sensor.GNSS;

/// <summary>
/// 受信機の解を nmea_msgs/Sentence として publish する。
/// </summary>
/// <remarks>
/// これがあると、ROS ユーザは**実機で使っているのと同じ NMEA ドライバ**を
/// 仮想シリアルなしでシミュレータに向けられる。RTK Fix と Float の区別も
/// GGA の quality 4/5 としてバイト列の中で生き残る。
///
/// ロボット側の HILS 経路 (gpsd) 向けには、hardware_emulator の gps_emulator が
/// この文をそのまま pty へ流すだけになる。
///
/// 1 エポックにつき GGA と RMC の 2 文を出すので、1 センサ 1 メッセージ前提の
/// RosMsgPublisher は使えない。
/// </remarks>
public class GnssNmeaPublisher : MonoBehaviour
{
    [SerializeField, Min(0)]
    private float _frequency = 5.0f;
    [SerializeField]
    private string _topicName = "/gnss/nmea";
    [SerializeField]
    private GNSSSensor _source;
    [SerializeField]
    private string _frameId = "gnss_link";

    private ROSConnection _ros;
    private float _dt;
    // One message object per sentence type, not one shared. The connector does not
    // necessarily serialise before Publish returns, so publishing twice from the
    // same object in a frame lets the second write clobber the first -- which
    // showed up as almost every GGA arriving as an RMC.
    private SentenceMsg _ggaMsg;
    private SentenceMsg _rmcMsg;

    public string topicName { get => _topicName; set => _topicName = value; }
    public float frequency
    {
        get => _frequency;
        set => _frequency = Mathf.Max(value, 0);
    }

    public void Configure(GNSSSensor source, string frameId)
    {
        _source = source;
        _frameId = frameId;
    }

    private void Start()
    {
        if (_source == null)
        {
            _source = GetComponent<GNSSSensor>();
        }
        _ggaMsg = new SentenceMsg { header = new HeaderMsg { frame_id = _frameId } };
        _rmcMsg = new SentenceMsg { header = new HeaderMsg { frame_id = _frameId } };
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterPublisher<SentenceMsg>(_topicName);
    }

    private void Update()
    {
        if (_source == null || _ros == null)
        {
            return;
        }
        _dt += Time.deltaTime;
        float period = (_frequency > 0.0f) ? 1.0f / _frequency : 0.2f;
        if (_dt < period)
        {
            return;
        }
        _dt -= period;

        GnssSolution solution = _source.solution;
        NmeaProtocol.FixQuality quality = ToQuality(solution.state);
        if (quality == NmeaProtocol.FixQuality.NoFix)
        {
            // 測位できていない受信機は有効な文を出さない。quality 0 の GGA を
            // 出すと、quality を見ずに lat/lon を使う parser を動かしてしまう。
            return;
        }

        DateTime utc = DateTime.UtcNow;
        double latitude = _source.coordinate.latitude;
        double longitude = _source.coordinate.longitude;

        _ggaMsg.sentence = NmeaProtocol.Gga(latitude, longitude, _source.coordinate.altitude, quality,
            utc, solution.usableSatellites, solution.hdop);
        _ros.Publish(_topicName, _ggaMsg);

        _rmcMsg.sentence = NmeaProtocol.Rmc(latitude, longitude, quality, utc,
            _source.groundSpeed, _source.courseDegrees);
        _ros.Publish(_topicName, _rmcMsg);
    }

    private static NmeaProtocol.FixQuality ToQuality(RtkState state)
    {
        switch (state)
        {
            case RtkState.Fix: return NmeaProtocol.FixQuality.RtkFix;
            case RtkState.Float: return NmeaProtocol.FixQuality.RtkFloat;
            case RtkState.Single: return NmeaProtocol.FixQuality.Standard;
            default: return NmeaProtocol.FixQuality.NoFix;
        }
    }
}
