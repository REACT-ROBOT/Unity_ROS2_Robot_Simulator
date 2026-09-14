using UnityEngine;
using RosMessageTypes.SimulationExtraInterfaces;

using UnitySensors.Attribute;
using UnitySensors.DataType.Sensor;
using UnitySensors.Interface.Sensor;
using UnitySensors.ROS.Serializer;
using UnitySensors.ROS.Serializer.Std;

/// <summary>
/// <see cref="IGnssSkyViewInterface"/> の視界を simulation_extra_interfaces/GnssSkyView
/// に詰める。メッセージ型がこのシミュレータ固有なので UnitySensorsROS ではなくここに置く。
/// </summary>
[System.Serializable]
public class GnssSkyViewMsgSerializer : RosMsgSerializer<GnssSkyViewMsg>
{
    [SerializeField, Interface(typeof(IGnssSkyViewInterface))]
    private Object _source;
    [SerializeField]
    private HeaderSerializer _header;

    private IGnssSkyViewInterface _sourceInterface;

    public void Configure(IGnssSkyViewInterface source, HeaderSerializer header)
    {
        _source = source as Object;
        _sourceInterface = source;
        _header = header;
    }

    public override void Init()
    {
        base.Init();
        _header.Init();
        _sourceInterface = _source as IGnssSkyViewInterface;
        _msg.satellites = System.Array.Empty<GnssSatelliteMsg>();
    }

    public override GnssSkyViewMsg Serialize()
    {
        GnssSkyView view = _sourceInterface.skyView;
        SatelliteObservation[] satellites = view.satellites ?? System.Array.Empty<SatelliteObservation>();

        // 衛星数は走行中変わらないので、配列と要素は使い回して毎周期の GC を避ける。
        if (_msg.satellites == null || _msg.satellites.Length != satellites.Length)
        {
            _msg.satellites = new GnssSatelliteMsg[satellites.Length];
            for (int i = 0; i < satellites.Length; i++)
            {
                _msg.satellites[i] = new GnssSatelliteMsg();
            }
        }

        for (int i = 0; i < satellites.Length; i++)
        {
            GnssSatelliteMsg satellite = _msg.satellites[i];
            satellite.prn = satellites[i].prn;
            satellite.azimuth = satellites[i].azimuth;
            satellite.elevation = satellites[i].elevation;
            satellite.state = ToStateConstant(satellites[i].visibility);
            satellite.excess_path_length = satellites[i].excessPathLength;
            satellite.relative_power_db = satellites[i].relativePowerDb;
        }

        _msg.header = _header.Serialize();
        // 255 を超える衛星を同時に追うことはないので飽和で十分。
        _msg.usable_satellites = (byte)Mathf.Clamp(view.usableSatellites, 0, 255);
        _msg.hdop = view.hdop;
        _msg.pdop = view.pdop;
        return _msg;
    }

    private static byte ToStateConstant(SatelliteVisibility visibility)
    {
        switch (visibility)
        {
            case SatelliteVisibility.LineOfSight: return GnssSatelliteMsg.STATE_LOS;
            case SatelliteVisibility.Blocked: return GnssSatelliteMsg.STATE_BLOCKED;
            case SatelliteVisibility.Nlos: return GnssSatelliteMsg.STATE_NLOS;
            default: return GnssSatelliteMsg.STATE_BELOW_MASK;
        }
    }
}
