using UnityEngine;
using RosMessageTypes.SimulationExtraInterfaces;

using UnitySensors.Attribute;
using UnitySensors.DataType.Sensor;
using UnitySensors.Interface.Sensor;
using UnitySensors.ROS.Serializer;
using UnitySensors.ROS.Serializer.Std;

/// <summary>
/// <see cref="IGnssSolutionInterface"/> の解を simulation_extra_interfaces/GnssSolution
/// に詰める。NavSatFix が表現できない RTK Fix/Float の区別と、誤差の内訳を運ぶ。
/// メッセージ型がこのシミュレータ固有なので UnitySensorsROS ではなくここに置く。
/// </summary>
[System.Serializable]
public class GnssSolutionMsgSerializer : RosMsgSerializer<GnssSolutionMsg>
{
    [SerializeField, Interface(typeof(IGnssSolutionInterface))]
    private Object _source;
    [SerializeField]
    private HeaderSerializer _header;

    private IGnssSolutionInterface _sourceInterface;

    public void Configure(IGnssSolutionInterface source, HeaderSerializer header)
    {
        _source = source as Object;
        _sourceInterface = source;
        _header = header;
    }

    public override void Init()
    {
        base.Init();
        _header.Init();
        _sourceInterface = _source as IGnssSolutionInterface;
    }

    public override GnssSolutionMsg Serialize()
    {
        GnssSolution solution = _sourceInterface.solution;
        _msg.header = _header.Serialize();
        _msg.state = ToStateConstant(solution.state);
        // 255 を超える衛星を同時に追うことはないので飽和で十分。
        _msg.usable_satellites = (byte)Mathf.Clamp(solution.usableSatellites, 0, 255);
        _msg.nlos_satellites = (byte)Mathf.Clamp(solution.nlosSatellites, 0, 255);
        _msg.locked_satellites = (byte)Mathf.Clamp(solution.lockedSatellites, 0, 255);
        _msg.hdop = solution.hdop;
        _msg.pdop = solution.pdop;
        _msg.horizontal_sigma = (float)solution.horizontalSigma;
        _msg.error_east = (float)solution.errorEast;
        _msg.error_north = (float)solution.errorNorth;
        _msg.nlos_east = (float)solution.nlosEast;
        _msg.nlos_north = (float)solution.nlosNorth;
        _msg.wrong_fix = solution.wrongFix;
        _msg.wrong_fix_east = (float)solution.wrongFixEast;
        _msg.wrong_fix_north = (float)solution.wrongFixNorth;
        return _msg;
    }

    private static byte ToStateConstant(RtkState state)
    {
        switch (state)
        {
            case RtkState.Fix: return GnssSolutionMsg.STATE_FIX;
            case RtkState.Float: return GnssSolutionMsg.STATE_FLOAT;
            case RtkState.Single: return GnssSolutionMsg.STATE_SINGLE;
            default: return GnssSolutionMsg.STATE_NO_FIX;
        }
    }
}
