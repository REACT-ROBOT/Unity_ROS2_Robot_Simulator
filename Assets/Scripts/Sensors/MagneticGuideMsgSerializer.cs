using UnityEngine;
using RosMessageTypes.SimulationExtraInterfaces;

using UnitySensors.Attribute;
using UnitySensors.DataType.Sensor;
using UnitySensors.Interface.Sensor;
using UnitySensors.ROS.Serializer;
using UnitySensors.ROS.Serializer.Std;

/// <summary>
/// <see cref="IMagneticGuideInterface"/> の読みを simulation_extra_interfaces/MagneticGuide
/// に詰める。メッセージ型がこのシミュレータ固有なので UnitySensorsROS ではなくここに置く。
/// </summary>
[System.Serializable]
public class MagneticGuideMsgSerializer : RosMsgSerializer<MagneticGuideMsg>
{
    [SerializeField, Interface(typeof(IMagneticGuideInterface))]
    private Object _source;
    [SerializeField]
    private HeaderSerializer _header;

    private IMagneticGuideInterface _sourceInterface;

    public void Configure(IMagneticGuideInterface source, HeaderSerializer header)
    {
        _source = source as Object;
        _sourceInterface = source;
        _header = header;
    }

    public override void Init()
    {
        base.Init();
        _header.Init();
        _sourceInterface = _source as IMagneticGuideInterface;
        _msg.track_positions = System.Array.Empty<float>();
    }

    public override MagneticGuideMsg Serialize()
    {
        MagneticGuideReading reading = _sourceInterface.reading;
        _msg.header = _header.Serialize();
        _msg.track_detected = reading.trackDetected;
        _msg.position = reading.trackPosition;
        _msg.left_marker = reading.leftMarkerDetected;
        _msg.right_marker = reading.rightMarkerDetected;
        _msg.track_positions = reading.trackPositions ?? System.Array.Empty<float>();
        return _msg;
    }
}
