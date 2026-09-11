using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;
using UnitySensors.DataType.Sensor;
using UnitySensors.Sensor.MagneticGuide;

/// <summary>
/// 磁気誘導センサが、シーン内の磁気テープ (MagneticTape 付きトリガコライダ) を
/// レイキャストで読めるか。ソルバ単体は UnitySensors パッケージ側のテストが見るので、
/// ここでは幾何 (左右・高さ・遮蔽) と URDF 由来の軸規約を確かめる。
/// </summary>
/// <remarks>
/// センサは URDF リンクの規約で「左 = -transform.right、下 = -transform.up」。
/// 単位回転で置くと Unity の -X が左になる。
/// </remarks>
public class MagneticGuideSensorTests
{
    readonly List<GameObject> spawned = new List<GameObject>();

    [TearDown]
    public void TearDown()
    {
        foreach (GameObject go in spawned)
        {
            if (go != null) Object.DestroyImmediate(go);
        }
        spawned.Clear();
    }

    /// <summary>厚さ 1 mm の帯を y=0 の床面に置く。x が幅方向 (Unity +x = センサの右)。</summary>
    GameObject Tape(float centerX, float width, MagneticPolarity polarity, float topY = 0.0f, float lengthZ = 1.0f)
    {
        var go = new GameObject($"tape_{polarity}_{centerX}");
        var box = go.AddComponent<BoxCollider>();
        box.isTrigger = true;
        box.size = new Vector3(width, 0.001f, lengthZ);
        go.transform.position = new Vector3(centerX, topY - 0.0005f, 0.0f);
        go.AddComponent<MagneticTape>().polarity = polarity;
        spawned.Add(go);
        return go;
    }

    MagneticGuideSensor Sensor(float height, float minHeight = 0.01f, float maxHeight = 0.06f)
    {
        var go = new GameObject("uds");
        go.transform.position = new Vector3(0.0f, height, 0.0f);
        var sensor = go.AddComponent<MagneticGuideSensor>();
        sensor.Configure(0.16f, 0.001f, minHeight, maxHeight, MagneticForkSelection.Nearest, 0.0f);
        spawned.Add(go);
        return sensor;
    }

    static MagneticGuideReading Read(MagneticGuideSensor sensor)
    {
        Physics.SyncTransforms();
        return sensor.Measure();
    }

    [Test]
    public void CentredTape_ReadsZero()
    {
        Tape(0.0f, 0.025f, MagneticPolarity.Track);
        var r = Read(Sensor(0.03f));
        Assert.IsTrue(r.trackDetected);
        Assert.AreEqual(0.0f, r.trackPosition, 0.0015f);
        Assert.IsFalse(r.leftMarkerDetected);
        Assert.IsFalse(r.rightMarkerDetected);
    }

    [Test]
    public void TapeOnTheRobotsLeft_IsPositive()
    {
        // 左 = Unity -X。テープを x=-0.03 に置くと +30 mm と読める (ROS +y)。
        Tape(-0.03f, 0.025f, MagneticPolarity.Track);
        var r = Read(Sensor(0.03f));
        Assert.IsTrue(r.trackDetected);
        Assert.AreEqual(0.030f, r.trackPosition, 0.0015f);
    }

    [Test]
    public void SensorYaw_FollowsTheLink()
    {
        // センサを Y 軸まわりに 90° 回すとバーは世界 Z に沿う (right=(0,0,-1) → 左=+Z)。
        // 世界 X に沿う 25 mm 幅の帯 (z=0) を、z=+0.04 に置いたバーで読むと
        // 帯はバー中心から左方向へ -0.04 にある。
        var tape = Tape(0.0f, 0.3f, MagneticPolarity.Track, lengthZ: 0.025f);
        var sensor = Sensor(0.03f);
        sensor.transform.rotation = Quaternion.Euler(0, 90, 0);
        sensor.transform.position = new Vector3(0.0f, 0.03f, 0.04f);
        var r = Read(sensor);
        Assert.IsTrue(r.trackDetected);
        Assert.AreEqual(-0.040f, r.trackPosition, 0.0015f);
    }

    [Test]
    public void TooHighOrTooLow_NoTrack()
    {
        Tape(0.0f, 0.025f, MagneticPolarity.Track);
        Assert.IsFalse(Read(Sensor(0.08f)).trackDetected, "60 mm を超えると磁場が弱くて見えない");
        Assert.IsFalse(Read(Sensor(0.005f)).trackDetected, "10 mm 未満は擦っている");
        Assert.IsTrue(Read(Sensor(0.05f)).trackDetected);
    }

    [Test]
    public void Markers_LeftAndRightOfTheTrack()
    {
        Tape(0.0f, 0.025f, MagneticPolarity.Track);
        Tape(-0.045f, 0.025f, MagneticPolarity.Marker);   // 左 (ROS +y) にマーカ
        var r = Read(Sensor(0.03f));
        Assert.IsTrue(r.trackDetected);
        Assert.IsTrue(r.leftMarkerDetected);
        Assert.IsFalse(r.rightMarkerDetected);

        Tape(0.045f, 0.025f, MagneticPolarity.Marker);    // 右にも
        r = Read(Sensor(0.03f));
        Assert.IsTrue(r.rightMarkerDetected);
    }

    [Test]
    public void Fork_ReportsBothTracks()
    {
        Tape(-0.04f, 0.025f, MagneticPolarity.Track);
        Tape(0.04f, 0.025f, MagneticPolarity.Track);
        var r = Read(Sensor(0.03f));
        Assert.AreEqual(2, r.trackPositions.Length);
        Assert.AreEqual(0.040f, r.trackPositions[0], 0.0015f, "左から");
        Assert.AreEqual(-0.040f, r.trackPositions[1], 0.0015f);
    }

    [Test]
    public void SolidObjectBetweenSensorAndTape_ShieldsIt()
    {
        Tape(0.0f, 0.025f, MagneticPolarity.Track);
        var plate = GameObject.CreatePrimitive(PrimitiveType.Cube);
        plate.transform.position = new Vector3(0.0f, 0.015f, 0.0f);
        plate.transform.localScale = new Vector3(0.3f, 0.002f, 0.3f);
        spawned.Add(plate);
        Assert.IsFalse(Read(Sensor(0.03f)).trackDetected);
    }
}
