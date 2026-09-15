using System;
using System.Collections.Generic;
using System.IO;
using NUnit.Framework;
using UnityEngine;
using SdfWorld;

/// <summary>
/// SdfWorldImporter の変換規則のテスト。座標系の期待値は
/// ROS (X 前 Y 左 Z 上, 右手) → Unity (X 右 Y 上 Z 前, 左手):
/// pos (x,y,z) → (-y, z, x)、寸法 (x,y,z) → (y, z, x)。
/// </summary>
public class SdfWorldImporterTests
{
    private static SdfWorldData Parse(string worldXml, string baseDir = null,
        IReadOnlyList<string> roots = null)
    {
        bool ok = SdfWorldImporter.TryParseWorld(worldXml, baseDir, roots,
            out SdfWorldData data, out string error);
        Assert.IsTrue(ok, $"parse failed: {error}");
        return data;
    }

    private static string Wrap(string worldBody, string worldName = "test")
    {
        return $"<?xml version=\"1.0\"?><sdf version=\"1.7\"><world name=\"{worldName}\">{worldBody}</world></sdf>";
    }

    private static void AssertVector(Vector3 expected, Vector3 actual, string label)
    {
        Assert.Less((expected - actual).magnitude, 1e-3f,
            $"{label}: expected {expected}, got {actual}");
    }

    private static void AssertRotation(Quaternion expected, Vector3 actualEuler, string label)
    {
        float angle = Quaternion.Angle(expected, Quaternion.Euler(actualEuler));
        Assert.Less(angle, 0.1f, $"{label}: expected {expected.eulerAngles}, got {actualEuler}");
    }

    // ====================================================================
    // 基本形状と座標変換
    // ====================================================================

    [Test]
    public void Box_PositionAndSizeAreConverted()
    {
        var data = Parse(Wrap(@"
            <model name='wall'><static>true</static><pose>3 0 1 0 0 0</pose>
              <link name='link'><visual name='v'>
                <geometry><box><size>0.2 4 2</size></box></geometry>
              </visual></link></model>"));
        Assert.AreEqual(1, data.objects.Count);
        var obj = data.objects[0];
        Assert.AreEqual("Cube", obj.type);
        AssertVector(new Vector3(0, 1, 3), obj.position, "position");
        AssertVector(new Vector3(4, 2, 0.2f), obj.scale, "scale");
        AssertRotation(Quaternion.identity, obj.rotationEuler, "rotation");
    }

    [Test]
    public void Yaw_BecomesNegativeUnityYRotation()
    {
        var data = Parse(Wrap(@"
            <model name='m'><pose>0 0 0 0 0 1.5707963</pose>
              <link name='l'><visual name='v'>
                <geometry><box><size>1 1 1</size></box></geometry>
              </visual></link></model>"));
        // ROS の +90° yaw は Unity では Y 軸回りに -90°
        AssertRotation(Quaternion.Euler(0, -90, 0), data.objects[0].rotationEuler, "yaw");
    }

    [Test]
    public void PoseComposition_ModelLinkVisual()
    {
        // model が yaw 90° を持ち、link が前方 (x=2) にずれる → ワールドでは y=2 (ROS)
        var data = Parse(Wrap(@"
            <model name='m'><pose>1 0 0 0 0 1.5707963</pose>
              <link name='l'><pose>2 0 0.5 0 0 0</pose><visual name='v'>
                <geometry><sphere><radius>0.5</radius></sphere></geometry>
              </visual></link></model>"));
        // ROS: (1,0,0) + yaw90·(2,0,0.5) = (1,2,0.5) → Unity (-2, 0.5, 1)
        AssertVector(new Vector3(-2, 0.5f, 1), data.objects[0].position, "composed position");
    }

    [Test]
    public void Cylinder_ScaleUsesUnityHalfHeight()
    {
        var data = Parse(Wrap(@"
            <model name='p'><link name='l'><visual name='v'>
              <geometry><cylinder><radius>0.3</radius><length>1.5</length></cylinder></geometry>
            </visual></link></model>"));
        var obj = data.objects[0];
        Assert.AreEqual("Cylinder", obj.type);
        AssertVector(new Vector3(0.6f, 0.75f, 0.6f), obj.scale, "cylinder scale");
    }

    [Test]
    public void Plane_ScaleAndDefaultNormal()
    {
        var data = Parse(Wrap(@"
            <model name='g'><link name='l'><visual name='v'>
              <geometry><plane><normal>0 0 1</normal><size>100 60</size></plane></geometry>
            </visual></link></model>"));
        var obj = data.objects[0];
        Assert.AreEqual("Plane", obj.type);
        // Unity の Plane は 10x10 m。size (sx=100, sy=60) → scale (sy/10, 1, sx/10)
        AssertVector(new Vector3(6, 1, 10), obj.scale, "plane scale");
        AssertRotation(Quaternion.identity, obj.rotationEuler, "plane rotation");
    }

    [Test]
    public void CollisionOnlyLink_UsesCollisionGeometry()
    {
        var data = Parse(Wrap(@"
            <model name='m'><link name='l'>
              <collision name='c'><geometry><box><size>1 1 1</size></box></geometry></collision>
            </link></model>"));
        Assert.AreEqual(1, data.objects.Count);
        Assert.AreEqual("Cube", data.objects[0].type);
    }

    [Test]
    public void Material_DiffuseBecomesColor()
    {
        var data = Parse(Wrap(@"
            <model name='m'><link name='l'><visual name='v'>
              <geometry><box><size>1 1 1</size></box></geometry>
              <material><diffuse>0.6 0.3 0.1 1</diffuse></material>
            </visual></link></model>"));
        var obj = data.objects[0];
        Assert.IsTrue(obj.hasColor);
        Assert.Less(Mathf.Abs(obj.color.r - 0.6f), 1e-3f);
        Assert.Less(Mathf.Abs(obj.color.g - 0.3f), 1e-3f);
    }

    // ====================================================================
    // ライト
    // ====================================================================

    [Test]
    public void DirectionalLight_DirectionIsConverted()
    {
        var data = Parse(Wrap(@"
            <light name='sun' type='directional'>
              <pose>0 0 10 0 0 0</pose>
              <direction>0 0 -1</direction>
              <diffuse>0.8 0.8 0.8 1</diffuse>
            </light>"));
        var obj = data.objects[0];
        Assert.AreEqual("Directional Light", obj.type);
        AssertVector(new Vector3(0, 10, 0), obj.position, "light position");
        // 真下向き: Unity では forward が (0,-1,0)
        AssertRotation(Quaternion.LookRotation(Vector3.down), obj.rotationEuler, "light rotation");
        Assert.IsTrue(obj.hasColor);
    }

    // ====================================================================
    // include とメッシュ
    // ====================================================================

    [Test]
    public void Include_ResolvesModelDirAndAppliesPoseOverride()
    {
        string root = MakeTempDir();
        try
        {
            string modelDir = Path.Combine(root, "box_model");
            Directory.CreateDirectory(modelDir);
            File.WriteAllText(Path.Combine(modelDir, "model.sdf"),
                "<?xml version=\"1.0\"?><sdf version=\"1.7\"><model name=\"box\">" +
                "<pose>99 99 99 0 0 0</pose>" + // include の pose に置き換えられるはず
                "<link name=\"l\"><visual name=\"v\">" +
                "<geometry><box><size>1 2 3</size></box></geometry>" +
                "</visual></link></model></sdf>");

            var data = Parse(Wrap(@"
                <include>
                  <uri>model://box_model</uri>
                  <pose>1 0 0 0 0 0</pose>
                  <static>true</static>
                </include>"), null, new[] { root });

            Assert.AreEqual(1, data.objects.Count, string.Join("; ", data.messages));
            AssertVector(new Vector3(0, 0, 1), data.objects[0].position, "include pose");
            AssertVector(new Vector3(2, 3, 1), data.objects[0].scale, "include scale");
            Assert.IsFalse(data.hasMissingAssets);
        }
        finally
        {
            Directory.Delete(root, true);
        }
    }

    [Test]
    public void Include_MissingModelIsReported()
    {
        var data = Parse(Wrap("<include><uri>model://no_such_model</uri></include>"),
            null, new[] { "/nonexistent" });
        Assert.AreEqual(0, data.objects.Count);
        Assert.IsTrue(data.hasMissingAssets);
    }

    [Test]
    public void Mesh_RelativeUriResolvesAgainstBaseDir()
    {
        string root = MakeTempDir();
        try
        {
            string meshPath = Path.Combine(root, "meshes", "part.stl");
            Directory.CreateDirectory(Path.GetDirectoryName(meshPath));
            File.WriteAllBytes(meshPath, new byte[84]); // 中身は解決だけなので空で良い

            var data = Parse(Wrap(@"
                <model name='m'><link name='l'><visual name='v'>
                  <geometry><mesh><uri>meshes/part.stl</uri><scale>1 2 3</scale></mesh></geometry>
                </visual></link></model>"), root);

            Assert.AreEqual(1, data.objects.Count, string.Join("; ", data.messages));
            var obj = data.objects[0];
            Assert.AreEqual("RosMesh", obj.type);
            Assert.AreEqual(Path.GetFullPath(meshPath), obj.meshPath);
            AssertVector(new Vector3(2, 3, 1), obj.scale, "mesh scale");
        }
        finally
        {
            Directory.Delete(root, true);
        }
    }

    [Test]
    public void Mesh_MissingFileSetsMissingAssets()
    {
        var data = Parse(Wrap(@"
            <model name='m'><link name='l'><visual name='v'>
              <geometry><mesh><uri>model://nope/mesh.stl</uri></mesh></geometry>
            </visual></link></model>"));
        Assert.AreEqual(0, data.objects.Count);
        Assert.IsTrue(data.hasMissingAssets);
    }

    // ====================================================================
    // 未対応要素と異常系
    // ====================================================================

    [Test]
    public void NonStaticModel_IsPlacedWithMessage()
    {
        var data = Parse(Wrap(@"
            <model name='crate'><link name='l'><visual name='v'>
              <geometry><box><size>1 1 1</size></box></geometry>
            </visual></link></model>"));
        Assert.AreEqual(1, data.objects.Count);
        Assert.IsTrue(data.messages.Exists(m => m.Contains("static")));
    }

    [Test]
    public void Actor_IsUnsupported()
    {
        var data = Parse(Wrap("<actor name='walker'></actor>"));
        Assert.IsTrue(data.hasUnsupportedElements);
    }

    [Test]
    public void PhysicsAndPlugins_AreSkippedSilently()
    {
        var data = Parse(Wrap(@"
            <physics type='ode'><max_step_size>0.001</max_step_size></physics>
            <plugin name='p' filename='libp.so'/>
            <gravity>0 0 -9.8</gravity>"));
        Assert.IsFalse(data.hasUnsupportedElements);
        Assert.AreEqual(0, data.objects.Count);
    }

    [Test]
    public void UnknownGeometry_IsUnsupported()
    {
        var data = Parse(Wrap(@"
            <model name='m'><link name='l'><visual name='v'>
              <geometry><heightmap><uri>x</uri></heightmap></geometry>
            </visual></link></model>"));
        Assert.AreEqual(0, data.objects.Count);
        Assert.IsTrue(data.hasUnsupportedElements);
    }

    [Test]
    public void ModelOnlySdf_IsRejected()
    {
        bool ok = SdfWorldImporter.TryParseWorld(
            "<sdf version=\"1.7\"><model name=\"m\"/></sdf>", null, null,
            out _, out string error);
        Assert.IsFalse(ok);
        Assert.IsTrue(error.Contains("world"));
    }

    [Test]
    public void BrokenXml_IsRejected()
    {
        bool ok = SdfWorldImporter.TryParseWorld("<sdf><world", null, null, out _, out _);
        Assert.IsFalse(ok);
    }

    [Test]
    public void WorldName_IsExtracted()
    {
        var data = Parse(Wrap("", "warehouse"));
        Assert.AreEqual("warehouse", data.worldName);
    }

    [Test]
    public void LooksLikeSdfWorld_Heuristic()
    {
        Assert.IsTrue(SdfWorldImporter.LooksLikeSdfWorld("<sdf version='1.7'><world name='x'/></sdf>"));
        Assert.IsFalse(SdfWorldImporter.LooksLikeSdfWorld("{\"objects\":[]}"));
        Assert.IsFalse(SdfWorldImporter.LooksLikeSdfWorld("<sdf version='1.7'><model name='x'/></sdf>"));
    }

    [Test]
    public void NestedModel_ComposesPose()
    {
        var data = Parse(Wrap(@"
            <model name='outer'><static>true</static><pose>1 0 0 0 0 0</pose>
              <model name='inner'><pose>0 2 0 0 0 0</pose>
                <link name='l'><visual name='v'>
                  <geometry><box><size>1 1 1</size></box></geometry>
                </visual></link></model></model>"));
        Assert.AreEqual(1, data.objects.Count);
        // ROS (1,2,0) → Unity (-2, 0, 1)
        AssertVector(new Vector3(-2, 0, 1), data.objects[0].position, "nested pose");
        // outer が static なので inner の「static でない」報告は出ない
        Assert.IsFalse(data.messages.Exists(m => m.Contains("static でない")));
    }

    // ====================================================================
    // actor (動く障害物)
    // ====================================================================

    [Test]
    public void Actor_TrajectoryBecomesMotionWaypoints()
    {
        var data = Parse(Wrap(@"
            <actor name='patrol'>
              <link name='body'><visual name='v'>
                <geometry><box><size>1 1 1</size></box></geometry>
              </visual></link>
              <script>
                <loop>true</loop>
                <trajectory id='0' type='walk'>
                  <waypoint><time>0</time><pose>0 2 0.5 0 0 0</pose></waypoint>
                  <waypoint><time>2</time><pose>2 2 0.5 0 0 1.5707963</pose></waypoint>
                  <waypoint><time>4</time><pose>0 2 0.5 0 0 0</pose></waypoint>
                </trajectory>
              </script>
            </actor>"));
        Assert.AreEqual(1, data.objects.Count, string.Join("; ", data.messages));
        var obj = data.objects[0];
        Assert.IsNotNull(obj.motionWaypoints);
        Assert.AreEqual(3, obj.motionWaypoints.Count);
        // ROS (0,2,0.5) → Unity (-2, 0.5, 0)
        AssertVector(new Vector3(-2, 0.5f, 0), obj.motionWaypoints[0].position, "waypoint 0");
        AssertVector(new Vector3(-2, 0.5f, 2), obj.motionWaypoints[1].position, "waypoint 1");
        Assert.AreEqual(-90f, obj.motionWaypoints[1].yawDeg, 0.1f, "yaw conversion");
        Assert.AreEqual(2f, obj.motionWaypoints[1].time, 1e-3f, "waypoint time");
    }

    [Test]
    public void Actor_WithoutTrajectory_IsStatic()
    {
        var data = Parse(Wrap(@"
            <actor name='statue'>
              <pose>1 0 0 0 0 0</pose>
              <link name='body'><visual name='v'>
                <geometry><sphere><radius>0.3</radius></sphere></geometry>
              </visual></link>
            </actor>"));
        Assert.AreEqual(1, data.objects.Count);
        Assert.IsNull(data.objects[0].motionWaypoints);
        AssertVector(new Vector3(0, 0, 1), data.objects[0].position, "static actor pose");
    }

    [Test]
    public void Actor_WithoutLink_IsUnsupported()
    {
        var data = Parse(Wrap("<actor name='ghost'><skin><filename>x.dae</filename></skin></actor>"));
        Assert.AreEqual(0, data.objects.Count);
        Assert.IsTrue(data.hasUnsupportedElements);
    }

    private static string MakeTempDir()
    {
        string dir = Path.Combine(Application.temporaryCachePath,
            "sdf_test_" + Guid.NewGuid().ToString("N").Substring(0, 8));
        Directory.CreateDirectory(dir);
        return dir;
    }
}
