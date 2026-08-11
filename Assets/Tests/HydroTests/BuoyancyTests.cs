using System.Collections;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using NaughtyWaterBuoyancy;
using Hydrodynamics;

/// <summary>
/// 体積ベース浮力の物理テスト。
/// 期待値はアルキメデスの原理: F = ρ水(1000) × V排水 × g。
/// 一辺 0.5 m の立方体 (V = 0.125 m^3) は排水質量 125 kg なので、
/// 質量 62.5 kg なら半分沈んだ位置 (中心 = 水面) で釣り合う。
/// </summary>
public class BuoyancyTests
{
    private const float CubeSize = 0.5f;
    private const float CubeVolume = CubeSize * CubeSize * CubeSize; // 0.125 m^3

    private GameObject water;
    private GameObject floater;
    private float savedTimeScale;
    private float savedMaxDelta;

    [SetUp]
    public void SetUp()
    {
        savedTimeScale = Time.timeScale;
        savedMaxDelta = Time.maximumDeltaTime;
        // 実時間で待たずに物理を回す
        Time.timeScale = 20f;
        Time.maximumDeltaTime = 1f;

        // 水面 y=0。メッシュは空で良い (水位は transform.position.y へフォールバック)
        water = new GameObject("TestWater");
        water.tag = WaterVolume.TAG;
        water.transform.position = Vector3.zero;
        var trigger = water.AddComponent<BoxCollider>();
        trigger.isTrigger = true;
        trigger.center = new Vector3(0f, -5f, 0f);
        trigger.size = new Vector3(50f, 10f, 50f);
        water.AddComponent<MeshFilter>().mesh = new Mesh();
        water.AddComponent<WaterVolume>(); // Density = 1 (相対密度、= 1000 kg/m^3)
    }

    [TearDown]
    public void TearDown()
    {
        Time.timeScale = savedTimeScale;
        Time.maximumDeltaTime = savedMaxDelta;
        if (floater != null) Object.Destroy(floater);
        if (water != null) Object.Destroy(water);
    }

    /// <summary>
    /// URDF スポーンと同じ構造を作る: リンク (ArticulationBody、浮力コンポーネントも
    /// ここに付く) の子にコリジョン用 GameObject (Collider のみ)。
    /// </summary>
    private GameObject CreateFloatingCube(float mass, float startY, out GameObject colliderChild)
    {
        var root = new GameObject("FloatingLink");
        root.transform.position = new Vector3(0f, startY, 0f);
        var body = root.AddComponent<ArticulationBody>();
        body.mass = mass;
        body.useGravity = true;

        colliderChild = new GameObject("Collisions");
        colliderChild.transform.SetParent(root.transform, false);
        var box = colliderChild.AddComponent<BoxCollider>();
        box.size = Vector3.one * CubeSize;
        return root;
    }

    private IEnumerator Settle(float seconds = 12f)
    {
        int steps = Mathf.CeilToInt(seconds / Time.fixedDeltaTime);
        for (int i = 0; i < steps; i++)
        {
            yield return new WaitForFixedUpdate();
        }
    }

    // ====================================================================
    // ArticulationFloatingObject (パッケージ側)
    // ====================================================================

    [UnityTest]
    public IEnumerator NoPhantomBodyIsCreatedOnColliderChild()
    {
        floater = CreateFloatingCube(62.5f, 0f, out GameObject child);
        var obj = floater.AddComponent<ArticulationFloatingObject>();
        obj.VolumeMode = BuoyancyVolumeMode.FromGeometry;
        yield return null;

        // RequireComponent(ArticulationBody) 時代はコリジョン側に 1 kg の暗黙ボディが湧いた
        Assert.IsNull(child.GetComponent<ArticulationBody>(),
            "collider child must NOT get its own ArticulationBody");
        Assert.IsNull(floater.GetComponent<Collider>(),
            "no collider must be auto-added to the link (RequireComponent removed)");
        Assert.AreSame(floater.GetComponent<ArticulationBody>(), obj.Body,
            "force target must be the parent link body");
        Assert.AreEqual(CubeVolume, obj.DisplacedVolume, CubeVolume * 0.01f,
            "geometric volume of a 0.5 m cube");
    }

    [UnityTest]
    public IEnumerator HalfDensityCube_FloatsHalfSubmerged_FromGeometry()
    {
        // 62.5 kg / 125 kg 排水 → 平衡は中心 = 水面 (y = 0)
        floater = CreateFloatingCube(62.5f, 0.1f, out GameObject child);
        var obj = floater.AddComponent<ArticulationFloatingObject>();
        obj.VolumeMode = BuoyancyVolumeMode.FromGeometry;
        SetDamping(obj, 3f, 3f);

        yield return Settle();

        Assert.AreEqual(0f, floater.transform.position.y, 0.05f,
            $"half-density cube should settle with its center at the water line, got y={floater.transform.position.y:F3}");
    }

    [UnityTest]
    public IEnumerator FromDensity_UsesRealLinkMass()
    {
        // 比重 0.5 → V = m/(1000*0.5)。質量に依らず半分沈んで釣り合う
        floater = CreateFloatingCube(10f, 0.1f, out GameObject child);
        var obj = floater.AddComponent<ArticulationFloatingObject>();
        obj.Density = 0.5f; // VolumeMode 既定 = FromDensity
        SetDamping(obj, 3f, 3f);

        Assert.AreEqual(10f / (1000f * 0.5f), obj.DisplacedVolume, 1e-6f,
            "FromDensity volume must be m / (1000 * density) using the real link mass");
        yield return Settle();

        Assert.AreEqual(0f, floater.transform.position.y, 0.05f,
            $"relative density 0.5 should settle half submerged, got y={floater.transform.position.y:F3}");
    }

    [UnityTest]
    public IEnumerator DenseCube_Sinks()
    {
        // 比重 2 → 最大浮力 = mg/2 < mg → 沈む
        floater = CreateFloatingCube(20f, 0f, out GameObject child);
        var obj = floater.AddComponent<ArticulationFloatingObject>();
        obj.Density = 2f;

        yield return Settle(6f);

        Assert.Less(floater.transform.position.y, -1f,
            $"relative density 2 should sink, got y={floater.transform.position.y:F3}");
    }

    [UnityTest]
    public IEnumerator ExplicitVolume_OverridesGeometry()
    {
        // 排水体積を幾何の 2 倍に明示 → より浅い喫水で釣り合う (y > 0)
        floater = CreateFloatingCube(62.5f, 0f, out GameObject child);
        var obj = floater.AddComponent<ArticulationFloatingObject>();
        obj.VolumeMode = BuoyancyVolumeMode.Explicit;
        obj.ExplicitVolume = CubeVolume * 2f;
        SetDamping(obj, 3f, 3f);

        yield return Settle();

        Assert.Greater(floater.transform.position.y, 0.03f,
            $"doubled displaced volume should ride higher than half submerged, got y={floater.transform.position.y:F3}");
        Assert.Less(floater.transform.position.y, CubeSize / 2f + 0.01f, "still touching the water");
    }

    // ====================================================================
    // HydrodynamicFloatingObject (リポジトリ側)
    // ====================================================================

    [UnityTest]
    public IEnumerator Hydrodynamic_NoPhantomBody_AndFloatsHalfSubmerged()
    {
        floater = CreateFloatingCube(62.5f, 0.1f, out GameObject child);
        var obj = floater.AddComponent<HydrodynamicFloatingObject>();
        obj.VolumeMode = BuoyancyVolumeMode.FromGeometry;
        // 浮力だけを検証する (抗力系はゼロ速度近傍でほぼ効かないが明示的に切る)
        var p = new HydrodynamicParameters
        {
            enableViscousResistance = false,
            enablePressureDrag = false,
            enableSlammingForce = false,
            enableAirResistance = false,
        };
        obj.Parameters = p;
        SetHydroDamping(obj, 3f, 3f);

        yield return null;
        Assert.IsNull(child.GetComponent<ArticulationBody>(),
            "collider child must NOT get its own ArticulationBody");

        yield return Settle();
        Assert.AreEqual(0f, floater.transform.position.y, 0.05f,
            $"hydrodynamic half-density cube should settle at the water line, got y={floater.transform.position.y:F3}");
    }

    // ====================================================================
    // 体積計算 (純粋関数)
    // ====================================================================

    [Test]
    public void ColliderVolume_Primitives()
    {
        var go = new GameObject("VolumeProbe");
        try
        {
            var box = go.AddComponent<BoxCollider>();
            box.size = new Vector3(0.5f, 0.4f, 0.3f);
            Assert.AreEqual(0.06f, ColliderUtils.CalculateVolume(box), 1e-4f, "box");
            Object.DestroyImmediate(box);

            var sphere = go.AddComponent<SphereCollider>();
            sphere.radius = 0.5f;
            Assert.AreEqual(4f / 3f * Mathf.PI * 0.125f, ColliderUtils.CalculateVolume(sphere), 1e-3f, "sphere");
            Object.DestroyImmediate(sphere);

            var capsule = go.AddComponent<CapsuleCollider>();
            capsule.radius = 0.25f;
            capsule.height = 1f; // 円柱部 0.5 + 球 r=0.25
            float expected = Mathf.PI * 0.0625f * 0.5f + 4f / 3f * Mathf.PI * 0.015625f;
            Assert.AreEqual(expected, ColliderUtils.CalculateVolume(capsule), 1e-3f, "capsule");
            Object.DestroyImmediate(capsule);

            // スケールが効くこと
            go.transform.localScale = new Vector3(2f, 2f, 2f);
            var scaledBox = go.AddComponent<BoxCollider>();
            scaledBox.size = Vector3.one;
            Assert.AreEqual(8f, ColliderUtils.CalculateVolume(scaledBox), 1e-3f, "scaled box");
        }
        finally
        {
            Object.DestroyImmediate(go);
        }
    }

    // ====================================================================
    // helpers
    // ====================================================================

    private static void SetDamping(ArticulationFloatingObject obj, float linear, float angular)
    {
        SetPrivateField(obj, "linearDampingInWater", linear);
        SetPrivateField(obj, "angularDampingInWater", angular);
    }

    private static void SetHydroDamping(HydrodynamicFloatingObject obj, float linear, float angular)
    {
        SetPrivateField(obj, "linearDampingInWater", linear);
        SetPrivateField(obj, "angularDampingInWater", angular);
    }

    private static void SetPrivateField(object target, string name, object value)
    {
        var field = target.GetType().GetField(name,
            System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic);
        Assert.IsNotNull(field, $"field {name} not found on {target.GetType().Name}");
        field.SetValue(target, value);
    }
}
