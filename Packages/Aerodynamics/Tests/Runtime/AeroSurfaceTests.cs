using NUnit.Framework;
using UnityEngine;
using Aerodynamics;

/// <summary>
/// AeroSurface の準定常揚力モデルのサニティテスト。
/// 期待値は薄翼理論 + 有限翼補正:
///   Cl = a_corr * α,  a_corr = a * AR / (AR + 2(AR+4)/(AR+4)),  L = Cl * q * S
/// chord=1, span=5 (AR=5, S=5): a_corr = 6.28 * 5/7 = 4.486
/// 係数レート制限は「初回呼び出しは無制限」なので、測定ごとに新しいサーフェスを使う。
/// </summary>
public class AeroSurfaceTests
{
    private const float Chord = 1f;
    private const float Span = 5f;
    private const float AirDensity = 1.225f;
    private const float WaterDensity = 1027f;
    private const float CorrectedSlope = 6.28f * 5f / 7f; // 4.486

    private readonly System.Collections.Generic.List<GameObject> spawned =
        new System.Collections.Generic.List<GameObject>();

    [TearDown]
    public void TearDown()
    {
        foreach (var go in spawned)
        {
            Object.DestroyImmediate(go);
        }
        spawned.Clear();
    }

    /// <summary>ピッチ角 [deg] の翼に前方から流速をあて、1 回だけ力を計算する。</summary>
    private Vector3 MeasureForce(float pitchDeg, float speed, float density)
    {
        var go = new GameObject($"Wing_{pitchDeg}deg");
        spawned.Add(go);
        // Unity は左手系なので、機首上げは X 軸まわりの負回転
        go.transform.rotation = Quaternion.Euler(-pitchDeg, 0f, 0f);

        var surface = go.AddComponent<AeroSurface>();
        var p = new AeroSurfaceParameters
        {
            chord = Chord,
            span = Span,
            autoAspectRatio = true,
            liftSlope = 6.28f,
            zeroLiftAoA = 0f,
            fluidMedium = FluidMedium.Air,
            airDensity = AirDensity,
            waterDensity = WaterDensity,
            numElements = 1,
        };
        SetInlineParameters(surface, p);

        // 機体が +Z へ speed で進む → 相対流体速度は -Z 方向
        Vector3 fluidVelocity = Vector3.back * speed;
        return surface.CalculateForces(fluidVelocity, density, Vector3.zero).force;
    }

    [Test]
    public void ZeroAoA_SymmetricWing_ProducesNoLift()
    {
        Vector3 force = MeasureForce(0f, 30f, AirDensity);
        float q = 0.5f * AirDensity * 30f * 30f;
        float qS = q * Chord * Span;

        Assert.Less(Mathf.Abs(force.y), 0.02f * qS,
            $"symmetric wing at zero AoA should make ~no lift, got {force}");
        Assert.Less(force.z, 0f, $"drag must oppose motion (+Z), got {force}");
    }

    [Test]
    public void FiveDegreesPitchUp_LiftMatchesThinAirfoilTheory()
    {
        Vector3 force = MeasureForce(5f, 30f, AirDensity);
        float alpha = 5f * Mathf.Deg2Rad;
        float q = 0.5f * AirDensity * 30f * 30f;
        float expectedLift = CorrectedSlope * alpha * q * Chord * Span; // ≈ 1079 N

        Assert.Greater(force.y, 0f, $"pitch-up must lift upward, got {force}");
        Assert.AreEqual(expectedLift, force.y, expectedLift * 0.25f,
            $"lift should match a_corr*α*q*S ≈ {expectedLift:F0} N, got {force.y:F0} N");
    }

    [Test]
    public void Lift_IsLinearInSmallAoA()
    {
        float lift2 = MeasureForce(2f, 30f, AirDensity).y;
        float lift4 = MeasureForce(4f, 30f, AirDensity).y;
        Assert.Greater(lift2, 0f);
        Assert.AreEqual(2f, lift4 / lift2, 0.2f,
            $"lift should double from 2° to 4°, got {lift2:F1} → {lift4:F1}");
    }

    [Test]
    public void Lift_ScalesWithDynamicPressure()
    {
        float lift30 = MeasureForce(5f, 30f, AirDensity).y;
        float lift60 = MeasureForce(5f, 60f, AirDensity).y;
        Assert.AreEqual(4f, lift60 / lift30, 0.4f,
            $"doubling speed should quadruple lift, got {lift30:F1} → {lift60:F1}");
    }

    [Test]
    public void Lift_ScalesWithFluidDensity()
    {
        float liftAir = MeasureForce(5f, 5f, AirDensity).y;
        float liftWater = MeasureForce(5f, 5f, WaterDensity).y;
        float ratio = liftWater / liftAir;
        float expected = WaterDensity / AirDensity; // ≈ 838
        Assert.AreEqual(expected, ratio, expected * 0.15f,
            $"lift should scale with density, got ratio {ratio:F0} (expected ≈ {expected:F0})");
    }

    private static void SetInlineParameters(AeroSurface surface, AeroSurfaceParameters parameters)
    {
        var field = typeof(AeroSurface).GetField("inlineParameters",
            System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic);
        Assert.IsNotNull(field, "inlineParameters field not found");
        field.SetValue(surface, parameters);
    }
}
