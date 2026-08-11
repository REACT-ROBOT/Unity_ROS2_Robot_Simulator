using System;
using System.Collections;
using UnityEngine;

/// <summary>
/// simulation_resources.json の settings 要素。全フィールド任意で、
/// 0 (未指定) のものは既定値 (物理 50 Hz / 描画 10 FPS) のまま。
/// </summary>
[Serializable]
public class SimulationSettingsConfig
{
    public float physics_hz;  // 物理演算レート [Hz]。0 = 未指定
    public int target_fps;    // Application.targetFrameRate。0 = 未指定
    public float time_scale;  // 再生/ステップの時間倍率。0 = 未指定 (=1: 実時間)
}

/// <summary>
/// 起動時に settings を Time.fixedDeltaTime / Application.targetFrameRate へ
/// 反映する常駐コンポーネント。シーンには置かず、コードから自分で立ち上がる。
/// </summary>
/// <remarks>
/// FrameRateController.Start は targetFrameRate = 10 をハードコードしていて、
/// スクリプト実行順は不定なので、Start で 1 度反映したうえで 1 フレーム後に
/// もう一度反映し直す。全スクリプトの Start は最初のフレームで終わるため、
/// 2 回目の適用が必ず勝ち、settings がある場合はその値が最終値になる。
/// settings が無い構成では何もしないので、従来の既定値がそのまま残る。
/// 反映後は既存の入力欄の表示も実際の値へ合わせ直す。
/// </remarks>
public class SimulationSettingsApplier : MonoBehaviour
{
    public const int MinTargetFps = 1;
    public const int MaxTargetFps = 1000;
    public const float MinTimeScale = 0.1f;
    public const float MaxTimeScale = 100f;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    private static void Bootstrap()
    {
        if (FindFirstObjectByType<SimulationSettingsApplier>(FindObjectsInactive.Include) != null)
        {
            return;
        }
        new GameObject("SimulationSettings").AddComponent<SimulationSettingsApplier>();
    }

    void Start()
    {
        // まず即時反映。物理レートは他に書き手がいないのでこの時点で確定するし、
        // 最初の FixedUpdate から設定どおりのステップで回したい。
        Apply();
        // targetFrameRate は FrameRateController.Start の 10 と競合しうるので、
        // 全 Start が済んだ次のフレームで確実に上書きする。
        StartCoroutine(ReapplyAfterFirstFrame());
    }

    private IEnumerator ReapplyAfterFirstFrame()
    {
        yield return null;
        Apply();
        SyncInputFields();
    }

    private void Apply()
    {
        SimulationSettingsConfig settings = SimulationResources.Settings;
        if (settings == null)
        {
            return;
        }

        if (settings.physics_hz > 0f)
        {
            float hz = Mathf.Clamp(settings.physics_hz,
                PhysicsRateController.MinPhysicsHz, PhysicsRateController.MaxPhysicsHz);
            Time.fixedDeltaTime = 1f / hz;
            Debug.Log($"[SimulationSettings] physics_hz = {hz} (fixedDeltaTime = {Time.fixedDeltaTime:F6} s)");
        }

        if (settings.target_fps > 0)
        {
            int fps = Mathf.Clamp(settings.target_fps, MinTargetFps, MaxTargetFps);
            Application.targetFrameRate = fps;
            Debug.Log($"[SimulationSettings] target_fps = {fps}");
        }

        if (settings.time_scale > 0f)
        {
            float scale = Mathf.Clamp(settings.time_scale, MinTimeScale, MaxTimeScale);
            SimulationControl.ConfiguredTimeScale = scale;
            // 高倍率では 1 フレームに進められる物理時間が maximumDeltaTime で
            // 頭打ちになるので、倍率ぶんだけ引き上げる (RL 用途はレイテンシより
            // スループット優先)。
            Time.maximumDeltaTime = Mathf.Max(Time.maximumDeltaTime, scale / 3f);
            Debug.Log($"[SimulationSettings] time_scale = {scale}");
        }
    }

    /// <summary>入力欄の表示を実際に効いている値へ合わせる。</summary>
    private void SyncInputFields()
    {
        FrameRateController frameCtl =
            FindFirstObjectByType<FrameRateController>(FindObjectsInactive.Include);
        if (frameCtl != null && frameCtl.frameRateInput != null)
        {
            frameCtl.frameRateInput.text = Application.targetFrameRate.ToString();
        }

        PhysicsRateController physCtl =
            FindFirstObjectByType<PhysicsRateController>(FindObjectsInactive.Include);
        if (physCtl != null)
        {
            physCtl.RefreshDisplayedValue();
        }
    }
}
