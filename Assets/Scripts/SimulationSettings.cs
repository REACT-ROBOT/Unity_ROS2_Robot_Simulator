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

    /// <summary>
    /// Time.maximumDeltaTime [s]。1 フレームで進める物理時間の上限。0 = 未指定
    /// (Unity 既定の 0.3333)。
    /// </summary>
    /// <remarks>
    /// 物理が実時間に追いつかなくなると、Unity は毎フレームこの値ぶんの
    /// FixedUpdate をまとめて回してから 1 回描画する。既定の 0.3333 では
    /// 1 フレーム = 0.333 秒ぶんの物理となり、フレームレートが 3 FPS 前後まで
    /// 落ちる。RTF は物理コストで決まりこの値では変わらないので、小さくすると
    /// 「同じ RTF のまま描画だけ滑らかにする」ことができる。遠隔操縦のように
    /// 映像の滑らかさが要る用途では 0.05 程度まで下げるとよい。
    /// 逆に time_scale を上げる用途では大きいほうがスループットが出る。
    /// </remarks>
    public float max_delta_time;

    /// <summary>
    /// Physics.defaultSolverIterations。0 = 未指定 (Unity 既定の 6)。
    /// </summary>
    /// <remarks>
    /// 接触の求解精度と 1 ステップのコストを直接決める。剛体が積み重なる場面
    /// (ディスクの装填など) では効くが、台数が増えて実時間に収まらないときは
    /// 下げると効く。GUI の入力欄と同じものを起動時から指定できるようにした。
    /// </remarks>
    public int solver_iterations;

    /// <summary>
    /// Physics.defaultSolverVelocityIterations。0 = 未指定。
    /// solver_iterations だけ指定した場合は GUI と同じ clamp(iter/4, 1, 8)。
    /// </summary>
    public int solver_velocity_iterations;
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
    // 物理 1 ステップぶんを下回らせない (それ以下にしても 1 ステップは必ず回る)。
    public const float MinMaxDeltaTime = 0.001f;
    public const float MaxMaxDeltaTime = 10f;
    public const int MinSolverIterations = 1;
    public const int MaxSolverIterations = 255;

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
            // スループット優先)。max_delta_time が明示されていればそちらを優先
            // するので、ここでは触らない。
            if (settings.max_delta_time <= 0f)
            {
                Time.maximumDeltaTime = Mathf.Max(Time.maximumDeltaTime, scale / 3f);
            }
            Debug.Log($"[SimulationSettings] time_scale = {scale}");
        }

        // time_scale の後に置く。両方指定されたときは明示値を最終値にする。
        if (settings.max_delta_time > 0f)
        {
            float maxDelta = Mathf.Clamp(settings.max_delta_time,
                MinMaxDeltaTime, MaxMaxDeltaTime);
            Time.maximumDeltaTime = maxDelta;
            Debug.Log($"[SimulationSettings] max_delta_time = {maxDelta} s " +
                      $"(1 フレームで進める物理時間の上限)");
        }

        if (settings.solver_iterations > 0)
        {
            int iter = Mathf.Clamp(settings.solver_iterations,
                MinSolverIterations, MaxSolverIterations);
            Physics.defaultSolverIterations = iter;
            // 明示が無ければ GUI (SolverIterationController) と同じ導出にする。
            int velIter = settings.solver_velocity_iterations > 0
                ? settings.solver_velocity_iterations
                : Mathf.Min(Mathf.Max(1, iter / 4), 8);
            Physics.defaultSolverVelocityIterations =
                Mathf.Clamp(velIter, MinSolverIterations, MaxSolverIterations);
            Debug.Log($"[SimulationSettings] solver_iterations = {iter}, " +
                      $"solver_velocity_iterations = {Physics.defaultSolverVelocityIterations}");
        }
        else if (settings.solver_velocity_iterations > 0)
        {
            Physics.defaultSolverVelocityIterations = Mathf.Clamp(
                settings.solver_velocity_iterations, MinSolverIterations, MaxSolverIterations);
            Debug.Log($"[SimulationSettings] solver_velocity_iterations = " +
                      $"{Physics.defaultSolverVelocityIterations}");
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
