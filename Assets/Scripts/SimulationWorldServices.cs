using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.SceneManagement;

using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;

using RosMessageTypes.SimulationInterfaces;

/// <summary>
/// simulation_interfaces の world 系サービスと step_simulation。
/// </summary>
/// <remarks>
/// このシミュレータの「ワールド」は ObjectSpawner が保存するシーン JSON
/// (SavedSceneData) とする。Unity のシーンを切り替える手もあるが、それだと
/// ワールドを増やすたびにプレイヤーのビルドが要る。JSON ならランタイムで完結し、
/// GUI で組んだ景観をそのまま LoadWorld で配れる。
///
/// 地面や既定のライトといった組み込みシーンの土台は常に在り、ワールドはその上に
/// 載る景観として扱う。起動直後は組み込みシーンを「ロード済みのワールド」と見なす
/// ので、起動直後が STATE_STOPPED であるという従来の挙動は変わらない。
/// </remarks>
public partial class SimulationControl
{
    [Header("ワールド (シーン JSON) の読み込み先")]
    [Tooltip("未設定ならシーン内の ObjectSpawner を自動で探す")]
    [SerializeField]
    private ObjectSpawner m_ObjectSpawner;

    // 現在ロードされているワールド。UnloadWorld 後は null。
    private WorldResourceMsg m_CurrentWorld;

    // step_simulation の進行状態。
    private bool m_Stepping;
    private TaskCompletionSource<bool> m_StepCompletion;
    private Coroutine m_StepCoroutine;

    // 1 回の step_simulation で進めてよいステップ数の上限。uint64 をそのまま
    // 受けると、桁を間違えた要求でシミュレータが何時間も戻ってこなくなる。
    private const int k_MaxStepsPerCall = 100000;

    private bool IsWorldLoaded => m_CurrentWorld != null;

    /// <summary>
    /// シーン JSON から WorldResource を組み立てる。world_resource は呼び出し側で入れる。
    /// </summary>
    /// <remarks>
    /// name / description / tags はワールドファイルに書かれていればそれを使い、
    /// 無ければファイル名と既定の説明で埋める。GetAvailableWorlds と
    /// GetCurrentWorld / LoadWorld が同じ内容を返すよう、ここに寄せている。
    /// </remarks>
    private static WorldResourceMsg DescribeWorld(SavedSceneData sceneData, string fallbackName)
    {
        return new WorldResourceMsg
        {
            name = !string.IsNullOrEmpty(sceneData.name) ? sceneData.name : fallbackName,
            world_resource = new ResourceMsg { uri = "", resource_string = "" },
            description = !string.IsNullOrEmpty(sceneData.description)
                ? sceneData.description
                : $"Scene JSON with {sceneData.objects.Count} object(s)",
            tags = sceneData.tags ?? Array.Empty<string>()
        };
    }

    /// <summary>起動直後の組み込みシーンを、ロード済みのワールドとして登録する。</summary>
    private void InitializeWorld()
    {
        if (m_ObjectSpawner == null)
        {
            m_ObjectSpawner = FindFirstObjectByType<ObjectSpawner>();
        }

        string sceneName = SceneManager.GetActiveScene().name;
        m_CurrentWorld = new WorldResourceMsg
        {
            name = sceneName,
            // 組み込みシーンはファイルとして取り出せないので uri は空。
            // LoadWorld で復元できるものではない、という意味でもある。
            world_resource = new ResourceMsg { uri = "", resource_string = "" },
            description = "Built-in scene shipped with the simulator",
            tags = Array.Empty<string>()
        };
    }

    // ====================================================================
    // world 系サービス
    // ====================================================================

    /// <summary>load_world サービス。景観を差し替えて停止状態に戻す。</summary>
    private LoadWorldResponse LoadWorld(LoadWorldRequest request)
    {
        var response = new LoadWorldResponse();
        response.world = new WorldResourceMsg();

        string uri = request.world_resource != null ? request.world_resource.uri : null;
        string resourceString = request.world_resource != null ? request.world_resource.resource_string : null;

        string json;
        string worldName;
        string resolvedUri = "";
        if (!string.IsNullOrEmpty(uri))
        {
            if (!Uri.TryCreate(uri, UriKind.Absolute, out Uri parsed) || !parsed.IsFile)
            {
                response.result.result = LoadWorldResponse.UNSUPPORTED_FORMAT;
                response.result.error_message = $"Only local file URIs are supported, got '{uri}'";
                return response;
            }
            string path = parsed.LocalPath;
            if (!string.Equals(Path.GetExtension(path), ".json", StringComparison.OrdinalIgnoreCase))
            {
                response.result.result = LoadWorldResponse.UNSUPPORTED_FORMAT;
                response.result.error_message =
                    "World files are ObjectSpawner scene JSON; expected a .json path";
                return response;
            }
            if (!File.Exists(path))
            {
                response.result.result = LoadWorldResponse.MISSING_ASSETS;
                response.result.error_message = $"World file not found: {path}";
                return response;
            }
            try
            {
                json = File.ReadAllText(path);
            }
            catch (Exception e)
            {
                response.result.result = LoadWorldResponse.RESOURCE_PARSE_ERROR;
                response.result.error_message = $"Cannot read {path}: {e.Message}";
                return response;
            }
            worldName = Path.GetFileNameWithoutExtension(path);
            resolvedUri = new Uri(Path.GetFullPath(path)).AbsoluteUri;
        }
        else if (!string.IsNullOrEmpty(resourceString))
        {
            // シーン JSON は中のメッシュを絶対パスで指すので、文字列で渡されても
            // そのまま成立する。spawn 側の resource_string と違って対応できる。
            json = resourceString;
            worldName = "inline";
        }
        else
        {
            response.result.result = LoadWorldResponse.NO_RESOURCE;
            response.result.error_message = "Both uri and resource_string are empty";
            return response;
        }

        // 現在のシーンを消す前に構文を確かめる。読めないものを渡されたときに
        // 景観だけ消えて何も残らない、という壊れ方を避ける。
        if (!ObjectSpawner.TryParseSceneJson(json, out SavedSceneData sceneData, out string parseError))
        {
            response.result.result = LoadWorldResponse.RESOURCE_PARSE_ERROR;
            response.result.error_message = parseError;
            return response;
        }
        if (m_ObjectSpawner == null)
        {
            response.result.result = ResultMsg.RESULT_OPERATION_FAILED;
            response.result.error_message = "No ObjectSpawner in the scene to load the world into";
            return response;
        }

        CancelStepping();

        // LoadWorld.srv は「今のワールドを降ろし、スポーン済みのものを全部消して、
        // 停止状態で終わる」と決めている。
        DespawnAllEntities();
        m_ObjectSpawner.ClearSpawnedObjects();
        Clock.ResetTime();

        SceneLoadReport report = m_ObjectSpawner.LoadSceneData(sceneData);

        // 名前・説明・タグはワールドファイルに書かれていればそれを使う。
        // 一覧 (GetAvailableWorlds) と同じ内容が GetCurrentWorld からも見えるように
        // 組み立ては DescribeWorld に寄せてある。
        m_CurrentWorld = DescribeWorld(sceneData, worldName);
        m_CurrentWorld.world_resource = new ResourceMsg { uri = resolvedUri, resource_string = "" };
        m_SimulationState = SimulationStateMsg.STATE_STOPPED;
        Time.timeScale = 0f;
        if (playStopImage != null)
        {
            playStopImage.sprite = playIcon;
        }

        response.world = m_CurrentWorld;
        response.result.error_message = string.Join("; ", report.messages);
        if (report.missingAssets && !request.ignore_missing_or_unsupported_assets)
        {
            response.result.result = LoadWorldResponse.MISSING_ASSETS;
        }
        else if (report.unsupportedElements && request.fail_on_unsupported_element)
        {
            response.result.result = LoadWorldResponse.UNSUPPORTED_ELEMENTS;
        }
        else
        {
            response.result.result = ResultMsg.RESULT_OK;
        }
        return response;
    }

    /// <summary>unload_world サービス。</summary>
    private UnloadWorldResponse UnloadWorld(UnloadWorldRequest request)
    {
        var response = new UnloadWorldResponse();
        if (!IsWorldLoaded)
        {
            response.result.result = UnloadWorldResponse.NO_WORLD_LOADED;
            response.result.error_message = "No world is loaded";
            return response;
        }

        CancelStepping();
        DespawnAllEntities();
        if (m_ObjectSpawner != null)
        {
            m_ObjectSpawner.ClearSpawnedObjects();
        }

        m_CurrentWorld = null;
        m_SimulationState = SimulationStateMsg.STATE_NO_WORLD;
        Time.timeScale = 0f;
        if (playStopImage != null)
        {
            playStopImage.sprite = playIcon;
        }

        response.result.result = ResultMsg.RESULT_OK;
        return response;
    }

    /// <summary>get_current_world サービス。</summary>
    private GetCurrentWorldResponse GetCurrentWorld(GetCurrentWorldRequest request)
    {
        var response = new GetCurrentWorldResponse();
        if (!IsWorldLoaded)
        {
            response.result.result = GetCurrentWorldResponse.NO_WORLD_LOADED;
            response.result.error_message = "No world is loaded";
            response.world = new WorldResourceMsg();
            return response;
        }

        response.result.result = ResultMsg.RESULT_OK;
        response.world = m_CurrentWorld;
        return response;
    }

    /// <summary>get_available_worlds サービス。設定ファイルの world_paths を走査する。</summary>
    private GetAvailableWorldsResponse GetAvailableWorlds(GetAvailableWorldsRequest request)
    {
        var response = new GetAvailableWorldsResponse();
        response.worlds = Array.Empty<WorldResourceMsg>();

        // タグは各ワールドの JSON に書かれている (SavedSceneData.tags)。
        // 未知の filter_mode を黙って ANY として扱うと、要求と違う絞り込み結果を
        // 正常応答として返してしまうので、ここで弾く。
        if (!IsKnownTagFilterMode(request.filter))
        {
            response.result.result = ResultMsg.RESULT_OPERATION_FAILED;
            response.result.error_message =
                $"Unknown tags filter_mode {request.filter.filter_mode}; expected FILTER_MODE_ANY(0) or FILTER_MODE_ALL(1)";
            return response;
        }

        var problems = new List<string>();
        var roots = new List<string>();
        foreach (string path in SimulationResources.WorldPaths)
        {
            AddSearchRoot(path, roots, problems, "world_paths");
        }
        if (request.additional_sources != null)
        {
            foreach (string source in request.additional_sources)
            {
                AddSearchRoot(source, roots, problems, "additional_sources");
            }
        }

        var worlds = new List<WorldResourceMsg>();
        var seen = new HashSet<string>();
        var budget = new ScanBudget();
        foreach (string root in roots)
        {
            foreach (string file in EnumerateFiles(root, problems, budget))
            {
                if (!string.Equals(Path.GetExtension(file), ".json", StringComparison.OrdinalIgnoreCase))
                {
                    continue;
                }
                if (!seen.Add(file))
                {
                    continue;
                }

                // 設定ファイルなど無関係な JSON が混ざるので、シーンとして
                // 読めるものだけを候補にする。
                SavedSceneData sceneData;
                try
                {
                    if (!ObjectSpawner.TryParseSceneJson(File.ReadAllText(file), out sceneData, out _))
                    {
                        continue;
                    }
                }
                catch (Exception e)
                {
                    problems.Add($"cannot read '{file}': {e.Message}");
                    continue;
                }

                WorldResourceMsg world = DescribeWorld(sceneData, Path.GetFileNameWithoutExtension(file));
                world.world_resource = new ResourceMsg { uri = ToFileUri(file), resource_string = "" };
                if (!MatchesTags(world.tags, request.filter))
                {
                    continue;
                }
                worlds.Add(world);
            }
        }

        if (budget.Exhausted)
        {
            problems.Add($"stopped after scanning {k_MaxScannedFiles} files; narrow world_paths");
        }
        if (!string.IsNullOrEmpty(SimulationResources.LoadError))
        {
            problems.Add(SimulationResources.LoadError);
        }

        response.worlds = worlds.ToArray();
        response.result.error_message = string.Join("; ", problems);
        if (problems.Count > 0 && !request.continue_on_error)
        {
            response.result.result = GetAvailableWorldsResponse.DEFAULT_SOURCES_FAILED;
        }
        else
        {
            response.result.result = ResultMsg.RESULT_OK;
        }
        return response;
    }

    // ====================================================================
    // step_simulation
    // ====================================================================

    /// <summary>
    /// step_simulation サービス。一時停止状態から指定ステップだけ進めて、また止める。
    /// </summary>
    /// <remarks>
    /// 進み終わるまで応答を返さない決まりなので、非同期のサービス実装
    /// (ROSConnection の Func&lt;TRequest, Task&lt;TResponse&gt;&gt; 版) を使う。
    /// サービスは Update から呼ばれるので、await している間も Unity は回り続ける。
    ///
    /// Physics.Simulate() で回さないのは、それだと FixedUpdate が呼ばれず、
    /// ServoJointModel や JointStateSub といった制御側が動かないまま物理だけ
    /// 進んでしまうため。timeScale を戻して普通に 1 ステップずつ回す。
    /// </remarks>
    private async Task<StepSimulationResponse> StepSimulation(StepSimulationRequest request)
    {
        var response = new StepSimulationResponse();

        if (!IsWorldLoaded)
        {
            response.result.result = ResultMsg.RESULT_INCORRECT_STATE;
            response.result.error_message = "No world is loaded";
            return response;
        }
        if (m_SimulationState != SimulationStateMsg.STATE_PAUSED)
        {
            // StepSimulation.srv が「一時停止していなければ RESULT_OPERATION_FAILED」と
            // 明示している。
            response.result.result = ResultMsg.RESULT_OPERATION_FAILED;
            response.result.error_message =
                $"Simulation must be paused to step; it is in state {m_SimulationState}";
            return response;
        }
        if (m_Stepping)
        {
            response.result.result = ResultMsg.RESULT_OPERATION_FAILED;
            response.result.error_message = "Another step_simulation call is still running";
            return response;
        }
        if (request.steps > k_MaxStepsPerCall)
        {
            response.result.result = ResultMsg.RESULT_OPERATION_FAILED;
            response.result.error_message =
                $"steps={request.steps} exceeds the per-call limit of {k_MaxStepsPerCall}";
            return response;
        }
        if (request.steps == 0)
        {
            response.result.result = ResultMsg.RESULT_OK;
            return response;
        }

        bool completed = await RunSteps(request.steps, null);

        if (!completed)
        {
            response.result.result = ResultMsg.RESULT_OPERATION_FAILED;
            response.result.error_message = "Stepping was interrupted by another state change";
            return response;
        }
        response.result.result = ResultMsg.RESULT_OK;
        return response;
    }

    /// <summary>
    /// simulate_steps アクション。step_simulation と同じことを、1 ステップごとの
    /// 経過報告と途中キャンセルつきで行う。
    /// </summary>
    /// <remarks>
    /// 進行状態 (m_Stepping / m_StepCoroutine) はサービス版と共有している。
    /// 別々に持つと、サービスとアクションが同時に timeScale を奪い合って
    /// どちらのステップ数も合わなくなる。
    /// </remarks>
    private async Task<SimulateStepsResult> SimulateSteps(SimulateStepsGoal goal, ActionGoalHandle handle)
    {
        var result = new SimulateStepsResult();

        if (!IsWorldLoaded)
        {
            result.result.result = ResultMsg.RESULT_INCORRECT_STATE;
            result.result.error_message = "No world is loaded";
            handle.Abort();
            return result;
        }
        if (m_SimulationState != SimulationStateMsg.STATE_PAUSED)
        {
            // SimulateSteps.action も「一時停止していなければ OPERATION_FAILED」と
            // 明示している。
            result.result.result = ResultMsg.RESULT_OPERATION_FAILED;
            result.result.error_message =
                $"Simulation must be paused to step; it is in state {m_SimulationState}";
            handle.Abort();
            return result;
        }
        if (m_Stepping)
        {
            result.result.result = ResultMsg.RESULT_OPERATION_FAILED;
            result.result.error_message = "Another stepping request is still running";
            handle.Abort();
            return result;
        }
        if (goal.steps > k_MaxStepsPerCall)
        {
            result.result.result = ResultMsg.RESULT_OPERATION_FAILED;
            result.result.error_message =
                $"steps={goal.steps} exceeds the per-call limit of {k_MaxStepsPerCall}";
            handle.Abort();
            return result;
        }
        if (goal.steps == 0)
        {
            result.result.result = ResultMsg.RESULT_OK;
            return result;
        }

        bool completed = await RunSteps(goal.steps, handle);

        if (!completed)
        {
            result.result.result = ResultMsg.RESULT_OPERATION_FAILED;
            result.result.error_message = "Stepping was interrupted by another state change";
            handle.Abort();
            return result;
        }
        // 途中でキャンセルされた場合、進んだぶんは有効なので結果自体は OK にする。
        // 「最後までは進まなかった」ことは goal の status (CANCELED) が伝える。
        result.result.result = ResultMsg.RESULT_OK;
        return result;
    }

    /// <summary>
    /// steps 回だけ物理を進める。handle が非 null ならステップごとに feedback を出し、
    /// キャンセル要求で打ち切る。完走・キャンセルなら true、割り込まれたら false。
    /// </summary>
    private Task<bool> RunSteps(ulong steps, ActionGoalHandle handle)
    {
        m_Stepping = true;
        m_StepCompletion = new TaskCompletionSource<bool>();
        Task<bool> pending = m_StepCompletion.Task;
        m_StepCoroutine = StartCoroutine(StepRoutine(steps, handle, m_StepCompletion));
        return pending;
    }

    private IEnumerator StepRoutine(ulong steps, ActionGoalHandle handle, TaskCompletionSource<bool> completion)
    {
        Time.timeScale = 1f;
        for (ulong i = 0; i < steps; i++)
        {
            // WaitForFixedUpdate はその回の物理ステップが終わってから再開するので、
            // これを steps 回まわすとちょうど steps ステップ進んだところで止まる。
            yield return new WaitForFixedUpdate();

            if (handle != null)
            {
                handle.PublishFeedback(new SimulateStepsFeedback
                {
                    completed_steps = i + 1,
                    remaining_steps = steps - (i + 1)
                });
                if (handle.IsCancelRequested)
                {
                    break;
                }
            }
        }
        Time.timeScale = 0f;
        m_Stepping = false;
        m_StepCompletion = null;
        m_StepCoroutine = null;
        completion.TrySetResult(true);
    }

    /// <summary>
    /// 進行中の step_simulation を打ち切る。状態を変えるサービスとボタンから呼ぶ。
    /// </summary>
    /// <remarks>
    /// フラグを下ろすだけでなくコルーチン自体を止める。フラグ任せにすると、
    /// 割り込んだ側が timeScale を 0 にした場合に古いコルーチンが
    /// WaitForFixedUpdate で止まったまま残り、次の step_simulation で再生が
    /// 始まった瞬間に復活して、新しい要求のステップ数を一緒に食ってしまう。
    /// </remarks>
    private void CancelStepping()
    {
        if (!m_Stepping)
        {
            return;
        }
        m_Stepping = false;
        if (m_StepCoroutine != null)
        {
            StopCoroutine(m_StepCoroutine);
            m_StepCoroutine = null;
        }
        TaskCompletionSource<bool> completion = m_StepCompletion;
        m_StepCompletion = null;
        completion?.TrySetResult(false);
    }
}
