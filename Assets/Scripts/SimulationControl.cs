using System;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Xml;
using System.Threading.Tasks;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;
using Unity.Robotics.UrdfImporter;
using Unity.Robotics.UrdfImporter.Control;
using UnitySensors.Sensor.Camera;
using UnitySensors.Sensor.LiDAR;
using UnitySensors.Sensor.IMU;
using UnitySensors.DataType.LiDAR;
using UnitySensors.ROS.Publisher.Camera;
using UnitySensors.ROS.Publisher.Sensor;
using UnitySensors.ROS.Serializer.Sensor;
using UnitySensors.ROS.Serializer.Std;
using UnitySensors.ROS.Serializer.PointCloud;
using UnitySensors.ROS.Serializer.Image;

using RosMessageTypes.SimulationInterfaces;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

using UnityMeshImporter;
using NaughtyWaterBuoyancy;
using Hydrodynamics;
using Aerodynamics;

public class FileLogger
{
    private static string logFilePath = "debug_log.txt";

    public static void Log(string message)
    {
        File.AppendAllText(logFilePath, message + "\n");
    }
}

public partial class SimulationControl : MonoBehaviour
{
    [SerializeField]
    string m_SetSimulationStateServiceName = "set_simulation_state";
    [SerializeField]
    string m_GetSimulationStateServiceName = "get_simulation_state";
    [SerializeField]
    string m_ResetSimulationServiceName = "reset_simulation";
    [SerializeField]
    string m_StepSimulationServiceName = "step_simulation";
    [SerializeField]
    string m_SpawnEntityServiceName = "spawn_entity";
    [SerializeField]
    string m_SpawnEntitiesServiceName = "spawn_entities";
    [SerializeField]
    string m_GetSimulatorFeaturesServiceName = "get_simulator_features";
    [SerializeField]
    string m_DeleteEntityServiceName = "delete_entity";
    [SerializeField]
    string m_GetEntitiesServiceName = "get_entities";
    [SerializeField]
    string m_GetEntitiesStatesServiceName = "get_entities_states";
    [SerializeField]
    string m_GetEntityStateServiceName = "get_entity_state";
    [SerializeField]
    string m_SetEntityStateServiceName = "set_entity_state";
    [SerializeField]
    string m_GetEntityInfoServiceName = "get_entity_info";
    [SerializeField]
    string m_SetEntityInfoServiceName = "set_entity_info";
    [SerializeField]
    string m_GetEntityBoundsServiceName = "get_entity_bounds";
    [SerializeField]
    string m_GetSpawnablesServiceName = "get_spawnables";
    [SerializeField]
    string m_GetNamedPosesServiceName = "get_named_poses";
    [SerializeField]
    string m_GetNamedPoseBoundsServiceName = "get_named_pose_bounds";
    [SerializeField]
    string m_LoadWorldServiceName = "load_world";
    [SerializeField]
    string m_UnloadWorldServiceName = "unload_world";
    [SerializeField]
    string m_GetCurrentWorldServiceName = "get_current_world";
    [SerializeField]
    string m_GetAvailableWorldsServiceName = "get_available_worlds";

    [Header("再生/停止 Button の Image コンポーネント")]
    public Image playStopImage;

    [Header("差し替え用スプライト")]
    public Sprite playIcon;
    public Sprite stopIcon;

    private byte m_SimulationState = new byte();
    private List<GameObject> m_EntityList = new List<GameObject>();
    // Entityの初期位置姿勢を保持する辞書
    private Dictionary<string, Vector3> m_EntityInitialPose = new Dictionary<string, Vector3>();
    private Dictionary<string, Quaternion> m_EntityInitialRotation = new Dictionary<string, Quaternion>();

    // Entity ごとに、そのスポーンで publisher 登録したトピック名。
    // デスポーン時に登録を解除して、消えたロボットのトピックが
    // ros2 topic list に残り続けないようにする。
    private Dictionary<string, List<string>> m_EntityPublishedTopics = new Dictionary<string, List<string>>();
    // トピックごとの参照数。/tf や /ground_truth は複数ロボットが同じ名前へ
    // publish するので、最後の 1 台が消えるまで解除してはいけない。
    private Dictionary<string, int> m_PublishedTopicRefCount = new Dictionary<string, int>();

    void Start()
    {
        ROSConnection.GetOrCreateInstance().ImplementService<SetSimulationStateRequest, SetSimulationStateResponse>(
            m_SetSimulationStateServiceName,
            SetSimulationState);
        ROSConnection.GetOrCreateInstance().ImplementService<GetSimulationStateRequest, GetSimulationStateResponse>(
            m_GetSimulationStateServiceName,
            GetSimulationState);
        ROSConnection.GetOrCreateInstance().ImplementService<ResetSimulationRequest, ResetSimulationResponse>(
            m_ResetSimulationServiceName,
            ResetSimulation);
        ROSConnection.GetOrCreateInstance().ImplementService<StepSimulationRequest, StepSimulationResponse>(
            m_StepSimulationServiceName,
            StepSimulation);
        ROSConnection.GetOrCreateInstance().ImplementService<SpawnEntityRequest, SpawnEntityResponse>(
            m_SpawnEntityServiceName,
            SpawnEntity);
        ROSConnection.GetOrCreateInstance().ImplementService<SpawnEntitiesRequest, SpawnEntitiesResponse>(
            m_SpawnEntitiesServiceName,
            SpawnEntities);
        ROSConnection.GetOrCreateInstance().ImplementService<GetSimulatorFeaturesRequest, GetSimulatorFeaturesResponse>(
            m_GetSimulatorFeaturesServiceName,
            GetSimulatorFeatures);

        // エンティティ系 (SimulationEntityServices.cs)
        ROSConnection.GetOrCreateInstance().ImplementService<DeleteEntityRequest, DeleteEntityResponse>(
            m_DeleteEntityServiceName,
            DeleteEntity);
        ROSConnection.GetOrCreateInstance().ImplementService<GetEntitiesRequest, GetEntitiesResponse>(
            m_GetEntitiesServiceName,
            GetEntities);
        ROSConnection.GetOrCreateInstance().ImplementService<GetEntitiesStatesRequest, GetEntitiesStatesResponse>(
            m_GetEntitiesStatesServiceName,
            GetEntitiesStates);
        ROSConnection.GetOrCreateInstance().ImplementService<GetEntityStateRequest, GetEntityStateResponse>(
            m_GetEntityStateServiceName,
            GetEntityState);
        ROSConnection.GetOrCreateInstance().ImplementService<SetEntityStateRequest, SetEntityStateResponse>(
            m_SetEntityStateServiceName,
            SetEntityState);
        ROSConnection.GetOrCreateInstance().ImplementService<GetEntityInfoRequest, GetEntityInfoResponse>(
            m_GetEntityInfoServiceName,
            GetEntityInfo);
        ROSConnection.GetOrCreateInstance().ImplementService<SetEntityInfoRequest, SetEntityInfoResponse>(
            m_SetEntityInfoServiceName,
            SetEntityInfo);
        ROSConnection.GetOrCreateInstance().ImplementService<GetEntityBoundsRequest, GetEntityBoundsResponse>(
            m_GetEntityBoundsServiceName,
            GetEntityBounds);
        ROSConnection.GetOrCreateInstance().ImplementService<GetSpawnablesRequest, GetSpawnablesResponse>(
            m_GetSpawnablesServiceName,
            GetSpawnables);
        ROSConnection.GetOrCreateInstance().ImplementService<GetNamedPosesRequest, GetNamedPosesResponse>(
            m_GetNamedPosesServiceName,
            GetNamedPoses);
        ROSConnection.GetOrCreateInstance().ImplementService<GetNamedPoseBoundsRequest, GetNamedPoseBoundsResponse>(
            m_GetNamedPoseBoundsServiceName,
            GetNamedPoseBounds);

        // world 系 (SimulationWorldServices.cs)
        ROSConnection.GetOrCreateInstance().ImplementService<LoadWorldRequest, LoadWorldResponse>(
            m_LoadWorldServiceName,
            LoadWorld);
        ROSConnection.GetOrCreateInstance().ImplementService<UnloadWorldRequest, UnloadWorldResponse>(
            m_UnloadWorldServiceName,
            UnloadWorld);
        ROSConnection.GetOrCreateInstance().ImplementService<GetCurrentWorldRequest, GetCurrentWorldResponse>(
            m_GetCurrentWorldServiceName,
            GetCurrentWorld);
        ROSConnection.GetOrCreateInstance().ImplementService<GetAvailableWorldsRequest, GetAvailableWorldsResponse>(
            m_GetAvailableWorldsServiceName,
            GetAvailableWorlds);

        // 起動直後の組み込みシーンをロード済みワールドとして登録する。
        // これをやらないと GetCurrentWorld が「ワールド無し」になり、
        // 起動直後が STATE_STOPPED という従来の挙動とも食い違う。
        InitializeWorld();
        SimulationResources.Reload();

        m_SimulationState = SimulationStateMsg.STATE_STOPPED;
        Time.timeScale = 0f;
    }

    private SetSimulationStateResponse SetSimulationState(SetSimulationStateRequest request)
    {
        var response = new SetSimulationStateResponse();
        // Result.msg の規約では成功は RESULT_OK (1)。既定値の 0 は
        // RESULT_FEATURE_UNSUPPORTED なので、明示的に上書きする。
        response.result.result = ResultMsg.RESULT_OK;

        if (request.state.state == m_SimulationState)
        {
            response.result.result = SetSimulationStateResponse.ALREADY_IN_TARGET_STATE;
            response.result.error_message = "Already in requested state";
            return response;
        }

        // SimulationState.msg では STATE_NO_WORLD は「シミュレーションが不活性で
        // 開始も停止も一時停止もできない」状態。LoadWorld を経ずに動かそうとする
        // 要求はここで弾く。QUITTING だけはワールドの有無に関係なく通す。
        if (!IsWorldLoaded && request.state.state != SimulationStateMsg.STATE_QUITTING)
        {
            response.result.result = SetSimulationStateResponse.INCORRECT_TRANSITION;
            response.result.error_message = "No world is loaded; call load_world first";
            return response;
        }
        if (request.state.state == SimulationStateMsg.STATE_NO_WORLD ||
            request.state.state == SimulationStateMsg.STATE_LOADING_WORLD)
        {
            // この 2 つはワールドの読み書きの結果として入る状態で、外から指定して
            // 遷移するものではない。
            response.result.result = SetSimulationStateResponse.INCORRECT_TRANSITION;
            response.result.error_message =
                "STATE_NO_WORLD / STATE_LOADING_WORLD are set by the world interfaces, not by this service";
            return response;
        }

        // 進行中の step_simulation は、こちらが timeScale を決め直す前に降ろす。
        CancelStepping();

        // TODO: implement the logic to set SetSimulationStateResponse.STATE_TRANSITION_ERROR

        if (request.state.state == SimulationStateMsg.STATE_STOPPED)
        {
            m_SimulationState = request.state.state;
            Time.timeScale = 0f;
            playStopImage.sprite = playIcon;

            DespawnAllEntities();
        }
        else if (request.state.state == SimulationStateMsg.STATE_PLAYING)
        {
            m_SimulationState = request.state.state;
            Time.timeScale = 1f;
            playStopImage.sprite = stopIcon;
        }
        else if (request.state.state == SimulationStateMsg.STATE_PAUSED)
        {
            m_SimulationState = request.state.state;
            Time.timeScale = 0f;
            playStopImage.sprite = playIcon;
        }
        else if (request.state.state == SimulationStateMsg.STATE_QUITTING)
        {
            m_SimulationState = request.state.state;
            Application.Quit();
        }
        else
        {
            response.result.result = SetSimulationStateResponse.INCORRECT_TRANSITION;
            response.result.error_message = "Invalid simulation state requested";
            return response;
        }
        return response;
    }

    public void StartStopSimulation()
    {
        // 画面のボタンからの操作でも、進行中の step_simulation は先に降ろす。
        // 残しておくと、コルーチンが後から timeScale を 0 に戻して再生が止まる。
        CancelStepping();

        if (m_SimulationState == SimulationStateMsg.STATE_PAUSED || m_SimulationState == SimulationStateMsg.STATE_STOPPED)
        {
            m_SimulationState = SimulationStateMsg.STATE_PLAYING;
            Time.timeScale = 1f;
            playStopImage.sprite = stopIcon;
        }
        else
        {
            m_SimulationState = SimulationStateMsg.STATE_PAUSED;
            Time.timeScale = 0f;
            ResetAllEntitiesState();
            playStopImage.sprite = playIcon;
        }
    }
    public void PauseSimulation()
    {
        CancelStepping();

        if (m_SimulationState == SimulationStateMsg.STATE_PLAYING)
        {
            m_SimulationState = SimulationStateMsg.STATE_PAUSED;
            Time.timeScale = 0f;
            playStopImage.sprite = playIcon;
        }
    }

    private GetSimulationStateResponse GetSimulationState(GetSimulationStateRequest request)
    {
        var response = new GetSimulationStateResponse();
        response.result.result = ResultMsg.RESULT_OK;
        response.state.state = m_SimulationState;
        return response;
    }

    private ResetSimulationResponse ResetSimulation(ResetSimulationRequest request)
    {
        var response = new ResetSimulationResponse();
        response.result.result = ResultMsg.RESULT_OK;

        if (!IsWorldLoaded)
        {
            response.result.result = ResultMsg.RESULT_INCORRECT_STATE;
            response.result.error_message = "No world is loaded; call load_world first";
            return response;
        }

        CancelStepping();
        m_SimulationState = SimulationStateMsg.STATE_STOPPED;
        Time.timeScale = 0f;
        if (playStopImage != null)
        {
            playStopImage.sprite = playIcon;
        }

        if (request.scope == ResetSimulationRequest.SCOPE_DEFAULT)
        {
            request.scope = ResetSimulationRequest.SCOPE_ALL;
        }

        if ((request.scope & ResetSimulationRequest.SCOPE_TIME) == ResetSimulationRequest.SCOPE_TIME)
        {
            Clock.ResetTime();
        }
        if ((request.scope & ResetSimulationRequest.SCOPE_STATE) == ResetSimulationRequest.SCOPE_STATE)
        {
            // SCOPE_SPAWNED も一緒に立っている場合、どうせ全部消えるので
            // 状態を戻す意味がない。
            if ((request.scope & ResetSimulationRequest.SCOPE_SPAWNED) != ResetSimulationRequest.SCOPE_SPAWNED)
            {
                ResetAllEntitiesState();
            }
        }
        if ((request.scope & ResetSimulationRequest.SCOPE_SPAWNED) == ResetSimulationRequest.SCOPE_SPAWNED)
        {
            DespawnAllEntities();
        }

        return response;
    }

    /// <summary>
    /// get_simulator_features サービス。このシミュレータが実際に実装している機能だけを申告する。
    /// </summary>
    /// <remarks>
    /// simulation_interfaces は「その機能が使えるかは GetSimulatorFeatures で確認せよ」という
    /// 建て付けなので、ここは実装状況と必ず揃えること。未実装のものを載せると、
    /// クライアントは使えると判断して呼びに行ってしまう。
    /// </remarks>
    private GetSimulatorFeaturesResponse GetSimulatorFeatures(GetSimulatorFeaturesRequest request)
    {
        var response = new GetSimulatorFeaturesResponse();
        response.features.features = new ushort[]
        {
            SimulatorFeaturesMsg.SPAWNING,                  // spawn_entity
            SimulatorFeaturesMsg.SPAWNING_ENTITIES,         // spawn_entities
            SimulatorFeaturesMsg.DELETING,                  // delete_entity
            SimulatorFeaturesMsg.SPAWNABLES,                // get_spawnables
            SimulatorFeaturesMsg.NAMED_POSES,               // get_named_poses
            SimulatorFeaturesMsg.POSE_BOUNDS,               // get_named_pose_bounds
            SimulatorFeaturesMsg.ENTITY_STATE_GETTING,      // get_entity_state / get_entities_states
            SimulatorFeaturesMsg.ENTITY_STATE_SETTING,      // set_entity_state
            SimulatorFeaturesMsg.ENTITY_INFO_GETTING,       // get_entity_info
            SimulatorFeaturesMsg.ENTITY_INFO_SETTING,       // set_entity_info
            SimulatorFeaturesMsg.ENTITY_CATEGORIES,         // EntityFilters.categories
            SimulatorFeaturesMsg.ENTITY_TAGS,               // EntityFilters.tags
            SimulatorFeaturesMsg.ENTITY_BOUNDS,             // get_entity_bounds
            SimulatorFeaturesMsg.ENTITY_BOUNDS_BOX,         // EntityFilters.bounds の TYPE_BOX
            SimulatorFeaturesMsg.SIMULATION_RESET,          // reset_simulation
            SimulatorFeaturesMsg.SIMULATION_RESET_TIME,     // SCOPE_TIME
            SimulatorFeaturesMsg.SIMULATION_RESET_STATE,    // SCOPE_STATE
            SimulatorFeaturesMsg.SIMULATION_RESET_SPAWNED,  // SCOPE_SPAWNED
            SimulatorFeaturesMsg.SIMULATION_STATE_GETTING,  // get_simulation_state
            SimulatorFeaturesMsg.SIMULATION_STATE_SETTING,  // set_simulation_state
            SimulatorFeaturesMsg.SIMULATION_STATE_PAUSE,    // STATE_PAUSED への遷移
            SimulatorFeaturesMsg.STEP_SIMULATION_SINGLE,    // step_simulation (steps = 1)
            SimulatorFeaturesMsg.STEP_SIMULATION_MULTIPLE,  // step_simulation (steps > 1)
            SimulatorFeaturesMsg.WORLD_LOADING,             // load_world
            SimulatorFeaturesMsg.WORLD_RESOURCE_STRING,     // load_world の resource_string
            SimulatorFeaturesMsg.WORLD_TAGS,                // ワールドのタグと絞り込み
            SimulatorFeaturesMsg.WORLD_UNLOADING,           // unload_world
            SimulatorFeaturesMsg.WORLD_INFO_GETTING,        // get_current_world
            SimulatorFeaturesMsg.AVAILABLE_WORLDS,          // get_available_worlds
        };
        // 申告していないもの:
        //  SPAWNING_RESOURCE_STRING  URDF の mesh 参照は URDF からの相対パスで解決するため、
        //                            文字列だけ受け取ってもアセットを見つけられない
        //  ENTITY_BOUNDS_CONVEX      凸包での絞り込みは未実装 (TYPE_BOX と TYPE_SPHERE のみ)
        //  STEP_SIMULATION_ACTION    ROS-TCP-Connector にアクションの口が無い
        response.features.spawn_formats = new string[] { "urdf" };
        response.features.custom_info =
            "Unity_ROS2_Robot_Simulator. Mesh files (obj/stl/dae) can also be spawned by uri. " +
            "Worlds are ObjectSpawner scene JSON files; spawnable, world and named-pose sources " +
            "come from simulation_resources.json.";
        return response;
    }

    /// <summary>
    /// spawn_entity サービス。simulation_interfaces 2.0.0 以降は SpawnEntities が推奨で
    /// こちらは deprecated 扱いだが、単体スポーンの入口として残っている。
    /// </summary>
    private SpawnEntityResponse SpawnEntity(SpawnEntityRequest request)
    {
        SpawnResultMsg result = SpawnEntityCore(
            request.name, request.allow_renaming, request.entity_resource,
            request.entity_namespace, request.initial_pose);

        // SpawnResult と SpawnEntity レスポンスは同じ追加コード (101-109) を共有する
        var response = new SpawnEntityResponse();
        response.result = result.result;
        response.entity_name = result.entity_name;
        return response;
    }

    /// <summary>
    /// spawn_entities サービス (simulation_interfaces 2.0.0 で追加)。
    /// </summary>
    /// <remarks>
    /// 1 件でも失敗したら result は ENTITIES_SPAWN_FAILED になり、個々の成否は
    /// results[i] に入る。途中で失敗しても残りの要求は続行する: 部分的に成功した
    /// シーンを呼び出し側が results から把握できるほうが、途中で止めて何が生成
    /// されたか分からなくなるより扱いやすい。
    /// </remarks>
    private SpawnEntitiesResponse SpawnEntities(SpawnEntitiesRequest request)
    {
        var response = new SpawnEntitiesResponse();
        int count = request.spawn_requests != null ? request.spawn_requests.Length : 0;
        response.results = new SpawnResultMsg[count];

        bool anyFailed = false;
        for (int i = 0; i < count; i++)
        {
            SpawnEntityMsg spawnRequest = request.spawn_requests[i];
            SpawnResultMsg result = SpawnEntityCore(
                spawnRequest.name, spawnRequest.allow_renaming, spawnRequest.entity_resource,
                spawnRequest.entity_namespace, spawnRequest.initial_pose);
            response.results[i] = result;
            if (result.result.result != ResultMsg.RESULT_OK)
            {
                anyFailed = true;
            }
        }

        if (anyFailed)
        {
            response.result.result = SpawnEntitiesResponse.ENTITIES_SPAWN_FAILED;
            response.result.error_message = "At least one spawn request failed; see results";
        }
        else
        {
            response.result.result = ResultMsg.RESULT_OK;
        }
        return response;
    }

    /// <summary>
    /// 1 体分のスポーン処理。spawn_entity と spawn_entities の共通実装。
    /// </summary>
    private SpawnResultMsg SpawnEntityCore(
        string requestedName,
        bool allowRenaming,
        ResourceMsg entityResource,
        string entityNamespace,
        RosMessageTypes.Geometry.PoseStampedMsg initialPose)
    {
        // prepare a response
        SpawnResultMsg spawnEntityResponse = new SpawnResultMsg();
        spawnEntityResponse.result.result = ResultMsg.RESULT_OK;
        spawnEntityResponse.entity_name = requestedName;

        // process the service request
        Debug.Log("Received request for object: " + requestedName);

        if (!IsWorldLoaded)
        {
            // ワールドが降ろされている間は置き場所が無い。
            Debug.LogError("Cannot spawn while no world is loaded");
            spawnEntityResponse.result.result = ResultMsg.RESULT_INCORRECT_STATE;
            spawnEntityResponse.result.error_message = "No world is loaded; call load_world first";
            return spawnEntityResponse;
        }

        // 2.0.0 で uri / resource_string は Resource メッセージへまとめられた。
        string resourceUri = entityResource != null ? entityResource.uri : null;
        string resourceString = entityResource != null ? entityResource.resource_string : null;

        if (string.IsNullOrEmpty(resourceUri))
        {
            if (!string.IsNullOrEmpty(resourceString))
            {
                // resource_string からの生成 (SPAWNING_RESOURCE_STRING) は未対応。
                // URDF の mesh 参照は URDF ファイルからの相対パスで解決されるため、
                // 文字列だけ受け取ってもアセットを見つけられない。
                Debug.LogError("Spawning from resource_string is not supported");
                spawnEntityResponse.result.result = SpawnResultMsg.UNSUPPORTED_FORMAT;
                spawnEntityResponse.result.error_message =
                    "Spawning from resource_string is not supported; provide a file uri instead";
                return spawnEntityResponse;
            }
            Debug.LogError("Neither uri nor resource_string was provided");
            spawnEntityResponse.result.result = SpawnResultMsg.NO_RESOURCE;
            spawnEntityResponse.result.error_message = "Both uri and resource_string are empty";
            return spawnEntityResponse;
        }

        string filePath;
        Uri uri;
        if (!Uri.TryCreate(resourceUri, UriKind.Absolute, out uri) || !uri.IsFile)
        {
            Debug.LogError("Invalid URI: " + resourceUri);
            spawnEntityResponse.result.result = SpawnResultMsg.RESOURCE_PARSE_ERROR;
            spawnEntityResponse.result.error_message = "Invalid URI: " + resourceUri;
            return spawnEntityResponse;
        }
        filePath = uri.LocalPath;

        if (!File.Exists(filePath))
        {
            Debug.LogError("Resource file not found: " + filePath);
            spawnEntityResponse.result.result = SpawnResultMsg.MISSING_ASSETS;
            spawnEntityResponse.result.error_message = "Resource file not found: " + filePath;
            return spawnEntityResponse;
        }

        double robot_x = initialPose.pose.position.x;
        double robot_y = initialPose.pose.position.y;
        double robot_z = initialPose.pose.position.z;
        double q_x = initialPose.pose.orientation.x;
        double q_y = initialPose.pose.orientation.y;
        double q_z = initialPose.pose.orientation.z;
        double q_w = initialPose.pose.orientation.w;

        Debug.Log("Received path: " + filePath);

        if (!filePath.EndsWith(".urdf"))
        {
            var ob = MeshImporter.Load(filePath);
            if (ob == null)
            {
                Debug.LogError("Failed to load object from File.");
                spawnEntityResponse.result.result = SpawnResultMsg.RESOURCE_PARSE_ERROR;
                spawnEntityResponse.result.error_message = "Failed to load object from File.";
                return spawnEntityResponse;
            }
            string meshName;
            if (!TryResolveEntityName(requestedName, ob.name, allowRenaming, out meshName))
            {
                Debug.LogError($"Entity name '{requestedName}' is already taken");
                GameObject.Destroy(ob);
                spawnEntityResponse.result.result = SpawnResultMsg.NAME_NOT_UNIQUE;
                spawnEntityResponse.result.error_message =
                    $"Entity name '{requestedName}' is already taken; set allow_renaming to spawn anyway";
                return spawnEntityResponse;
            }
            ob.name = meshName;
            spawnEntityResponse.entity_name = meshName;

            // オブジェクトの直下のすべての子オブジェクトを取得
            foreach (MeshCollider meshCollider in ob.GetComponentsInChildren<MeshCollider>())
            {
                meshCollider.sharedMesh = meshCollider.gameObject.GetComponent<MeshFilter>().mesh;
            }
            m_EntityList.Add(ob);
            // ロボットの位置・回転設定
            // Unityの座標系は左手系で真上がY軸、URDFの座標系は右手系で真上がZ軸
            // そのため、URDFのZ軸をUnityのY軸に変換する必要がある
            // URDFのX軸をUnityのZ軸に変換する必要がある
            Vector3 newMeshPosition = new Vector3(Convert.ToSingle(-robot_y), Convert.ToSingle(robot_z), Convert.ToSingle(robot_x));
            ob.transform.position = newMeshPosition;
            m_EntityInitialPose[ob.name] = newMeshPosition;
            Quaternion newMeshRotation = ConvertQuaternion(
                new Quaternion(
                    Convert.ToSingle(q_x),
                    Convert.ToSingle(q_y),
                    Convert.ToSingle(q_z),
                    Convert.ToSingle(q_w)
                )
            );
            ob.transform.rotation = newMeshRotation;
            m_EntityInitialRotation[ob.name] = newMeshRotation;
            // メッシュ単体は関節も ROS インターフェースも持たないので「物体」扱い。
            // 分類を変えたい場合は set_entity_info で上書きできる。
            RegisterEntityInfo(ob.name, EntityCategoryMsg.CATEGORY_OBJECT, "Spawned from " + resourceUri);

            return spawnEntityResponse;
        }

        ImportSettings settings = new ImportSettings();
        GameObject robotObject = UrdfRobotExtensions.CreateRuntime(filePath, settings);

        if (robotObject == null)
        {
            Debug.LogError("Failed to load robot from URDF.");
            spawnEntityResponse.result.result = SpawnResultMsg.RESOURCE_PARSE_ERROR;
            spawnEntityResponse.result.error_message = "Failed to load robot from URDF.";
            return spawnEntityResponse;
        }

        // 名前は topic 名 (/<name>/joint_states など) や m_EntityInitialPose の
        // キーになるので、配下の publisher を組み立てる前に確定させる。
        string resolvedName;
        if (!TryResolveEntityName(requestedName, robotObject.name, allowRenaming, out resolvedName))
        {
            Debug.LogError($"Entity name '{requestedName}' is already taken");
            GameObject.Destroy(robotObject);
            spawnEntityResponse.result.result = SpawnResultMsg.NAME_NOT_UNIQUE;
            spawnEntityResponse.result.error_message =
                $"Entity name '{requestedName}' is already taken; set allow_renaming to spawn anyway";
            return spawnEntityResponse;
        }
        robotObject.name = resolvedName;
        spawnEntityResponse.entity_name = resolvedName;

        m_EntityList.Add(robotObject);

        // ロボットの位置・回転設定
        // Unityの座標系は左手系で真上がY軸、URDFの座標系は右手系で真上がZ軸
        // そのため、URDFのZ軸をUnityのY軸に変換する必要がある
        // URDFのX軸をUnityのZ軸に変換する必要がある
        Vector3 newPosition = new Vector3(Convert.ToSingle(-robot_y), Convert.ToSingle(robot_z), Convert.ToSingle(robot_x));
        robotObject.transform.position = newPosition;
        m_EntityInitialPose[robotObject.name] = newPosition;
        Quaternion newRotation = ConvertQuaternion(
                new Quaternion(
                    Convert.ToSingle(q_x),
                    Convert.ToSingle(q_y),
                    Convert.ToSingle(q_z),
                    Convert.ToSingle(q_w)
                )
            );
        robotObject.transform.rotation = newRotation;
        m_EntityInitialRotation[robotObject.name] = newRotation;
        // URDF から作ったものは ros2_control 相当のトピックを持つので「ロボット」扱い。
        RegisterEntityInfo(robotObject.name, EntityCategoryMsg.CATEGORY_ROBOT, "Spawned from " + resourceUri);

        // 最初に見つかった UrdfLink に対してベースリンク設定と固定フラグを適用
        List<GameObject> childObjectsWithUrdfLink = GetChildObjectsWithComponent<UrdfLink>(robotObject);
        foreach (GameObject child in childObjectsWithUrdfLink)
        {
            UrdfLink link = child.GetComponent<UrdfLink>();
            link.IsBaseLink = true;

            ArticulationBody body = child.GetComponent<ArticulationBody>();
            if (body != null)
            {
                body.TeleportRoot(newPosition, newRotation);
                body.PublishTransform();
                if (link.name == "world")
                {
                    // world link の場合は immovable を true にする
                    body.immovable = true;
                }
                else
                {
                    // それ以外のリンクは immovable を false にする
                    body.immovable = false;
                }
            }
            break;
        }

        // スポーン直後の関節状態を明示的にゼロへリセットする。
        // ランタイム構築中や TeleportRoot によるルート回転 (spawn yaw) は
        // 関節座標にステップ入力として現れ、1 物理ステップで数十 rad/s の
        // 速度が注入されて多回転・リミット突破・固着の原因になるため、
        // 構築完了とテレポートの後に URDF ゼロ姿勢・速度ゼロへ戻す。
        ResetArticulationState(robotObject);

        // URDFファイルの解析
        XmlDocument xmlDoc = new XmlDocument();
        xmlDoc.Load(filePath);

        // JointState 用の Publisher/Subscriber の設定
        JointStatePub jointStatePub = robotObject.AddComponent<JointStatePub>();
        JointStateSub jointStateSub = robotObject.AddComponent<JointStateSub>();
        GroundTruthPub groundTruthPub = robotObject.AddComponent<GroundTruthPub>();
        List<GameObject> childObjectsWithArticulationBody = FindArticulationBodyObjectsInChildren(robotObject);
        List<ArticulationBody> articulationBodyList = new List<ArticulationBody>();
        List<string> jointNameList = new List<string>();
        foreach (GameObject child in childObjectsWithArticulationBody)
        {
            ArticulationBody body = child.GetComponent<ArticulationBody>();
            Debug.Log("Received joint type: " + body.jointType);
            if (body.jointType != ArticulationJointType.FixedJoint)
            {
                UrdfJoint urdfJoint = child.GetComponent<UrdfJoint>();
                articulationBodyList.Add(body);
                jointNameList.Add(urdfJoint.jointName);
                // Note: xDrive stiffness/damping is now set by URDF-Importer via <drive> element
            }
        }

        jointStatePub.articulationBodies = articulationBodyList.ToArray();
        jointStatePub.jointName = jointNameList.ToArray();
        jointStatePub.jointLength = articulationBodyList.Count;
        XmlNode jointStateParam = xmlDoc.SelectSingleNode("//robot/ros2_control/hardware/param[@name='joint_states_topic']");
        if (jointStateParam != null)
        {
            jointStatePub.topicName = jointStateParam.InnerText;
        }
        // URDF 指定が無ければ既定値のままなので、確定してから控える。
        // TrackPublishedTopic は名前空間を適用した名前を返すので必ず代入し直すこと
        // (代入を忘れると publisher だけ名前空間なしのまま登録され、解除対象の
        // 名前ともずれる)。
        jointStatePub.topicName =
            TrackPublishedTopic(robotObject.name, entityNamespace, jointStatePub.topicName);

        jointStateSub.articulationBodies = articulationBodyList.ToArray();
        jointStateSub.jointName = jointNameList.ToArray();
        jointStateSub.jointLength = articulationBodyList.Count;
        XmlNode jointCommandParam = xmlDoc.SelectSingleNode("//robot/ros2_control/hardware/param[@name='joint_commands_topic']");
        if (jointCommandParam != null)
        {
            jointStateSub.topicName = jointCommandParam.InnerText;
        }
        jointStateSub.topicName = ApplyNamespace(entityNamespace, jointStateSub.topicName);

        groundTruthPub.targetObject = childObjectsWithUrdfLink[0];
        XmlNode groundTruthParam = xmlDoc.SelectSingleNode("//robot/ros2_control/hardware/param[@name='ground_truth_topic']");
        if (groundTruthParam != null)
        {
            groundTruthPub.topicName = groundTruthParam.InnerText;
        }
        // ground_truth と tf は既定では全ロボット共通の名前になる。参照数で管理するので
        // ここでは素直に両方控えておけばよい。
        groundTruthPub.topicName =
            TrackPublishedTopic(robotObject.name, entityNamespace, groundTruthPub.topicName);
        groundTruthPub.tfTopicName =
            TrackPublishedTopic(robotObject.name, entityNamespace, groundTruthPub.tfTopicName);

        // Physics Material の生成（ランタイムでは AssetDatabase は使用不可のため new で生成）
        string directoryPath = Path.GetDirectoryName(filePath);
        int assetsIndex = directoryPath.IndexOf("Assets");
        if (assetsIndex >= 0)
        {
            directoryPath = directoryPath.Substring(assetsIndex);
        }
        XmlNode robotNode = xmlDoc.SelectSingleNode("/robot");
        List<PhysicsMaterial> physicsMaterialList = new List<PhysicsMaterial>();
        Dictionary<string, float> contactOffsetDict = new Dictionary<string, float>();
        if (robotNode != null)
        {
            XmlNodeList physicsMaterials = robotNode.SelectNodes("collision_material");
            if (physicsMaterials.Count == 0)
            {
                Debug.LogWarning("<physics_material> is deprecated. Use <collision_material> instead.");
                physicsMaterials = robotNode.SelectNodes("physics_material");
            }
            foreach (XmlNode physicsMaterial in physicsMaterials)
            {
                string materialName = physicsMaterial.Attributes["name"]?.Value;
                PhysicsMaterial newMaterial = new PhysicsMaterial(materialName);
                XmlNode frictionNode = physicsMaterial.SelectSingleNode("friction");
                if (frictionNode != null)
                {
                    newMaterial.staticFriction = TryParseFloat(frictionNode.Attributes["static"]?.Value);
                    newMaterial.dynamicFriction = TryParseFloat(frictionNode.Attributes["dynamic"]?.Value);
                    string combineMode = frictionNode.Attributes["combine"]?.Value?.ToLower();
                    switch (combineMode)
                    {
                        case "average":
                            newMaterial.frictionCombine = PhysicsMaterialCombine.Average;
                            break;
                        case "multiply":
                            newMaterial.frictionCombine = PhysicsMaterialCombine.Multiply;
                            break;
                        case "maximum":
                            newMaterial.frictionCombine = PhysicsMaterialCombine.Maximum;
                            break;
                        case "minimum":
                            newMaterial.frictionCombine = PhysicsMaterialCombine.Minimum;
                            break;
                        default:
                            newMaterial.frictionCombine = PhysicsMaterialCombine.Average;
                            break;
                    }
                }
                XmlNode contactOffsetNode = physicsMaterial.SelectSingleNode("contact_offset");
                if (contactOffsetNode != null)
                {
                    contactOffsetDict[materialName] = TryParseFloat(contactOffsetNode.Attributes["value"]?.Value);
                }
                physicsMaterialList.Add(newMaterial);
            }
        }

        // Physics Material の適用
        if (robotNode != null)
        {
            XmlNodeList links = robotNode.SelectNodes("link");
            foreach (XmlNode link in links)
            {
                XmlNode collisionNode = link.SelectSingleNode("collision");
                if (collisionNode != null)
                {
                    XmlNode physicsMaterial = collisionNode.SelectSingleNode("collision_material");
                    if (physicsMaterial == null)
                    {
                        physicsMaterial = collisionNode.SelectSingleNode("physics_material");
                    }
                    if (physicsMaterial != null)
                    {
                        string materialName = physicsMaterial.Attributes["name"]?.Value;
                        string linkName = link.Attributes["name"]?.Value;
                        GameObject targetObject = FindInChildrenByName(robotObject.transform, linkName);
                        if (targetObject != null)
                        {
                            Transform collisionTransform = targetObject.transform.Find("Collisions");
                            if (collisionTransform != null && collisionTransform.childCount > 0)
                            {
                                Transform unnamedCollision = collisionTransform.GetChild(0);
                                if (unnamedCollision.childCount > 0)
                                {
                                    Transform targetCollision = unnamedCollision.GetChild(0);
                                    if (targetCollision != null)
                                    {
                                        Collider meshCollider = targetCollision.gameObject.GetComponent<Collider>();
                                        if (meshCollider != null)
                                        {
                                            foreach (PhysicsMaterial material in physicsMaterialList)
                                            {
                                                if (material.name == materialName)
                                                {
                                                    meshCollider.material = material;
                                                    if (contactOffsetDict.TryGetValue(materialName, out float contactOffset))
                                                    {
                                                        meshCollider.contactOffset = contactOffset;
                                                    }
                                                    Debug.Log($"[CollisionMaterial] Applied '{materialName}' to '{linkName}' (static={material.staticFriction}, dynamic={material.dynamicFriction}, combine={material.frictionCombine}, contactOffset={meshCollider.contactOffset})");
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // サーボモデル (摩擦・バックラッシ) の設定: <servo_model joint="..."> 要素
        if (robotNode != null)
        {
            ServoJointModel[] servoModels = new ServoJointModel[articulationBodyList.Count];
            bool anyServoModel = false;
            XmlNodeList servoModelNodes = robotNode.SelectNodes("servo_model");
            foreach (XmlNode servoNode in servoModelNodes)
            {
                string servoJointName = servoNode.Attributes["joint"]?.Value;
                int jointIndex = jointNameList.IndexOf(servoJointName);
                if (jointIndex < 0)
                {
                    Debug.LogWarning($"[ServoModel] joint '{servoJointName}' not found, skipping");
                    continue;
                }
                ArticulationBody jointBody = articulationBodyList[jointIndex];
                ServoJointModel model = jointBody.gameObject.AddComponent<ServoJointModel>();

                // xDrive の stiffness/damping は SI (N·m/rad) のままモデルに流用できる
                // (target のみ度単位だが、剛性はラジアン換算誤差に掛かる)
                ArticulationDrive jointDrive = jointBody.xDrive;
                model.servoStiffness = jointDrive.stiffness;
                model.servoDamping = jointDrive.damping;
                if (jointDrive.forceLimit < float.MaxValue)
                    model.motorTorqueLimit = jointDrive.forceLimit;

                XmlNode servoFrictionNode = servoNode.SelectSingleNode("friction");
                if (servoFrictionNode != null)
                {
                    model.staticFriction = TryParseFloat(servoFrictionNode.Attributes["static"]?.Value);
                    model.dynamicFriction = TryParseFloat(servoFrictionNode.Attributes["dynamic"]?.Value);
                    model.stribeckVelocity = TryParseFloat(servoFrictionNode.Attributes["stribeck_velocity"]?.Value, 0.1f);
                    model.viscousFriction = TryParseFloat(servoFrictionNode.Attributes["viscous"]?.Value);
                }
                XmlNode backlashNode = servoNode.SelectSingleNode("backlash");
                if (backlashNode != null)
                {
                    model.backlashWidth = TryParseFloat(backlashNode.Attributes["width"]?.Value);
                    // Same default as ServoJointModel: 400 N*m/rad is past the
                    // point where the dead-zone term stays quantitative at
                    // 50 Hz, so a URDF that omits the attribute would silently
                    // get a transmission the model cannot resolve.
                    model.transmissionStiffness = TryParseFloat(backlashNode.Attributes["stiffness"]?.Value, 20f);
                    model.transmissionDamping = TryParseFloat(backlashNode.Attributes["damping"]?.Value, 0.5f);
                }
                XmlNode motorNode = servoNode.SelectSingleNode("motor");
                if (motorNode != null)
                {
                    model.motorInertia = TryParseFloat(motorNode.Attributes["inertia"]?.Value, 2e-3f);
                    if (motorNode.Attributes["p_gain"] != null)
                        model.servoStiffness = TryParseFloat(motorNode.Attributes["p_gain"].Value);
                    if (motorNode.Attributes["d_gain"] != null)
                        model.servoDamping = TryParseFloat(motorNode.Attributes["d_gain"].Value);
                    if (motorNode.Attributes["torque_limit"] != null)
                        model.motorTorqueLimit = TryParseFloat(motorNode.Attributes["torque_limit"].Value);
                }
                servoModels[jointIndex] = model;
                anyServoModel = true;
                Debug.Log($"[ServoModel] joint '{servoJointName}': friction(static={model.staticFriction}, dynamic={model.dynamicFriction}), backlash(width={model.backlashWidth}, K={model.transmissionStiffness}), servo(P={model.servoStiffness}, D={model.servoDamping})");
            }
            if (anyServoModel)
            {
                jointStateSub.servoModels = servoModels;
            }
        }

        // Buoyancy Material の収集と ArticulationFloatingObject/HydrodynamicFloatingObject の付与
        Dictionary<string, float> buoyancyMaterialDict = new Dictionary<string, float>();
        bool useHydrodynamics = false; // URDFでuse_hydrodynamics="true"が指定されているか
        HydrodynamicParameters hydrodynamicParams = null;

        if (robotNode != null)
        {
            XmlNodeList buoyancyMaterials = robotNode.SelectNodes("buoyancy_material");
            foreach (XmlNode buoyancyMaterial in buoyancyMaterials)
            {
                string materialName = buoyancyMaterial.Attributes["name"]?.Value;
                float density = 1.0f; // デフォルト密度
                XmlNode densityNode = buoyancyMaterial.SelectSingleNode("density");
                if (densityNode != null)
                {
                    density = TryParseFloat(densityNode.Attributes["value"]?.Value, 1.0f);
                }
                if (!string.IsNullOrEmpty(materialName))
                {
                    buoyancyMaterialDict[materialName] = density;
                }
            }

            // hydrodynamics要素をチェック (MARUS連携の水力学モデル) - ロボット全体のデフォルトパラメータ
            XmlNode defaultHydrodynamicsNode = robotNode.SelectSingleNode("hydrodynamics");
            if (defaultHydrodynamicsNode != null)
            {
                useHydrodynamics = true;
                hydrodynamicParams = ParseHydrodynamicsNode(defaultHydrodynamicsNode, null);
                Debug.Log($"[Hydrodynamics] Default params loaded: water_density={hydrodynamicParams.waterDensity}");
            }
        }

        // 全リンクに ArticulationFloatingObject または HydrodynamicFloatingObject を付与
        if (robotNode != null)
        {
            XmlNodeList links = robotNode.SelectNodes("link");
            foreach (XmlNode link in links)
            {
                string linkName = link.Attributes["name"]?.Value;
                GameObject targetObject = FindInChildrenByName(robotObject.transform, linkName);
                if (targetObject != null)
                {
                    ArticulationBody artBody = targetObject.GetComponent<ArticulationBody>();
                    Collider linkCollider = targetObject.GetComponentInChildren<Collider>();

                    if (artBody != null && linkCollider != null)
                    {
                        // Colliderがあるオブジェクトを探す
                        GameObject colliderObject = linkCollider.gameObject;

                        // ArticulationBodyがあるか確認、なければ親から取得
                        ArticulationBody targetArtBody = colliderObject.GetComponent<ArticulationBody>();
                        if (targetArtBody == null)
                        {
                            targetArtBody = colliderObject.GetComponentInParent<ArticulationBody>();
                        }

                        if (targetArtBody != null)
                        {
                            // デフォルト密度は1.0
                            float density = 1.0f;

                            // buoyancy_materialが指定されていれば、その密度を使用
                            XmlNode collisionNode = link.SelectSingleNode("collision");
                            if (collisionNode != null)
                            {
                                XmlNode buoyancyMaterialNode = collisionNode.SelectSingleNode("buoyancy_material");
                                if (buoyancyMaterialNode != null)
                                {
                                    string materialName = buoyancyMaterialNode.Attributes["name"]?.Value;
                                    if (!string.IsNullOrEmpty(materialName) && buoyancyMaterialDict.ContainsKey(materialName))
                                    {
                                        density = buoyancyMaterialDict[materialName];
                                    }
                                }
                            }

                            // リンク固有のhydrodynamicsパラメータをチェック
                            XmlNode linkHydrodynamicsNode = collisionNode?.SelectSingleNode("hydrodynamics");
                            bool linkHasHydrodynamics = linkHydrodynamicsNode != null;

                            if (useHydrodynamics || linkHasHydrodynamics)
                            {
                                // HydrodynamicFloatingObjectを追加 (MARUS水力学モデル)
                                HydrodynamicFloatingObject hydroObj = colliderObject.GetComponent<HydrodynamicFloatingObject>();
                                if (hydroObj == null)
                                {
                                    hydroObj = colliderObject.AddComponent<HydrodynamicFloatingObject>();
                                }
                                hydroObj.Density = density;

                                // リンク固有のパラメータがあれば、デフォルトを上書きしてマージ
                                if (linkHasHydrodynamics)
                                {
                                    HydrodynamicParameters linkParams = ParseHydrodynamicsNode(linkHydrodynamicsNode, hydrodynamicParams);
                                    hydroObj.Parameters = linkParams;
                                    Debug.Log($"Added HydrodynamicFloatingObject to {linkName} with density {density} (link-specific params)");
                                }
                                else if (hydrodynamicParams != null)
                                {
                                    hydroObj.Parameters = hydrodynamicParams;
                                    Debug.Log($"Added HydrodynamicFloatingObject to {linkName} with density {density} (default params)");
                                }
                                else
                                {
                                    Debug.Log($"Added HydrodynamicFloatingObject to {linkName} with density {density} (built-in defaults)");
                                }
                            }
                            else
                            {
                                // ArticulationFloatingObjectを追加 (従来の浮力のみ)
                                ArticulationFloatingObject floatingObj = colliderObject.GetComponent<ArticulationFloatingObject>();
                                if (floatingObj == null)
                                {
                                    floatingObj = colliderObject.AddComponent<ArticulationFloatingObject>();
                                }
                                floatingObj.Density = density;
                                Debug.Log($"Added ArticulationFloatingObject to {linkName} with density {density}");
                            }
                        }
                    }
                }
            }
        }

        // Aerodynamics 設定（空力学モデル）
        AeroSurfaceParameters defaultAeroParams = null;
        bool useAerodynamics = false;

        if (robotNode != null)
        {
            // ロボット全体のデフォルト空力学パラメータを読み込む
            XmlNode defaultAerodynamicsNode = robotNode.SelectSingleNode("aerodynamics");
            if (defaultAerodynamicsNode != null)
            {
                useAerodynamics = true;
                defaultAeroParams = ParseAerodynamicsNode(defaultAerodynamicsNode, null);
                Debug.Log($"[Aerodynamics] Default params loaded: liftSlope={defaultAeroParams.liftSlope}, chord={defaultAeroParams.chord}");
            }

            // リンクごとの空力学サーフェスを設定
            XmlNodeList links = robotNode.SelectNodes("link");
            foreach (XmlNode link in links)
            {
                string linkName = link.Attributes["name"]?.Value;
                GameObject targetObject = FindInChildrenByName(robotObject.transform, linkName);
                if (targetObject == null) continue;

                XmlNode collisionNode = link.SelectSingleNode("collision");
                if (collisionNode == null) continue;

                // aerodynamic_surface 要素を検索
                XmlNode aeroSurfaceNode = collisionNode.SelectSingleNode("aerodynamic_surface");
                if (aeroSurfaceNode == null && !useAerodynamics) continue;

                // 明示的な aerodynamic_surface 要素がある場合のみサーフェスを追加
                if (aeroSurfaceNode != null)
                {
                    // Colliderオブジェクトを取得
                    Collider linkCollider = targetObject.GetComponentInChildren<Collider>();
                    GameObject surfaceObject = linkCollider != null ? linkCollider.gameObject : targetObject;

                    // AeroSurface コンポーネントを追加
                    AeroSurface aeroSurface = surfaceObject.GetComponent<AeroSurface>();
                    if (aeroSurface == null)
                    {
                        aeroSurface = surfaceObject.AddComponent<AeroSurface>();
                    }

                    // パラメータを解析して設定
                    AeroSurfaceParameters surfaceParams = ParseAerodynamicsNode(aeroSurfaceNode, defaultAeroParams);
                    SetAeroSurfaceParameters(aeroSurface, surfaceParams);

                    // 制御面設定
                    XmlNode controlNode = aeroSurfaceNode.SelectSingleNode("control_surface");
                    if (controlNode != null)
                    {
                        SetAeroSurfaceControlSettings(aeroSurface, controlNode);
                    }

                    Debug.Log($"[Aerodynamics] Added AeroSurface to {linkName}: chord={surfaceParams.chord}, span={surfaceParams.span}, liftSlope={surfaceParams.liftSlope}");
                }
            }

            // AerodynamicsController をルートオブジェクトに追加（空力サーフェスがある場合のみ）
            AeroSurface[] allSurfaces = robotObject.GetComponentsInChildren<AeroSurface>();
            if (allSurfaces.Length > 0)
            {
                AerodynamicsController aeroController = robotObject.GetComponent<AerodynamicsController>();
                if (aeroController == null)
                {
                    aeroController = robotObject.AddComponent<AerodynamicsController>();
                }

                // デフォルトの流体密度を設定
                if (defaultAeroParams != null)
                {
                    aeroController.FluidDensity = defaultAeroParams.fluidDensity;
                }

                Debug.Log($"[Aerodynamics] Added AerodynamicsController with {allSurfaces.Length} surfaces");
            }
        }

        // センサ設定 (URDF 内の <simulation> 要素に基づく)
        int next_display_number = 1;
        if (robotNode != null)
        {
            XmlNode simulationNode = robotNode.SelectSingleNode("simulation");
            if (simulationNode == null)
            {
                Debug.LogWarning("<unity> is deprecated. Use <simulation> instead.");
                simulationNode = robotNode.SelectSingleNode("unity");
            }
            if (simulationNode != null)
            {
                XmlNodeList unitySensors = simulationNode.SelectNodes("sensor");
                foreach (XmlNode sensor in unitySensors)
                {
                    string sensorType = sensor.Attributes["type"]?.Value;
                    string sensorLinkName = sensor.Attributes["name"]?.Value;
                    GameObject targetObject = FindInChildrenByName(robotObject.transform, sensorLinkName);
                    if (targetObject != null)
                    {
                        switch (sensorType)
                        {
                            case "lidar":
                                Debug.Log("sensor type 'lidar' found");
                                // ランタイムで利用可能な LiDAR コンポーネントの追加処理を記述
                                RaycastLiDARSensor lidarSensor = targetObject.AddComponent<RaycastLiDARSensor>();

                                float lidarMinRange = TryParseFloat(sensor.SelectSingleNode("ray/range/min").InnerText);
                                float lidarMaxRange = TryParseFloat(sensor.SelectSingleNode("ray/range/max").InnerText);
                                float lidarGaussianNoiseSigma = TryParseFloat(sensor.SelectSingleNode("ray/noise/stddev").InnerText);

                                if (sensor.SelectSingleNode("ray/scan/vertical") == null)
                                {
                                    int pointsNumPerScan = int.Parse(sensor.SelectSingleNode("ray/scan/horizontal/samples").InnerText);
                                    var scanPattern = ScriptableObject.CreateInstance<ScanPattern>();
                                    scanPattern.size = pointsNumPerScan;
                                    scanPattern.scans = new Unity.Mathematics.float3[scanPattern.size];
                                    scanPattern.minZenithAngle = Mathf.PI / 2.0f;
                                    scanPattern.maxZenithAngle = Mathf.PI / 2.0f;
                                    scanPattern.minAzimuthAngle = TryParseFloat(sensor.SelectSingleNode("ray/scan/horizontal/min_angle").InnerText);
                                    scanPattern.maxAzimuthAngle = TryParseFloat(sensor.SelectSingleNode("ray/scan/horizontal/max_angle").InnerText);
                                    float angleStep = (scanPattern.maxAzimuthAngle - scanPattern.minAzimuthAngle) / (scanPattern.size - 1);
                                    for (int i = 0; i < scanPattern.size; i++)
                                    {
                                        float azimuth = scanPattern.minAzimuthAngle + i * angleStep;
                                        // 水平面上のスキャン方向を設定 (zenithは90度固定)
                                        scanPattern.scans[i] = new Unity.Mathematics.float3(
                                            -Mathf.Sin(azimuth),
                                            0.0f,  // 水平面なのでY=0
                                            Mathf.Cos(azimuth)
                                        );
                                    }
                                    // Use public Configure API instead of Reflection
                                    lidarSensor.Configure(scanPattern, lidarMinRange, lidarMaxRange, lidarGaussianNoiseSigma, pointsNumPerScan);
                                    lidarSensor.Initialize();

                                    // Set update rate from URDF AFTER Initialize()
                                    var lidarUpdateRateNode = sensor.SelectSingleNode("update_rate");
                                    if (lidarUpdateRateNode != null)
                                    {
                                        float updateRate = TryParseFloat(lidarUpdateRateNode.InnerText);
                                        SetSensorUpdateRate(lidarSensor, updateRate, "LiDAR:" + sensorLinkName);
                                    }

                                    LaserScanMsgPublisher laserScanMsgPublisher = targetObject.AddComponent<LaserScanMsgPublisher>();
                                    // Set publisher update rate to match sensor
                                    if (lidarUpdateRateNode != null)
                                    {
                                        float updateRate = TryParseFloat(lidarUpdateRateNode.InnerText);
                                        SetPublisherUpdateRate(laserScanMsgPublisher, updateRate, "LaserScan:" + sensorLinkName);
                                    }
                                    // Use public API instead of Reflection
                                    var laserScanMsgPublisherSerializer = new LaserScanMsgSerializer();
                                    laserScanMsgPublisherSerializer.SetSource(lidarSensor);
                                    var laserScanHeader = new HeaderSerializer();
                                    laserScanHeader.Configure(lidarSensor, sensorLinkName);
                                    laserScanMsgPublisherSerializer.Configure(laserScanHeader, scanPattern, lidarMinRange, lidarMaxRange, lidarGaussianNoiseSigma);
                                    laserScanMsgPublisher.serializer = laserScanMsgPublisherSerializer;
                                    laserScanMsgPublisher.topicName = TrackPublishedTopic(robotObject.name, entityNamespace, "/" + robotObject.name + "/" + sensorLinkName + "/scan");
                                }
                                else
                                {
                                    int verticalSamples = int.Parse(sensor.SelectSingleNode("ray/scan/vertical/samples").InnerText);
                                    int horizontalSamples = int.Parse(sensor.SelectSingleNode("ray/scan/horizontal/samples").InnerText);
                                    int pointsNumPerScan3D = horizontalSamples * verticalSamples;
                                    var scanPattern = ScriptableObject.CreateInstance<ScanPattern>();
                                    scanPattern.size = pointsNumPerScan3D;
                                    scanPattern.scans = new Unity.Mathematics.float3[scanPattern.size];
                                    scanPattern.minZenithAngle = TryParseFloat(sensor.SelectSingleNode("ray/scan/vertical/min_angle").InnerText);
                                    scanPattern.maxZenithAngle = TryParseFloat(sensor.SelectSingleNode("ray/scan/vertical/max_angle").InnerText);
                                    scanPattern.minAzimuthAngle = TryParseFloat(sensor.SelectSingleNode("ray/scan/horizontal/min_angle").InnerText);
                                    scanPattern.maxAzimuthAngle = TryParseFloat(sensor.SelectSingleNode("ray/scan/horizontal/max_angle").InnerText);
                                    float verticalAngleStep = (scanPattern.maxZenithAngle - scanPattern.minZenithAngle) / (verticalSamples - 1);
                                    float horizontalAngleStep = (scanPattern.maxAzimuthAngle - scanPattern.minAzimuthAngle) / (horizontalSamples - 1);
                                    for (int i = 0; i < horizontalSamples; i++)
                                    {
                                        float azimuth = scanPattern.minAzimuthAngle + i * horizontalAngleStep;
                                        for (int j = 0; j < verticalSamples; j++)
                                        {
                                            float zenith = scanPattern.minZenithAngle + i * verticalAngleStep;

                                            scanPattern.scans[i * verticalSamples + j] = new Unity.Mathematics.float3(
                                                Mathf.Cos(zenith) * -Mathf.Sin(azimuth),
                                                Mathf.Sin(zenith),
                                                Mathf.Cos(zenith) * Mathf.Cos(azimuth)
                                            );
                                        }
                                    }
                                    // Use public Configure API instead of Reflection
                                    lidarSensor.Configure(scanPattern, lidarMinRange, lidarMaxRange, lidarGaussianNoiseSigma, pointsNumPerScan3D);
                                    lidarSensor.Initialize();

                                    // Set update rate from URDF AFTER Initialize()
                                    var lidar3DUpdateRateNode = sensor.SelectSingleNode("update_rate");
                                    if (lidar3DUpdateRateNode != null)
                                    {
                                        float updateRate = TryParseFloat(lidar3DUpdateRateNode.InnerText);
                                        SetSensorUpdateRate(lidarSensor, updateRate, "LiDAR3D:" + sensorLinkName);
                                    }

                                    LiDARPointCloud2MsgPublisher lidarPointCloud2MsgPublisher = targetObject.AddComponent<LiDARPointCloud2MsgPublisher>();
                                    // Set publisher update rate to match sensor
                                    if (lidar3DUpdateRateNode != null)
                                    {
                                        float updateRate = TryParseFloat(lidar3DUpdateRateNode.InnerText);
                                        SetPublisherUpdateRate(lidarPointCloud2MsgPublisher, updateRate, "LiDARPointCloud2:" + sensorLinkName);
                                    }
                                    // Use public API instead of Reflection
                                    var lidarPointCloud2MsgPublisherSerializer = new PointCloud2MsgSerializer<UnitySensors.DataType.Sensor.PointCloud.PointXYZI>();
                                    lidarPointCloud2MsgPublisherSerializer.SetSource(lidarSensor);
                                    var pointCloud2Header = new HeaderSerializer();
                                    pointCloud2Header.Configure(lidarSensor, sensorLinkName);
                                    lidarPointCloud2MsgPublisherSerializer.Configure(pointCloud2Header);
                                    lidarPointCloud2MsgPublisher.serializer = lidarPointCloud2MsgPublisherSerializer;
                                    lidarPointCloud2MsgPublisher.topicName = TrackPublishedTopic(robotObject.name, entityNamespace, "/" + robotObject.name + "/" + sensorLinkName + "/scan");
                                }
                                break;
                            case "camera":
                                Debug.Log("sensor type 'camera' found");
                                RGBCameraSensor cameraSensor = targetObject.AddComponent<RGBCameraSensor>();
                                float cameraFov = TryParseFloat(sensor.SelectSingleNode("horizontal_fov").InnerText) * 180.0f / 3.14f;
                                int image_width, image_height;
                                int.TryParse(sensor.SelectSingleNode("image/width").InnerText, out image_width);
                                int.TryParse(sensor.SelectSingleNode("image/height").InnerText, out image_height);
                                // Use public Configure API instead of Reflection
                                cameraSensor.Configure(new Vector2Int(image_width, image_height), cameraFov);
                                // Set update rate from URDF
                                var updateRateNode = sensor.SelectSingleNode("update_rate");
                                if (updateRateNode != null)
                                {
                                    float updateRate = TryParseFloat(updateRateNode.InnerText);
                                    SetSensorUpdateRate(cameraSensor, updateRate, "Camera:" + sensorLinkName);
                                }
                                UnityEngine.Camera cameraComponent = targetObject.GetComponent<UnityEngine.Camera>();
                                if (cameraComponent != null)
                                {
                                    cameraComponent.targetDisplay = next_display_number;
                                    next_display_number++;
                                }
                                CameraInfoMsgPublisher cameraInfoPublisher = targetObject.AddComponent<CameraInfoMsgPublisher>();
                                ImageMsgPublisher cameraImagePublisher = targetObject.AddComponent<ImageMsgPublisher>();
                                // Set publisher update rate to match sensor
                                if (updateRateNode != null)
                                {
                                    float updateRate = TryParseFloat(updateRateNode.InnerText);
                                    SetPublisherUpdateRate(cameraInfoPublisher, updateRate, "CameraInfo:" + sensorLinkName);
                                    SetPublisherUpdateRate(cameraImagePublisher, updateRate, "CameraImage:" + sensorLinkName);
                                }
                                // Use public API instead of Reflection
                                var cameraInfoHeader = new HeaderSerializer();
                                cameraInfoHeader.Configure(cameraSensor, sensorLinkName);
                                var cameraInfoSerializer = new CameraInfoMsgSerializer();
                                cameraInfoSerializer.Configure(cameraSensor, cameraInfoHeader);
                                cameraInfoPublisher.serializer = cameraInfoSerializer;
                                cameraInfoPublisher.topicName = TrackPublishedTopic(robotObject.name, entityNamespace, "/" + robotObject.name + "/" + sensorLinkName + "/camera_info");

                                var cameraImageHeader = new HeaderSerializer();
                                cameraImageHeader.Configure(cameraSensor, sensorLinkName);
                                var cameraImageSerializer = new ImageMsgSerializer();
                                cameraImageSerializer.Configure(cameraSensor, cameraImageHeader, 0, 0); // Texture0, RGB8
                                cameraImagePublisher.serializer = cameraImageSerializer;
                                cameraImagePublisher.topicName = TrackPublishedTopic(robotObject.name, entityNamespace, "/" + robotObject.name + "/" + sensorLinkName + "/image_raw");
                                break;
                            case "wideanglecamera":
                                Debug.Log("sensor type 'wideanglecamera' found");
                                FisheyeCameraSensor fisheyeCameraSensor = targetObject.AddComponent<FisheyeCameraSensor>();
                                float fisheyeFov = TryParseFloat(sensor.SelectSingleNode("horizontal_fov").InnerText) * 180.0f / 3.14f;
                                int fisheye_image_width, fisheye_image_height;
                                int.TryParse(sensor.SelectSingleNode("image/width").InnerText, out fisheye_image_width);
                                int.TryParse(sensor.SelectSingleNode("image/height").InnerText, out fisheye_image_height);
                                // Use public Configure API instead of Reflection
                                fisheyeCameraSensor.Configure(new Vector2Int(fisheye_image_width, fisheye_image_height), fisheyeFov);
                                // Set update rate from URDF
                                var fisheyeUpdateRateNode = sensor.SelectSingleNode("update_rate");
                                if (fisheyeUpdateRateNode != null)
                                {
                                    float updateRate = TryParseFloat(fisheyeUpdateRateNode.InnerText);
                                    SetSensorUpdateRate(fisheyeCameraSensor, updateRate, "FisheyeCamera:" + sensorLinkName);
                                }
                                UnityEngine.Camera fisheyeCameraComponent = targetObject.GetComponent<UnityEngine.Camera>();
                                if (fisheyeCameraComponent != null)
                                {
                                    fisheyeCameraComponent.targetDisplay = next_display_number;
                                    next_display_number++;
                                }
                                CameraInfoMsgPublisher fisheyeCameraInfoPublisher = targetObject.AddComponent<CameraInfoMsgPublisher>();
                                ImageMsgPublisher fisheyeCameraImagePublisher = targetObject.AddComponent<ImageMsgPublisher>();
                                // Set publisher update rate to match sensor
                                if (fisheyeUpdateRateNode != null)
                                {
                                    float updateRate = TryParseFloat(fisheyeUpdateRateNode.InnerText);
                                    SetPublisherUpdateRate(fisheyeCameraInfoPublisher, updateRate, "FisheyeCameraInfo:" + sensorLinkName);
                                    SetPublisherUpdateRate(fisheyeCameraImagePublisher, updateRate, "FisheyeCameraImage:" + sensorLinkName);
                                }
                                // Use public API instead of Reflection
                                var fisheyeCameraInfoHeader = new HeaderSerializer();
                                fisheyeCameraInfoHeader.Configure(fisheyeCameraSensor, sensorLinkName);
                                var fisheyeCameraInfoSerializer = new CameraInfoMsgSerializer();
                                fisheyeCameraInfoSerializer.Configure(fisheyeCameraSensor, fisheyeCameraInfoHeader);
                                fisheyeCameraInfoPublisher.serializer = fisheyeCameraInfoSerializer;
                                fisheyeCameraInfoPublisher.topicName = TrackPublishedTopic(robotObject.name, entityNamespace, "/" + robotObject.name + "/" + sensorLinkName + "/fisheye_camera_info");

                                var fisheyeCameraImageHeader = new HeaderSerializer();
                                fisheyeCameraImageHeader.Configure(fisheyeCameraSensor, sensorLinkName);
                                var fisheyeCameraImageSerializer = new ImageMsgSerializer();
                                fisheyeCameraImageSerializer.Configure(fisheyeCameraSensor, fisheyeCameraImageHeader, 0, 0); // Texture0, RGB8
                                fisheyeCameraImagePublisher.serializer = fisheyeCameraImageSerializer;
                                fisheyeCameraImagePublisher.topicName = TrackPublishedTopic(robotObject.name, entityNamespace, "/" + robotObject.name + "/" + sensorLinkName + "/fisheye_image_raw");
                                break;
                            case "panoramiccamera":
                                Debug.Log("sensor type 'panoramiccamera' found");
                                PanoramicCameraSensor panoramicCameraSensor = targetObject.AddComponent<PanoramicCameraSensor>();
                                float panoramicFov = TryParseFloat(sensor.SelectSingleNode("horizontal_fov").InnerText) * 180.0f / 3.14f;
                                int panoramic_image_width, panoramic_image_height;
                                int.TryParse(sensor.SelectSingleNode("image/width").InnerText, out panoramic_image_width);
                                int.TryParse(sensor.SelectSingleNode("image/height").InnerText, out panoramic_image_height);
                                // Use public Configure API instead of Reflection
                                panoramicCameraSensor.Configure(new Vector2Int(panoramic_image_width, panoramic_image_height), panoramicFov);
                                // Set update rate from URDF
                                var panoramicUpdateRateNode = sensor.SelectSingleNode("update_rate");
                                if (panoramicUpdateRateNode != null)
                                {
                                    float updateRate = TryParseFloat(panoramicUpdateRateNode.InnerText);
                                    SetSensorUpdateRate(panoramicCameraSensor, updateRate, "PanoramicCamera:" + sensorLinkName);
                                }
                                UnityEngine.Camera panoramicCameraComponent = targetObject.GetComponent<UnityEngine.Camera>();
                                if (panoramicCameraComponent != null)
                                {
                                    panoramicCameraComponent.targetDisplay = next_display_number;
                                    next_display_number++;
                                }
                                CameraInfoMsgPublisher panoramicCameraInfoPublisher = targetObject.AddComponent<CameraInfoMsgPublisher>();
                                CompressedImageMsgPublisher panoramicCameraImagePublisher = targetObject.AddComponent<CompressedImageMsgPublisher>();
                                // Set publisher update rate to match sensor
                                if (panoramicUpdateRateNode != null)
                                {
                                    float updateRate = TryParseFloat(panoramicUpdateRateNode.InnerText);
                                    SetPublisherUpdateRate(panoramicCameraInfoPublisher, updateRate, "PanoramicCameraInfo:" + sensorLinkName);
                                    SetPublisherUpdateRate(panoramicCameraImagePublisher, updateRate, "PanoramicCameraImage:" + sensorLinkName);
                                }
                                // Use public API instead of Reflection
                                var panoramicCameraInfoHeader = new HeaderSerializer();
                                panoramicCameraInfoHeader.Configure(panoramicCameraSensor, sensorLinkName);
                                var panoramicCameraInfoSerializer = new CameraInfoMsgSerializer();
                                panoramicCameraInfoSerializer.Configure(panoramicCameraSensor, panoramicCameraInfoHeader);
                                panoramicCameraInfoPublisher.serializer = panoramicCameraInfoSerializer;
                                panoramicCameraInfoPublisher.topicName = TrackPublishedTopic(robotObject.name, entityNamespace, "/" + robotObject.name + "/" + sensorLinkName + "/panoramic_camera_info");

                                var panoramicCameraImageHeader = new HeaderSerializer();
                                panoramicCameraImageHeader.Configure(panoramicCameraSensor, sensorLinkName);
                                var panoramicCameraImageSerializer = new CompressedImageMsgSerializer();
                                panoramicCameraImageSerializer.Configure(panoramicCameraSensor, panoramicCameraImageHeader, 0); // Texture0
                                panoramicCameraImagePublisher.serializer = panoramicCameraImageSerializer;
                                panoramicCameraImagePublisher.topicName = TrackPublishedTopic(robotObject.name, entityNamespace, "/" + robotObject.name + "/" + sensorLinkName + "/panoramic_image_raw");
                                break;
                            case "depth_camera":
                                Debug.Log("sensor type 'depth_camera' found");
                                DepthCameraSensor depthCameraSensor = targetObject.AddComponent<DepthCameraSensor>();
                                float depthFov = TryParseFloat(sensor.SelectSingleNode("horizontal_fov").InnerText) * 180.0f / 3.14f;
                                int depth_image_width, depth_image_height;
                                int.TryParse(sensor.SelectSingleNode("image/width").InnerText, out depth_image_width);
                                int.TryParse(sensor.SelectSingleNode("image/height").InnerText, out depth_image_height);
                                // Get min/max range for depth camera
                                var depthMinRangeNode = sensor.SelectSingleNode("clip/near");
                                var depthMaxRangeNode = sensor.SelectSingleNode("clip/far");
                                float depthMinRange = depthMinRangeNode != null ? TryParseFloat(depthMinRangeNode.InnerText) : 0.05f;
                                float depthMaxRange = depthMaxRangeNode != null ? TryParseFloat(depthMaxRangeNode.InnerText) : 100.0f;
                                // Use public Configure API instead of Reflection
                                depthCameraSensor.Configure(new Vector2Int(depth_image_width, depth_image_height), depthFov, depthMinRange, depthMaxRange);

                                // Set update rate from URDF
                                var depthUpdateRateNode = sensor.SelectSingleNode("update_rate");
                                if (depthUpdateRateNode != null)
                                {
                                    float updateRate = TryParseFloat(depthUpdateRateNode.InnerText);
                                    SetSensorUpdateRate(depthCameraSensor, updateRate, "DepthCamera:" + sensorLinkName);
                                }
                                UnityEngine.Camera depthCameraComponent = targetObject.GetComponent<UnityEngine.Camera>();
                                if (depthCameraComponent != null)
                                {
                                    depthCameraComponent.targetDisplay = next_display_number;
                                    next_display_number++;
                                    Debug.Log("Depth camera component configured");
                                }
                                CameraInfoMsgPublisher depthCameraInfoPublisher = targetObject.AddComponent<CameraInfoMsgPublisher>();
                                ImageMsgPublisher depthCameraImagePublisher = targetObject.AddComponent<ImageMsgPublisher>();
                                // Set publisher update rate to match sensor
                                if (depthUpdateRateNode != null)
                                {
                                    float updateRate = TryParseFloat(depthUpdateRateNode.InnerText);
                                    SetPublisherUpdateRate(depthCameraInfoPublisher, updateRate, "DepthCameraInfo:" + sensorLinkName);
                                    SetPublisherUpdateRate(depthCameraImagePublisher, updateRate, "DepthCameraImage:" + sensorLinkName);
                                }
                                // Use public API instead of Reflection
                                var depthCameraInfoHeader = new HeaderSerializer();
                                depthCameraInfoHeader.Configure(depthCameraSensor, sensorLinkName);
                                var depthCameraInfoSerializer = new CameraInfoMsgSerializer();
                                depthCameraInfoSerializer.Configure(depthCameraSensor, depthCameraInfoHeader);
                                depthCameraInfoPublisher.serializer = depthCameraInfoSerializer;
                                depthCameraInfoPublisher.topicName = TrackPublishedTopic(robotObject.name, entityNamespace, "/" + robotObject.name + "/" + sensorLinkName + "/depth_camera_info");

                                var depthCameraImageHeader = new HeaderSerializer();
                                depthCameraImageHeader.Configure(depthCameraSensor, sensorLinkName);
                                var depthCameraImageSerializer = new ImageMsgSerializer();
                                depthCameraImageSerializer.Configure(depthCameraSensor, depthCameraImageHeader, 0, 1); // Texture0, 32FC1
                                depthCameraImagePublisher.serializer = depthCameraImageSerializer;
                                depthCameraImagePublisher.topicName = TrackPublishedTopic(robotObject.name, entityNamespace, "/" + robotObject.name + "/" + sensorLinkName + "/depth_image_raw");
                                break;
                            case "rgbd_camera":
                                Debug.Log("sensor type 'rgbd_camera' found");
                                RGBDCameraSensor rgbdCameraSensor = targetObject.AddComponent<RGBDCameraSensor>();
                                float rgbdFov = TryParseFloat(sensor.SelectSingleNode("horizontal_fov").InnerText) * 180.0f / 3.14f;
                                int rgbd_image_width, rgbd_image_height;
                                int.TryParse(sensor.SelectSingleNode("image/width").InnerText, out rgbd_image_width);
                                int.TryParse(sensor.SelectSingleNode("image/height").InnerText, out rgbd_image_height);
                                // Get min/max range for RGBD camera
                                var rgbdMinRangeNode = sensor.SelectSingleNode("clip/near");
                                var rgbdMaxRangeNode = sensor.SelectSingleNode("clip/far");
                                float rgbdMinRange = rgbdMinRangeNode != null ? TryParseFloat(rgbdMinRangeNode.InnerText) : 0.05f;
                                float rgbdMaxRange = rgbdMaxRangeNode != null ? TryParseFloat(rgbdMaxRangeNode.InnerText) : 100.0f;
                                // Use public Configure API instead of Reflection
                                rgbdCameraSensor.Configure(new Vector2Int(rgbd_image_width, rgbd_image_height), rgbdFov, rgbdMinRange, rgbdMaxRange);

                                Debug.Log("RGBD camera component configured");

                                // Set update rate from URDF
                                var rgbdUpdateRateNode = sensor.SelectSingleNode("update_rate");
                                if (rgbdUpdateRateNode != null)
                                {
                                    float updateRate = TryParseFloat(rgbdUpdateRateNode.InnerText);
                                    SetSensorUpdateRate(rgbdCameraSensor, updateRate, "RGBDCamera:" + sensorLinkName);
                                }
                                UnityEngine.Camera rgbdCameraComponent = targetObject.GetComponent<UnityEngine.Camera>();
                                if (rgbdCameraComponent != null)
                                {
                                    rgbdCameraComponent.targetDisplay = next_display_number;
                                    next_display_number++;
                                }

                                // Setup publishers for both depth and color
                                CameraInfoMsgPublisher rgbdDepthCameraInfoPublisher = targetObject.AddComponent<CameraInfoMsgPublisher>();
                                ImageMsgPublisher rgbdDepthImagePublisher = targetObject.AddComponent<ImageMsgPublisher>();
                                CameraInfoMsgPublisher rgbdColorCameraInfoPublisher = targetObject.AddComponent<CameraInfoMsgPublisher>();
                                ImageMsgPublisher rgbdColorImagePublisher = targetObject.AddComponent<ImageMsgPublisher>();

                                // Set publisher update rates to match sensor
                                if (rgbdUpdateRateNode != null)
                                {
                                    float updateRate = TryParseFloat(rgbdUpdateRateNode.InnerText);
                                    SetPublisherUpdateRate(rgbdDepthCameraInfoPublisher, updateRate, "RGBDDepthCameraInfo:" + sensorLinkName);
                                    SetPublisherUpdateRate(rgbdDepthImagePublisher, updateRate, "RGBDDepthImage:" + sensorLinkName);
                                    SetPublisherUpdateRate(rgbdColorCameraInfoPublisher, updateRate, "RGBDColorCameraInfo:" + sensorLinkName);
                                    SetPublisherUpdateRate(rgbdColorImagePublisher, updateRate, "RGBDColorImage:" + sensorLinkName);
                                }

                                // Use public API instead of Reflection
                                // Configure depth camera info publisher
                                var rgbdDepthCameraInfoHeader = new HeaderSerializer();
                                rgbdDepthCameraInfoHeader.Configure(rgbdCameraSensor, sensorLinkName);
                                var rgbdDepthCameraInfoSerializer = new CameraInfoMsgSerializer();
                                rgbdDepthCameraInfoSerializer.Configure(rgbdCameraSensor, rgbdDepthCameraInfoHeader);
                                rgbdDepthCameraInfoPublisher.serializer = rgbdDepthCameraInfoSerializer;
                                rgbdDepthCameraInfoPublisher.topicName = TrackPublishedTopic(robotObject.name, entityNamespace, "/" + robotObject.name + "/" + sensorLinkName + "/depth/camera_info");

                                // Configure depth image publisher
                                var rgbdDepthImageHeader = new HeaderSerializer();
                                rgbdDepthImageHeader.Configure(rgbdCameraSensor, sensorLinkName);
                                var rgbdDepthImageSerializer = new ImageMsgSerializer();
                                rgbdDepthImageSerializer.Configure(rgbdCameraSensor, rgbdDepthImageHeader, 0, 1); // Texture0 (depth), 32FC1
                                rgbdDepthImagePublisher.serializer = rgbdDepthImageSerializer;
                                rgbdDepthImagePublisher.topicName = TrackPublishedTopic(robotObject.name, entityNamespace, "/" + robotObject.name + "/" + sensorLinkName + "/depth/image_raw");

                                // Configure color camera info publisher
                                var rgbdColorCameraInfoHeader = new HeaderSerializer();
                                rgbdColorCameraInfoHeader.Configure(rgbdCameraSensor, sensorLinkName);
                                var rgbdColorCameraInfoSerializer = new CameraInfoMsgSerializer();
                                rgbdColorCameraInfoSerializer.Configure(rgbdCameraSensor, rgbdColorCameraInfoHeader);
                                rgbdColorCameraInfoPublisher.serializer = rgbdColorCameraInfoSerializer;
                                rgbdColorCameraInfoPublisher.topicName = TrackPublishedTopic(robotObject.name, entityNamespace, "/" + robotObject.name + "/" + sensorLinkName + "/color/camera_info");

                                // Configure color image publisher
                                var rgbdColorImageHeader = new HeaderSerializer();
                                rgbdColorImageHeader.Configure(rgbdCameraSensor, sensorLinkName);
                                var rgbdColorImageSerializer = new ImageMsgSerializer();
                                rgbdColorImageSerializer.Configure(rgbdCameraSensor, rgbdColorImageHeader, 1, 0); // Texture1 (color), RGB8
                                rgbdColorImagePublisher.serializer = rgbdColorImageSerializer;
                                rgbdColorImagePublisher.topicName = TrackPublishedTopic(robotObject.name, entityNamespace, "/" + robotObject.name + "/" + sensorLinkName + "/color/image_raw");
                                break;
                            case "imu":
                                Debug.Log("sensor type 'imu' found");
                                // Add IMU sensor component
                                IMUSensor imuSensor = targetObject.AddComponent<IMUSensor>();

                                // Set update rate from URDF
                                var imuUpdateRateNode = sensor.SelectSingleNode("update_rate");
                                if (imuUpdateRateNode != null)
                                {
                                    float updateRate = TryParseFloat(imuUpdateRateNode.InnerText);
                                    SetSensorUpdateRate(imuSensor, updateRate, "IMU:" + sensorLinkName);
                                }

                                // Add IMU message publisher
                                IMUMsgPublisher imuMsgPublisher = targetObject.AddComponent<IMUMsgPublisher>();

                                // Set publisher update rate to match sensor
                                if (imuUpdateRateNode != null)
                                {
                                    float updateRate = TryParseFloat(imuUpdateRateNode.InnerText);
                                    SetPublisherUpdateRate(imuMsgPublisher, updateRate, "IMU:" + sensorLinkName);
                                }

                                // Use public API instead of Reflection
                                var imuHeader = new HeaderSerializer();
                                imuHeader.Configure(imuSensor, sensorLinkName);
                                var imuSerializer = new IMUMsgSerializer();
                                imuSerializer.Configure(imuSensor, imuHeader);
                                imuMsgPublisher.serializer = imuSerializer;
                                imuMsgPublisher.topicName = TrackPublishedTopic(robotObject.name, entityNamespace, "/" + robotObject.name + "/" + sensorLinkName + "/imu");
                                break;
                            case "thruster":
                                Debug.Log("sensor type 'thruster' found");
                                string thrusterId = sensor.Attributes?["id"]?.Value;
                                string thrusterObjectName = string.IsNullOrEmpty(thrusterId)
                                    ? sensorLinkName + "_thruster"
                                    : sensorLinkName + "_" + thrusterId + "_thruster";

                                GameObject thrusterObject = new GameObject(thrusterObjectName);
                                thrusterObject.transform.SetParent(targetObject.transform, false);

                                XmlNode thrusterOriginNode = sensor.SelectSingleNode("origin");
                                Vector3 thrusterLocalPosition = Vector3.zero;
                                Vector3 thrusterLocalRpy = Vector3.zero;
                                if (thrusterOriginNode != null)
                                {
                                    thrusterLocalPosition = ParseVector3Attribute(thrusterOriginNode, "xyz", Vector3.zero);
                                    thrusterLocalRpy = ParseVector3Attribute(thrusterOriginNode, "rpy", Vector3.zero);
                                }

                                thrusterObject.transform.localPosition = thrusterLocalPosition;
                                thrusterObject.transform.localRotation = Quaternion.Euler(
                                    thrusterLocalRpy.x * Mathf.Rad2Deg,
                                    thrusterLocalRpy.y * Mathf.Rad2Deg,
                                    thrusterLocalRpy.z * Mathf.Rad2Deg);

                                Vector3 axis = Vector3.forward;
                                XmlNode axisNode = sensor.SelectSingleNode("axis");
                                if (axisNode != null)
                                {
                                    axis = ParseVector3Attribute(axisNode, "xyz", Vector3.forward);
                                }

                                float maxForce = 100f;
                                XmlNode maxForceNode = sensor.SelectSingleNode("max_force");
                                if (maxForceNode != null)
                                {
                                    maxForce = TryParseFloat(maxForceNode.InnerText, maxForce);
                                }

                                float initialThrottle = 0f;
                                XmlNode initialThrottleNode = sensor.SelectSingleNode("initial_throttle");
                                if (initialThrottleNode != null)
                                {
                                    initialThrottle = TryParseFloat(initialThrottleNode.InnerText, initialThrottle);
                                }

                                string commandTopic = null;
                                XmlNode commandTopicNode = sensor.SelectSingleNode("command_topic");
                                if (commandTopicNode != null)
                                {
                                    commandTopic = commandTopicNode.InnerText;
                                }

                                LinkThruster thruster = thrusterObject.AddComponent<LinkThruster>();
                                thruster.targetBody = targetObject.GetComponent<ArticulationBody>();
                                thruster.localDirection = axis;
                                thruster.maxForce = maxForce;
                                thruster.command = initialThrottle;
                                if (!string.IsNullOrEmpty(commandTopic))
                                {
                                    thruster.topicName = ApplyNamespace(entityNamespace, commandTopic);
                                }
                                break;
                            default:
                                Debug.Log("undefined sensor type found");
                                break;
                        }
                    }
                }
            }
        }

        // エディタ依存の DestroyImmediate の代わりに Destroy を利用
        GameObject.Destroy(robotObject.GetComponent<Controller>());

        return spawnEntityResponse;
    }

    /// <summary>
    /// エディタ用コルーチンをMonoBehaviourのStartCoroutineで代替するためのラッパー
    /// </summary>
    private Task<GameObject> WaitForCoroutine(IEnumerator<GameObject> coroutine)
    {
        var tcs = new TaskCompletionSource<GameObject>();
        StartCoroutine(HandleCoroutine(coroutine, tcs));
        return tcs.Task;
    }

    private IEnumerator HandleCoroutine(IEnumerator<GameObject> coroutine, TaskCompletionSource<GameObject> tcs)
    {
        GameObject result = null;
        while (coroutine.MoveNext())
        {
            // コルーチンの戻り値（GameObject）が返された場合は保持
            if (coroutine.Current != null)
            {
                result = coroutine.Current;
            }
            yield return null;
        }
        tcs.SetResult(result);
    }

    // 指定された名前の子オブジェクトを再帰的に検索
    private static GameObject FindInChildrenByName(Transform parent, string name)
    {
        if (parent.name == name)
            return parent.gameObject;

        foreach (Transform child in parent)
        {
            GameObject result = FindInChildrenByName(child, name);
            if (result != null)
                return result;
        }
        return null;
    }

    // 指定のコンポーネントを持つ子オブジェクト群を収集
    private static List<GameObject> GetChildObjectsWithComponent<T>(GameObject parent) where T : Component
    {
        List<GameObject> objectsWithComponent = new List<GameObject>();
        foreach (Transform child in parent.transform)
        {
            if (child.GetComponent<T>() != null)
                objectsWithComponent.Add(child.gameObject);
        }
        return objectsWithComponent;
    }

    // 子孫オブジェクトから ArticulationBody を再帰的に検索
    public static List<GameObject> FindArticulationBodyObjectsInChildren(GameObject parent)
    {
        List<GameObject> articulationBodies = new List<GameObject>();
        SearchArticulationBodies(parent.transform, articulationBodies);
        return articulationBodies;
    }

    private static void SearchArticulationBodies(Transform parent, List<GameObject> articulationBodies)
    {
        ArticulationBody articulationBody = parent.GetComponent<ArticulationBody>();
        if (articulationBody != null)
        {
            articulationBodies.Add(parent.gameObject);
        }
        foreach (Transform child in parent)
        {
            SearchArticulationBodies(child, articulationBodies);
        }
    }

    /// <summary>
    /// UnitySensorの更新レートを設定する（リフレクションを使用）
    /// </summary>
    private static void SetSensorUpdateRate(UnitySensors.Sensor.UnitySensor sensor, float updateRate, string sensorName)
    {
        Debug.Log($"Setting {sensorName} update rate to: {updateRate} Hz");

        var unitySensorType = typeof(UnitySensors.Sensor.UnitySensor);
        var bindingFlags = System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Instance;

        // Set _frequency
        var frequencyField = unitySensorType.GetField("_frequency", bindingFlags);
        if (frequencyField != null)
        {
            frequencyField.SetValue(sensor, updateRate);
            Debug.Log($"  _frequency set to: {updateRate}");
        }
        else
        {
            Debug.LogError($"  Failed to get _frequency field for {sensorName}");
        }

        // Set _frequency_inv
        var frequencyInvField = unitySensorType.GetField("_frequency_inv", bindingFlags);
        if (frequencyInvField != null)
        {
            float invValue = 1.0f / updateRate;
            frequencyInvField.SetValue(sensor, invValue);
            Debug.Log($"  _frequency_inv set to: {invValue}");
        }
        else
        {
            Debug.LogError($"  Failed to get _frequency_inv field for {sensorName}");
        }

        // Reset _dt
        var dtField = unitySensorType.GetField("_dt", bindingFlags);
        if (dtField != null)
        {
            dtField.SetValue(sensor, 0.0f);
            Debug.Log($"  _dt reset to 0");
        }
    }

    /// <summary>
    /// RosMsgPublisherの更新レートを設定する（リフレクションを使用）
    /// </summary>
    private static void SetPublisherUpdateRate(MonoBehaviour publisher, float updateRate, string publisherName)
    {
        Debug.Log($"Setting {publisherName} publisher update rate to: {updateRate} Hz");

        var bindingFlags = System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Instance;

        // Search through the inheritance hierarchy for the fields
        System.Type currentType = publisher.GetType();
        System.Reflection.FieldInfo frequencyField = null;
        System.Reflection.FieldInfo frequencyInvField = null;
        System.Reflection.FieldInfo dtField = null;

        while (currentType != null && currentType != typeof(MonoBehaviour))
        {
            if (frequencyField == null)
                frequencyField = currentType.GetField("_frequency", bindingFlags);
            if (frequencyInvField == null)
                frequencyInvField = currentType.GetField("_frequency_inv", bindingFlags);
            if (dtField == null)
                dtField = currentType.GetField("_dt", bindingFlags);

            if (frequencyField != null && frequencyInvField != null && dtField != null)
                break;

            currentType = currentType.BaseType;
        }

        // Set _frequency
        if (frequencyField != null)
        {
            frequencyField.SetValue(publisher, updateRate);
            Debug.Log($"  Publisher _frequency set to: {updateRate}");
        }
        else
        {
            Debug.LogError($"  Failed to get _frequency field for {publisherName}");
        }

        // Set _frequency_inv
        if (frequencyInvField != null)
        {
            float invValue = 1.0f / updateRate;
            frequencyInvField.SetValue(publisher, invValue);
            Debug.Log($"  Publisher _frequency_inv set to: {invValue}");
        }
        else
        {
            Debug.LogError($"  Failed to get _frequency_inv field for {publisherName}");
        }

        // Reset _dt
        if (dtField != null)
        {
            dtField.SetValue(publisher, 0.0f);
            Debug.Log($"  Publisher _dt reset to 0");
        }
    }

    /// <summary>
    /// URDF(ROS)座標系のクォータニオンを Unity 座標系のクォータニオンに変換する
    /// </summary>
    public static Quaternion ConvertQuaternion(Quaternion qURDF)
    {
        return Quaternion.Euler(
            qURDF.eulerAngles.y,
            -qURDF.eulerAngles.z,
            -qURDF.eulerAngles.x
        );
    }

    /// <summary>
    /// XMLノードからHydrodynamicParametersを解析する
    /// </summary>
    /// <param name="node">hydrodynamics XMLノード</param>
    /// <param name="defaults">デフォルト値（nullの場合は組み込みデフォルトを使用）</param>
    /// <returns>解析されたパラメータ</returns>
    private HydrodynamicParameters ParseHydrodynamicsNode(XmlNode node, HydrodynamicParameters defaults)
    {
        // デフォルト値を設定（指定がなければ組み込みデフォルト）
        var p = new HydrodynamicParameters();
        if (defaults != null)
        {
            // デフォルト値をコピー
            p.waterDensity = defaults.waterDensity;
            p.airDensity = defaults.airDensity;
            p.waterViscosity = defaults.waterViscosity;
            p.velocityReference = defaults.velocityReference;
            p.C_PD1 = defaults.C_PD1;
            p.C_PD2 = defaults.C_PD2;
            p.f_P = defaults.f_P;
            p.C_SD1 = defaults.C_SD1;
            p.C_SD2 = defaults.C_SD2;
            p.f_S = defaults.f_S;
            p.slammingPower = defaults.slammingPower;
            p.maxAcceleration = defaults.maxAcceleration;
            p.slammingMultiplier = defaults.slammingMultiplier;
            p.airResistanceCoefficient = defaults.airResistanceCoefficient;
            p.enableViscousResistance = defaults.enableViscousResistance;
            p.enablePressureDrag = defaults.enablePressureDrag;
            p.enableSlammingForce = defaults.enableSlammingForce;
            p.enableAirResistance = defaults.enableAirResistance;
        }

        if (node == null) return p;

        // 各パラメータを読み込み（指定があれば上書き）
        var waterDensityNode = node.SelectSingleNode("water_density");
        if (waterDensityNode != null)
            p.waterDensity = TryParseFloat(waterDensityNode.InnerText, p.waterDensity);

        var airDensityNode = node.SelectSingleNode("air_density");
        if (airDensityNode != null)
            p.airDensity = TryParseFloat(airDensityNode.InnerText, p.airDensity);

        var velocityRefNode = node.SelectSingleNode("velocity_reference");
        if (velocityRefNode != null)
            p.velocityReference = TryParseFloat(velocityRefNode.InnerText, p.velocityReference);

        // Pressure Drag
        var pressureDragNode = node.SelectSingleNode("pressure_drag");
        if (pressureDragNode != null)
        {
            var cpd1 = pressureDragNode.SelectSingleNode("C_PD1");
            if (cpd1 != null) p.C_PD1 = TryParseFloat(cpd1.InnerText, p.C_PD1);

            var cpd2 = pressureDragNode.SelectSingleNode("C_PD2");
            if (cpd2 != null) p.C_PD2 = TryParseFloat(cpd2.InnerText, p.C_PD2);

            var fp = pressureDragNode.SelectSingleNode("f_P");
            if (fp != null) p.f_P = TryParseFloat(fp.InnerText, p.f_P);
        }

        // Suction Drag
        var suctionDragNode = node.SelectSingleNode("suction_drag");
        if (suctionDragNode != null)
        {
            var csd1 = suctionDragNode.SelectSingleNode("C_SD1");
            if (csd1 != null) p.C_SD1 = TryParseFloat(csd1.InnerText, p.C_SD1);

            var csd2 = suctionDragNode.SelectSingleNode("C_SD2");
            if (csd2 != null) p.C_SD2 = TryParseFloat(csd2.InnerText, p.C_SD2);

            var fs = suctionDragNode.SelectSingleNode("f_S");
            if (fs != null) p.f_S = TryParseFloat(fs.InnerText, p.f_S);
        }

        // Slamming
        var slammingNode = node.SelectSingleNode("slamming");
        if (slammingNode != null)
        {
            var power = slammingNode.SelectSingleNode("power");
            if (power != null) p.slammingPower = TryParseFloat(power.InnerText, p.slammingPower);

            var maxAcc = slammingNode.SelectSingleNode("max_acceleration");
            if (maxAcc != null) p.maxAcceleration = TryParseFloat(maxAcc.InnerText, p.maxAcceleration);

            var mult = slammingNode.SelectSingleNode("multiplier");
            if (mult != null) p.slammingMultiplier = TryParseFloat(mult.InnerText, p.slammingMultiplier);
        }

        // Air Resistance
        var airResNode = node.SelectSingleNode("air_resistance");
        if (airResNode != null)
        {
            var coeff = airResNode.SelectSingleNode("coefficient");
            if (coeff != null) p.airResistanceCoefficient = TryParseFloat(coeff.InnerText, p.airResistanceCoefficient);
        }

        // Enable/Disable flags
        var enableNode = node.SelectSingleNode("enable");
        if (enableNode != null)
        {
            var viscous = enableNode.SelectSingleNode("viscous_resistance");
            if (viscous != null) p.enableViscousResistance = viscous.InnerText.ToLower() == "true";

            var pressure = enableNode.SelectSingleNode("pressure_drag");
            if (pressure != null) p.enablePressureDrag = pressure.InnerText.ToLower() == "true";

            var slamming = enableNode.SelectSingleNode("slamming_force");
            if (slamming != null) p.enableSlammingForce = slamming.InnerText.ToLower() == "true";

            var air = enableNode.SelectSingleNode("air_resistance");
            if (air != null) p.enableAirResistance = air.InnerText.ToLower() == "true";
        }

        return p;
    }

    /// <summary>
    /// XMLノードからAeroSurfaceParametersを解析する
    /// </summary>
    /// <param name="node">aerodynamics/aerodynamic_surface XMLノード</param>
    /// <param name="defaults">デフォルト値（nullの場合は組み込みデフォルトを使用）</param>
    /// <returns>解析されたパラメータ</returns>
    private AeroSurfaceParameters ParseAerodynamicsNode(XmlNode node, AeroSurfaceParameters defaults)
    {
        var p = new AeroSurfaceParameters();
        if (defaults != null)
        {
            // デフォルト値をコピー
            p.chord = defaults.chord;
            p.span = defaults.span;
            p.autoAspectRatio = defaults.autoAspectRatio;
            p.aspectRatio = defaults.aspectRatio;
            p.liftSlope = defaults.liftSlope;
            p.zeroLiftAoA = defaults.zeroLiftAoA;
            // 空気/水中別のストール角
            p.stallAngleHighAir = defaults.stallAngleHighAir;
            p.stallAngleLowAir = defaults.stallAngleLowAir;
            p.stallAngleHighWater = defaults.stallAngleHighWater;
            p.stallAngleLowWater = defaults.stallAngleLowWater;
            // 空気/水中別の摩擦係数
            p.skinFrictionAir = defaults.skinFrictionAir;
            p.skinFrictionWater = defaults.skinFrictionWater;
            p.flapFraction = defaults.flapFraction;
            p.maxFlapAngle = defaults.maxFlapAngle;
            p.fluidMedium = defaults.fluidMedium;
            // 空気/水中別の密度
            p.airDensity = defaults.airDensity;
            p.waterDensity = defaults.waterDensity;
            // 動粘度
            p.airKinematicViscosity = defaults.airKinematicViscosity;
            p.waterKinematicViscosity = defaults.waterKinematicViscosity;
            // キャビテーション
            p.enableCavitation = defaults.enableCavitation;
            p.cavitationThreshold = defaults.cavitationThreshold;
        }

        if (node == null) return p;

        // Geometry
        var chordNode = node.SelectSingleNode("chord");
        if (chordNode != null)
            p.chord = TryParseFloat(chordNode.InnerText, p.chord);

        var spanNode = node.SelectSingleNode("span");
        if (spanNode != null)
            p.span = TryParseFloat(spanNode.InnerText, p.span);

        var arNode = node.SelectSingleNode("aspect_ratio");
        if (arNode != null)
        {
            p.aspectRatio = TryParseFloat(arNode.InnerText, p.aspectRatio);
            p.autoAspectRatio = false;
        }

        var autoArAttr = node.Attributes?["auto_aspect_ratio"];
        if (autoArAttr != null)
            p.autoAspectRatio = autoArAttr.Value.ToLower() == "true";

        // Lift characteristics
        var liftSlopeNode = node.SelectSingleNode("lift_slope");
        if (liftSlopeNode != null)
            p.liftSlope = TryParseFloat(liftSlopeNode.InnerText, p.liftSlope);

        var zeroLiftNode = node.SelectSingleNode("zero_lift_aoa");
        if (zeroLiftNode != null)
            p.zeroLiftAoA = TryParseFloat(zeroLiftNode.InnerText, p.zeroLiftAoA);

        // ストール角 - 後方互換性のため単一値も対応
        var stallHighNode = node.SelectSingleNode("stall_angle_high");
        if (stallHighNode != null)
        {
            float val = TryParseFloat(stallHighNode.InnerText, p.stallAngleHighAir);
            p.stallAngleHighAir = val;
            p.stallAngleHighWater = val * 0.8f; // 水中は低め
        }

        var stallLowNode = node.SelectSingleNode("stall_angle_low");
        if (stallLowNode != null)
        {
            float val = TryParseFloat(stallLowNode.InnerText, p.stallAngleLowAir);
            p.stallAngleLowAir = val;
            p.stallAngleLowWater = val * 0.8f;
        }

        // 空気用ストール角（明示的指定）
        var stallHighAirNode = node.SelectSingleNode("stall_angle_high_air");
        if (stallHighAirNode != null)
            p.stallAngleHighAir = TryParseFloat(stallHighAirNode.InnerText, p.stallAngleHighAir);

        var stallLowAirNode = node.SelectSingleNode("stall_angle_low_air");
        if (stallLowAirNode != null)
            p.stallAngleLowAir = TryParseFloat(stallLowAirNode.InnerText, p.stallAngleLowAir);

        // 水中用ストール角（明示的指定）
        var stallHighWaterNode = node.SelectSingleNode("stall_angle_high_water");
        if (stallHighWaterNode != null)
            p.stallAngleHighWater = TryParseFloat(stallHighWaterNode.InnerText, p.stallAngleHighWater);

        var stallLowWaterNode = node.SelectSingleNode("stall_angle_low_water");
        if (stallLowWaterNode != null)
            p.stallAngleLowWater = TryParseFloat(stallLowWaterNode.InnerText, p.stallAngleLowWater);

        // Drag characteristics - 後方互換性のため単一値も対応
        var skinFrictionNode = node.SelectSingleNode("skin_friction");
        if (skinFrictionNode != null)
        {
            float val = TryParseFloat(skinFrictionNode.InnerText, p.skinFrictionAir);
            p.skinFrictionAir = val;
            p.skinFrictionWater = val * 0.5f; // 水中は低め
        }

        // 空気用摩擦係数（明示的指定）
        var skinFrictionAirNode = node.SelectSingleNode("skin_friction_air");
        if (skinFrictionAirNode != null)
            p.skinFrictionAir = TryParseFloat(skinFrictionAirNode.InnerText, p.skinFrictionAir);

        // 水中用摩擦係数（明示的指定）
        var skinFrictionWaterNode = node.SelectSingleNode("skin_friction_water");
        if (skinFrictionWaterNode != null)
            p.skinFrictionWater = TryParseFloat(skinFrictionWaterNode.InnerText, p.skinFrictionWater);

        // Control surface
        var flapFractionNode = node.SelectSingleNode("flap_fraction");
        if (flapFractionNode != null)
            p.flapFraction = TryParseFloat(flapFractionNode.InnerText, p.flapFraction);

        var maxFlapNode = node.SelectSingleNode("max_flap_angle");
        if (maxFlapNode != null)
            p.maxFlapAngle = TryParseFloat(maxFlapNode.InnerText, p.maxFlapAngle);

        // Fluid properties
        var fluidMediumNode = node.SelectSingleNode("fluid_medium");
        if (fluidMediumNode != null)
        {
            string medium = fluidMediumNode.InnerText.ToLower();
            p.fluidMedium = medium switch
            {
                "water" => FluidMedium.Water,
                "auto" => FluidMedium.Auto,
                _ => FluidMedium.Air
            };
        }

        // 後方互換性: 単一の fluid_density
        var fluidDensityNode = node.SelectSingleNode("fluid_density");
        if (fluidDensityNode != null)
        {
            float density = TryParseFloat(fluidDensityNode.InnerText, p.airDensity);
            // 密度値から空気か水かを推測
            if (density > 500f)
                p.waterDensity = density;
            else
                p.airDensity = density;
        }

        // 空気密度（明示的指定）
        var airDensityNode = node.SelectSingleNode("air_density");
        if (airDensityNode != null)
            p.airDensity = TryParseFloat(airDensityNode.InnerText, p.airDensity);

        // 水中密度（明示的指定）
        var waterDensityNode = node.SelectSingleNode("water_density");
        if (waterDensityNode != null)
            p.waterDensity = TryParseFloat(waterDensityNode.InnerText, p.waterDensity);

        // 動粘度
        var airViscosityNode = node.SelectSingleNode("air_kinematic_viscosity");
        if (airViscosityNode != null)
            p.airKinematicViscosity = TryParseFloat(airViscosityNode.InnerText, p.airKinematicViscosity);

        var waterViscosityNode = node.SelectSingleNode("water_kinematic_viscosity");
        if (waterViscosityNode != null)
            p.waterKinematicViscosity = TryParseFloat(waterViscosityNode.InnerText, p.waterKinematicViscosity);

        // キャビテーション設定
        var cavitationNode = node.SelectSingleNode("enable_cavitation");
        if (cavitationNode != null)
            p.enableCavitation = cavitationNode.InnerText.ToLower() == "true";

        var cavitationThresholdNode = node.SelectSingleNode("cavitation_threshold");
        if (cavitationThresholdNode != null)
            p.cavitationThreshold = TryParseFloat(cavitationThresholdNode.InnerText, p.cavitationThreshold);

        // Blade Element Theory 設定
        var betNode = node.SelectSingleNode("blade_element_theory");
        if (betNode != null)
        {
            var numElementsNode = betNode.SelectSingleNode("num_elements");
            if (numElementsNode != null)
                p.numElements = Mathf.Clamp((int)TryParseFloat(numElementsNode.InnerText, p.numElements), 1, 32);

            var inducedModelNode = betNode.SelectSingleNode("induced_velocity_model");
            if (inducedModelNode != null)
            {
                string modelStr = inducedModelNode.InnerText.ToLower();
                p.inducedVelocityModel = modelStr switch
                {
                    "none" => InducedVelocityModelType.None,
                    "prandtl" => InducedVelocityModelType.Prandtl,
                    "momentum" => InducedVelocityModelType.Momentum,
                    "simplewake" or "simple_wake" => InducedVelocityModelType.SimpleWake,
                    _ => InducedVelocityModelType.Prandtl
                };
            }

            var taperNode = betNode.SelectSingleNode("taper_ratio");
            if (taperNode != null)
                p.taperRatio = TryParseFloat(taperNode.InnerText, p.taperRatio);

            var twistRootNode = betNode.SelectSingleNode("twist_root");
            if (twistRootNode != null)
                p.twistRoot = TryParseFloat(twistRootNode.InnerText, p.twistRoot);

            var twistTipNode = betNode.SelectSingleNode("twist_tip");
            if (twistTipNode != null)
                p.twistTip = TryParseFloat(twistTipNode.InnerText, p.twistTip);

            var sweepNode = betNode.SelectSingleNode("sweep_angle");
            if (sweepNode != null)
                p.sweepAngle = TryParseFloat(sweepNode.InnerText, p.sweepAngle);
        }

        // 非定常効果設定
        var unsteadyNode = node.SelectSingleNode("unsteady_effects");
        if (unsteadyNode != null)
        {
            // enable属性をチェック
            var enableAttr = unsteadyNode.Attributes?["enable"];
            if (enableAttr != null)
                p.enableUnsteadyEffects = enableAttr.Value.ToLower() == "true";

            var addedMassCoeffNode = unsteadyNode.SelectSingleNode("added_mass_coefficient");
            if (addedMassCoeffNode != null)
                p.addedMassCoefficient = TryParseFloat(addedMassCoeffNode.InnerText, p.addedMassCoefficient);

            var enableAddedMassNode = unsteadyNode.SelectSingleNode("enable_added_mass");
            if (enableAddedMassNode != null)
                p.enableAddedMass = enableAddedMassNode.InnerText.ToLower() == "true";

            var enableWagnerNode = unsteadyNode.SelectSingleNode("enable_wagner_lag");
            if (enableWagnerNode != null)
                p.enableWagnerLag = enableWagnerNode.InnerText.ToLower() == "true";

            var wagnerConstNode = unsteadyNode.SelectSingleNode("wagner_time_constant");
            if (wagnerConstNode != null)
                p.wagnerTimeConstant = TryParseFloat(wagnerConstNode.InnerText, p.wagnerTimeConstant);
        }

        return p;
    }

    /// <summary>
    /// AeroSurfaceコンポーネントにパラメータを設定する（リフレクションを使用）
    /// </summary>
    private void SetAeroSurfaceParameters(AeroSurface surface, AeroSurfaceParameters parameters)
    {
        var bindingFlags = System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance;
        var surfaceType = typeof(AeroSurface);

        // inlineParameters フィールドを取得して設定
        var inlineParamsField = surfaceType.GetField("inlineParameters", bindingFlags);
        if (inlineParamsField != null)
        {
            inlineParamsField.SetValue(surface, parameters);
        }
        else
        {
            Debug.LogWarning("[Aerodynamics] Could not set inline parameters via reflection");
        }
    }

    /// <summary>
    /// AeroSurfaceの制御面設定を適用する（リフレクションを使用）
    /// </summary>
    private void SetAeroSurfaceControlSettings(AeroSurface surface, XmlNode controlNode)
    {
        var bindingFlags = System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance;
        var surfaceType = typeof(AeroSurface);

        // isControlSurface を true に設定
        var isControlField = surfaceType.GetField("isControlSurface", bindingFlags);
        if (isControlField != null)
        {
            isControlField.SetValue(surface, true);
        }

        // inputType を設定
        var inputTypeField = surfaceType.GetField("inputType", bindingFlags);
        if (inputTypeField != null)
        {
            string typeStr = controlNode.Attributes?["type"]?.Value?.ToLower() ?? "none";
            ControlInputType inputType = typeStr switch
            {
                "pitch" => ControlInputType.Pitch,
                "roll" => ControlInputType.Roll,
                "yaw" => ControlInputType.Yaw,
                "flap" => ControlInputType.Flap,
                _ => ControlInputType.None
            };
            inputTypeField.SetValue(surface, inputType);
        }

        // inputMultiplier を設定
        var multiplierField = surfaceType.GetField("inputMultiplier", bindingFlags);
        if (multiplierField != null)
        {
            float multiplier = TryParseFloat(controlNode.Attributes?["multiplier"]?.Value, 1f);
            multiplierField.SetValue(surface, multiplier);
        }

        // invertSpanDirection を設定
        var invertSpanField = surfaceType.GetField("invertSpanDirection", bindingFlags);
        if (invertSpanField != null)
        {
            string invertStr = controlNode.Attributes?["invert_span_direction"]?.Value?.ToLower();
            if (!string.IsNullOrEmpty(invertStr))
            {
                bool invert = invertStr == "true" || invertStr == "1" || invertStr == "yes";
                invertSpanField.SetValue(surface, invert);
            }
        }
    }

    private static float TryParseFloat(string value)
    {
        return float.TryParse(value, out float result) ? result : 0f;
    }

    private static float TryParseFloat(string value, float defaultValue)
    {
        return float.TryParse(value, out float result) ? result : defaultValue;
    }

    private static Vector3 ParseVector3Attribute(XmlNode node, string attributeName, Vector3 defaultValue)
    {
        if (node?.Attributes?[attributeName] == null)
        {
            return defaultValue;
        }
        return ParseVector3(node.Attributes[attributeName].Value, defaultValue);
    }

    private static Vector3 ParseVector3(string value, Vector3 defaultValue)
    {
        if (string.IsNullOrWhiteSpace(value))
        {
            return defaultValue;
        }

        string[] parts = value.Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
        if (parts.Length < 3)
        {
            return defaultValue;
        }

        float x = TryParseFloat(parts[0], defaultValue.x);
        float y = TryParseFloat(parts[1], defaultValue.y);
        float z = TryParseFloat(parts[2], defaultValue.z);
        return new Vector3(-y, z, x);
    }

    /// <summary>
    /// スポーンするエンティティの名前を決める。
    /// </summary>
    /// <remarks>
    /// 仕様 (SpawnEntity.srv) では、要求名が空ならリソース側の名前 (URDF の
    /// robot 名) を使う。決まった名前が既存エンティティと衝突する場合、
    /// allow_renaming が false なら失敗、true なら連番を足して一意にする。
    ///
    /// 名前は topic 名の一部になるため、既存の launch を壊さないよう
    /// 「要求名が空なら URDF 名」という優先順位は仕様どおり守る
    /// (simulation_ros2_utils の robot_name の既定値も空文字にしてある)。
    /// </remarks>
    private bool TryResolveEntityName(string requestedName, string resourceName, bool allowRenaming, out string resolved)
    {
        string desired = string.IsNullOrEmpty(requestedName) ? resourceName : requestedName;
        resolved = desired;

        if (string.IsNullOrEmpty(desired))
        {
            return false;
        }
        if (!IsEntityNameTaken(desired))
        {
            return true;
        }
        if (!allowRenaming)
        {
            return false;
        }
        for (int i = 1; i < 1000; i++)
        {
            string candidate = desired + "_" + i;
            if (!IsEntityNameTaken(candidate))
            {
                resolved = candidate;
                return true;
            }
        }
        return false;
    }

    private bool IsEntityNameTaken(string name)
    {
        foreach (GameObject entity in m_EntityList)
        {
            if (entity != null && entity.name == name)
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>
    /// publisher 登録したトピックを Entity に紐づけて控える。引数の topic をそのまま返すので
    /// <c>pub.topicName = TrackPublishedTopic(name, "/foo");</c> の形で代入に挟んで使う。
    /// </summary>
    /// <summary>
    /// SpawnEntity の entity_namespace をトピック名へ適用する。
    /// </summary>
    /// <remarks>
    /// 仕様は「そのエンティティのインターフェースをすべてこの名前空間の下に置く」。
    /// URDF の ros2_control に書かれたトピック名 (/diffbot/joint_states など) は
    /// リソース側で固定なので、同じ URDF から 2 体スポーンすると名前空間なしでは
    /// トピックが衝突する。空文字なら何もしないので、既存の呼び出しには影響しない。
    /// </remarks>
    private static string ApplyNamespace(string entityNamespace, string topic)
    {
        if (string.IsNullOrEmpty(entityNamespace) || string.IsNullOrEmpty(topic))
        {
            return topic;
        }
        string ns = entityNamespace.Trim('/');
        if (ns.Length == 0)
        {
            return topic;
        }
        return "/" + ns + (topic.StartsWith("/") ? topic : "/" + topic);
    }

    private string TrackPublishedTopic(string entityName, string entityNamespace, string topic)
    {
        topic = ApplyNamespace(entityNamespace, topic);
        if (string.IsNullOrEmpty(topic))
        {
            return topic;
        }

        List<string> topics;
        if (!m_EntityPublishedTopics.TryGetValue(entityName, out topics))
        {
            topics = new List<string>();
            m_EntityPublishedTopics[entityName] = topics;
        }
        topics.Add(topic);

        int count;
        m_PublishedTopicRefCount.TryGetValue(topic, out count);
        m_PublishedTopicRefCount[topic] = count + 1;

        return topic;
    }

    /// <summary>
    /// Entity が使っていた publisher 登録を解除する。他の Entity がまだ同じトピックへ
    /// publish している場合は参照数を減らすだけで、実際の解除は最後の 1 つが消えたとき。
    /// </summary>
    private void ReleasePublishedTopics(string entityName)
    {
        List<string> topics;
        if (!m_EntityPublishedTopics.TryGetValue(entityName, out topics))
        {
            return;
        }
        m_EntityPublishedTopics.Remove(entityName);

        ROSConnection ros = ROSConnection.GetOrCreateInstance();
        foreach (string topic in topics)
        {
            int count;
            if (!m_PublishedTopicRefCount.TryGetValue(topic, out count))
            {
                continue;
            }
            count--;
            if (count > 0)
            {
                m_PublishedTopicRefCount[topic] = count;
                continue;
            }
            m_PublishedTopicRefCount.Remove(topic);
            // __remove_publisher をエンドポイントへ送る。エンドポイント側の対応が要る
            // (JointStateSub.DetachFromRos の注記を参照)。
            ros.UnregisterPublisher(topic);
        }
    }

    /// <summary>
    /// 配下の ArticulationBody の関節状態を URDF ゼロ姿勢・速度ゼロへ戻す。
    /// </summary>
    /// <remarks>
    /// スポーン直後と reset_simulation (SCOPE_STATE) の両方から呼ぶ。
    /// 位置・速度・関節力だけでなく xDrive の目標値も戻さないと、リセット直後に
    /// 「最後に受け取った指令」へ向かって動き出してしまい、開始時の状態にならない。
    /// </remarks>
    private void ResetArticulationState(GameObject root)
    {
        foreach (GameObject abObject in FindArticulationBodyObjectsInChildren(root))
        {
            ArticulationBody ab = abObject.GetComponent<ArticulationBody>();
            if (ab == null)
                continue;

            ab.linearVelocity = Vector3.zero;
            ab.angularVelocity = Vector3.zero;

            int dof = ab.dofCount;
            if (dof > 0)
            {
                var jp = ab.jointPosition;
                var jv = ab.jointVelocity;
                var jf = ab.jointForce;
                for (int d = 0; d < dof; d++)
                {
                    jp[d] = 0f;
                    jv[d] = 0f;
                    jf[d] = 0f;
                }
                ab.jointPosition = jp;
                ab.jointVelocity = jv;
                ab.jointForce = jf;

                // 駆動目標も初期状態 (停止・原点) に戻す。ここを残すと関節を
                // ゼロにした次の物理ステップで元の指令位置へ飛び戻る。
                ArticulationDrive drive = ab.xDrive;
                drive.target = 0f;
                drive.targetVelocity = 0f;
                ab.xDrive = drive;
            }

            // サーボモデルはロータ角や伝達ばねのたわみを内部に持つ。関節をゼロに
            // した「後」で戻さないと、巻き上がったトルクが直後に解放される。
            ServoJointModel servo = abObject.GetComponent<ServoJointModel>();
            if (servo != null)
            {
                servo.ResetState();
            }
        }
    }

    private void ResetAllEntitiesState()
    {
        foreach (GameObject entity in m_EntityList)
        {
            if (entity == null)
                continue;

            // スポーン時に記録した初期姿勢。エンティティ名をキーにしているので、
            // 取り出せない場合は現在の姿勢を維持して関節だけ戻す。
            Vector3 initialPosition;
            Quaternion initialRotation;
            bool hasPosition = m_EntityInitialPose.TryGetValue(entity.name, out initialPosition);
            bool hasRotation = m_EntityInitialRotation.TryGetValue(entity.name, out initialRotation);
            if (!hasPosition || !hasRotation)
            {
                Debug.LogWarning($"[ResetSimulation] '{entity.name}' の初期姿勢が記録されていない。関節状態のみ戻す");
                initialPosition = entity.transform.position;
                initialRotation = entity.transform.rotation;
            }

            // 関節状態を先に戻す。ルートをテレポートしてから関節をゼロにすると、
            // その 1 ステップ分だけ関節が動いた状態でソルバが回る。
            ResetArticulationState(entity);

            entity.transform.position = initialPosition;
            entity.transform.rotation = initialRotation;

            List<GameObject> childObjectsWithUrdfLink = GetChildObjectsWithComponent<UrdfLink>(entity);
            foreach (GameObject child in childObjectsWithUrdfLink)
            {
                UrdfLink link = child.GetComponent<UrdfLink>();
                link.IsBaseLink = true;

                ArticulationBody body = child.GetComponent<ArticulationBody>();
                if (body != null)
                {
                    body.TeleportRoot(initialPosition, initialRotation);
                    // TeleportRoot はルートの速度を消さない。残すとテレポート直後に
                    // そのまま走り出して初期位置から離れてしまう。
                    body.linearVelocity = Vector3.zero;
                    body.angularVelocity = Vector3.zero;
                    body.PublishTransform();
                }
                break;
            }
        }
    }

    /// <summary>
    /// Entity を 1 体デスポーンする。m_EntityList からの除去は呼び出し側の責任。
    /// </summary>
    /// <remarks>
    /// delete_entity と DespawnAllEntities の共通実装。
    /// </remarks>
    private void DespawnEntity(GameObject entity)
    {
        if (entity == null)
            return;

        // まず非アクティブにして、この Entity 配下の Update / FixedUpdate を止める。
        // Destroy はフレーム終端まで遅延されるので、これをやらないと
        // UnitySensors の RosMsgPublisher.Update() が「未登録なら登録する」
        // 遅延登録を行い、下で解除した publisher をその場で登録し直してしまう。
        entity.SetActive(false);

        // Destroy より前に ROS の受信経路から切り離す。Unity の Destroy は
        // フレーム終端まで遅延されるため、OnDestroy 任せにすると破棄済みの
        // コールバックが 1 フレーム分生き残って例外を投げ、同じトピックの
        // 後続コールバック (再スポーンしたロボット) まで止めてしまう。
        foreach (JointStateSub sub in entity.GetComponentsInChildren<JointStateSub>(true))
        {
            sub.DetachFromRos();
        }
        foreach (LinkThruster thruster in entity.GetComponentsInChildren<LinkThruster>(true))
        {
            thruster.DetachFromRos();
        }
        // publisher 側も解除する。こちらは購読と違いコンポーネントに解除 API が無い
        // (センサ系は UnitySensors のクラス) ため、スポーン時に控えたトピック名で外す。
        ReleasePublishedTopics(entity.name);

        m_EntityInitialPose.Remove(entity.name);
        m_EntityInitialRotation.Remove(entity.name);
        m_EntityInfo.Remove(entity.name);

        GameObject.Destroy(entity);
    }

    private void DespawnAllEntities()
    {
        foreach (GameObject entity in m_EntityList)
        {
            DespawnEntity(entity);
        }
        m_EntityList.Clear();
        m_EntityInitialPose.Clear();
        m_EntityInitialRotation.Clear();
        m_EntityInfo.Clear();
    }
}
