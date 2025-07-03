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
using UnitySensors.DataType.LiDAR;
using UnitySensors.ROS.Publisher.Camera;
using UnitySensors.ROS.Publisher.Sensor;
using UnitySensors.ROS.Serializer.Sensor;
using UnitySensors.ROS.Serializer.Std;
using UnitySensors.ROS.Serializer.PointCloud;

using RosMessageTypes.SimulationInterfaces;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

using UnityMeshImporter;

public class FileLogger
{
    private static string logFilePath = "debug_log.txt";

    public static void Log(string message)
    {
        File.AppendAllText(logFilePath, message + "\n");
    }
}

// 保存するためのXDrive設定用クラス
[Serializable]
public class XDriveSettings
{
    public string joint_name;
    public float stiffness;
    public float damping;
    public float forceLimit;

    public XDriveSettings(string joint_name, float stiffness, float damping, float forceLimit)
    {
        this.joint_name = joint_name;
        this.stiffness = stiffness;
        this.damping = damping;
        this.forceLimit = forceLimit;
    }
}

public class SimulationControl : MonoBehaviour
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

        m_SimulationState = SimulationStateMsg.STATE_STOPPED;
        Time.timeScale = 0f;
    }

    private SetSimulationStateResponse SetSimulationState(SetSimulationStateRequest request)
    {
        var response = new SetSimulationStateResponse();

        if (request.state.state == m_SimulationState)
        {
            response.result.result = SetSimulationStateResponse.ALREADY_IN_TARGET_STATE;
            response.result.error_message = "Already in requested state";
            return response;
        }

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
        response.state.state = m_SimulationState;
        return response;
    }

    private ResetSimulationResponse ResetSimulation(ResetSimulationRequest request)
    {
        var response = new ResetSimulationResponse();

        m_SimulationState = SimulationStateMsg.STATE_STOPPED;
        Time.timeScale = 0f;

        if (request.scope == ResetSimulationRequest.SCOPE_DEFAULT)
        {
            request.scope = ResetSimulationRequest.SCOPE_ALL;
        }

        if ((request.scope & ResetSimulationRequest.SCOPE_TIME) == ResetSimulationRequest.SCOPE_TIME)
        {
            // TODO: Reset Time
        }
        if ((request.scope & ResetSimulationRequest.SCOPE_STATE) == ResetSimulationRequest.SCOPE_STATE)
        {
            ResetAllEntitiesState();
        }
        if ((request.scope & ResetSimulationRequest.SCOPE_SPAWNED) == ResetSimulationRequest.SCOPE_SPAWNED)
        {
            DespawnAllEntities();
        }

        return response;
    }

    private StepSimulationResponse StepSimulation(StepSimulationRequest request)
    {
        var response = new StepSimulationResponse();

        // TODO: implement the logic to step the simulation
        // For now, we just return an error

        response.result.result = ResultMsg.RESULT_OPERATION_FAILED;
        response.result.error_message = "Step simulation not implemented";
        return response;
    }

    /// <summary>
    ///  Callback to respond to the request
    /// </summary>
    /// <param name="request">service request containing the object name</param>
    /// <returns>service response containing the object pose (or 0 if object not found)</returns>
    private SpawnEntityResponse SpawnEntity(SpawnEntityRequest request)
    {
        // prepare a response
        SpawnEntityResponse spawnEntityResponse = new SpawnEntityResponse();

        // process the service request
        Debug.Log("Received request for object: " + request.name);

        string filePath;
        Uri uri = new Uri(request.uri);
        if (uri.IsFile)
        {
            filePath = uri.LocalPath;
        }
        else
        {
            Debug.LogError("Invalid URI: " + request.uri);
            spawnEntityResponse.result.result = SpawnEntityResponse.RESOURCE_PARSE_ERROR;
            spawnEntityResponse.result.error_message = "Invalid URI: " + request.uri;
            return spawnEntityResponse;
        }
        double robot_x = request.initial_pose.pose.position.x;
        double robot_y = request.initial_pose.pose.position.y;
        double robot_z = request.initial_pose.pose.position.z;
        double q_x = request.initial_pose.pose.orientation.x;
        double q_y = request.initial_pose.pose.orientation.y;
        double q_z = request.initial_pose.pose.orientation.z;
        double q_w = request.initial_pose.pose.orientation.w;

        Debug.Log("Received path: " + filePath);

        if (!filePath.EndsWith(".urdf"))
        {
            var ob = MeshImporter.Load(filePath);
            if (ob == null)
            {
                Debug.LogError("Failed to load object from File.");
                spawnEntityResponse.result.result = SpawnEntityResponse.RESOURCE_PARSE_ERROR;
                spawnEntityResponse.result.error_message = "Failed to load object from File.";
                return spawnEntityResponse;
            }
            ob.name = request.name;

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
            
            return spawnEntityResponse;
        }

        ImportSettings settings = new ImportSettings();
        GameObject robotObject = UrdfRobotExtensions.CreateRuntime(filePath, settings);

        if (robotObject == null)
        {
            Debug.LogError("Failed to load robot from URDF.");
            spawnEntityResponse.result.result = SpawnEntityResponse.RESOURCE_PARSE_ERROR;
            spawnEntityResponse.result.error_message = "Failed to load robot from URDF.";
            return spawnEntityResponse;
        }
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

                var parameters = GetUnityDriveApiParameters(xmlDoc, urdfJoint.jointName);
                ArticulationDrive drive = body.xDrive;
                drive.stiffness = parameters["stiffness"];
                drive.damping = parameters["damping"];
                drive.forceLimit = parameters["force_limit"];
                body.xDrive = drive;
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

        jointStateSub.articulationBodies = articulationBodyList.ToArray();
        jointStateSub.jointName = jointNameList.ToArray();
        jointStateSub.jointLength = articulationBodyList.Count;
        XmlNode jointCommandParam = xmlDoc.SelectSingleNode("//robot/ros2_control/hardware/param[@name='joint_commands_topic']");
        if (jointCommandParam != null)
        {
            jointStateSub.topicName = jointCommandParam.InnerText;
        }

        groundTruthPub.targetObject = childObjectsWithUrdfLink[0];
        XmlNode groundTruthParam = xmlDoc.SelectSingleNode("//robot/ros2_control/hardware/param[@name='ground_truth_topic']");
        if (groundTruthParam != null)
        {
            groundTruthPub.topicName = groundTruthParam.InnerText;
        }

        // Physics Material の生成（ランタイムでは AssetDatabase は使用不可のため new で生成）
        string directoryPath = Path.GetDirectoryName(filePath);
        int assetsIndex = directoryPath.IndexOf("Assets");
        if (assetsIndex >= 0)
        {
            directoryPath = directoryPath.Substring(assetsIndex);
        }
        XmlNode robotNode = xmlDoc.SelectSingleNode("/robot");
        List<PhysicsMaterial> physicsMaterialList = new List<PhysicsMaterial>();
        if (robotNode != null)
        {
            XmlNodeList physicsMaterials = robotNode.SelectNodes("collsion_material");
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
                                        Debug.Log(materialName + ": " + linkName);
                                        Collider meshCollider = targetCollision.gameObject.GetComponent<Collider>();
                                        if (meshCollider != null)
                                        {
                                            foreach (PhysicsMaterial material in physicsMaterialList)
                                            {
                                                if (material.name == materialName)
                                                {
                                                    meshCollider.material = material;
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
                                var minRangeField = typeof(LiDARSensor).GetField("_minRange", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                minRangeField.SetValue(lidarSensor, TryParseFloat(sensor.SelectSingleNode("ray/range/min").InnerText));
                                var maxRangeField = typeof(LiDARSensor).GetField("_maxRange", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                maxRangeField.SetValue(lidarSensor, TryParseFloat(sensor.SelectSingleNode("ray/range/max").InnerText));
                                var gaussianNoiseSigmaField = typeof(LiDARSensor).GetField("_gaussianNoiseSigma", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                gaussianNoiseSigmaField.SetValue(lidarSensor, TryParseFloat(sensor.SelectSingleNode("ray/noise/stddev").InnerText));
                                if (sensor.SelectSingleNode("ray/scan/vertical") == null)
                                {
                                    var pointsNumPerScanField = typeof(LiDARSensor).GetField("_pointsNumPerScan", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                    pointsNumPerScanField.SetValue(lidarSensor, int.Parse(sensor.SelectSingleNode("ray/scan/horizontal/samples").InnerText));
                                    var scanPattern = ScriptableObject.CreateInstance<ScanPattern>();
                                    scanPattern.size = int.Parse(sensor.SelectSingleNode("ray/scan/horizontal/samples").InnerText);
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
                                    var scanPatternField = typeof(LiDARSensor).GetField("_scanPattern", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                    scanPatternField.SetValue(lidarSensor, scanPattern);
                                    lidarSensor.Initialize();
                                    LaserScanMsgPublisher laserScanMsgPublisher = targetObject.AddComponent<LaserScanMsgPublisher>();
                                    var laserScanMsgPublisherSerializerField = laserScanMsgPublisher.GetType().GetField("_serializer", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                    if (laserScanMsgPublisherSerializerField != null)
                                    {
                                        var laserScanMsgPublisherSerializer = new LaserScanMsgSerializer();
                                        laserScanMsgPublisherSerializer.SetSource(lidarSensor);
                                        var headerField = laserScanMsgPublisherSerializer.GetType().GetField("_header", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                        headerField.SetValue(laserScanMsgPublisherSerializer, new HeaderSerializer());
                                        var header = headerField.GetValue(laserScanMsgPublisherSerializer);
                                        var headerSourceField = header.GetType().GetField("_source", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                        headerSourceField.SetValue(header, lidarSensor);
                                        var headerFrameIdField = header.GetType().GetField("_frame_id", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                        headerFrameIdField.SetValue(header, sensorLinkName);
                                        var serializerMinRangeField = laserScanMsgPublisherSerializer.GetType().GetField("_minRange", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                        serializerMinRangeField.SetValue(laserScanMsgPublisherSerializer, TryParseFloat(sensor.SelectSingleNode("ray/range/min").InnerText));
                                        var serializerMaxRangeField = laserScanMsgPublisherSerializer.GetType().GetField("_maxRange", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                        serializerMaxRangeField.SetValue(laserScanMsgPublisherSerializer, TryParseFloat(sensor.SelectSingleNode("ray/range/max").InnerText));
                                        var serializerGaussianNoiseSigmaField = laserScanMsgPublisherSerializer.GetType().GetField("_gaussianNoiseSigma", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                        serializerGaussianNoiseSigmaField.SetValue(laserScanMsgPublisherSerializer, TryParseFloat(sensor.SelectSingleNode("ray/noise/stddev").InnerText));
                                        var serializerScanPatternField = laserScanMsgPublisherSerializer.GetType().GetField("_scanPattern", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                        serializerScanPatternField.SetValue(laserScanMsgPublisherSerializer, scanPattern);
                                        laserScanMsgPublisherSerializerField.SetValue(laserScanMsgPublisher, laserScanMsgPublisherSerializer);
                                    }
                                    var laserScanMsgPublisherTopicNameField = laserScanMsgPublisher.GetType().GetField("_topicName", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                    laserScanMsgPublisherTopicNameField.SetValue(laserScanMsgPublisher, "/" + robotObject.name + "/" + sensorLinkName + "/scan");
                                }
                                else
                                {
                                    int verticalSamples = int.Parse(sensor.SelectSingleNode("ray/scan/vertical/samples").InnerText);
                                    int horizontalSamples = int.Parse(sensor.SelectSingleNode("ray/scan/horizontal/samples").InnerText);
                                    var pointsNumPerScanField = typeof(LiDARSensor).GetField("_pointsNumPerScan", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                    pointsNumPerScanField.SetValue(lidarSensor, horizontalSamples * verticalSamples);
                                    var scanPattern = ScriptableObject.CreateInstance<ScanPattern>();
                                    scanPattern.size = horizontalSamples * verticalSamples;
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
                                    var scanPatternField = typeof(LiDARSensor).GetField("_scanPattern", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                    scanPatternField.SetValue(lidarSensor, scanPattern);
                                    lidarSensor.Initialize();
                                    LiDARPointCloud2MsgPublisher lidarPointCloud2MsgPublisher = targetObject.AddComponent<LiDARPointCloud2MsgPublisher>();
                                    var lidarPointCloud2MsgPublisherSerializerField = lidarPointCloud2MsgPublisher.GetType().GetField("_serializer", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                    if (lidarPointCloud2MsgPublisherSerializerField != null)
                                    {
                                        var lidarPointCloud2MsgPublisherSerializer = new PointCloud2MsgSerializer<UnitySensors.DataType.Sensor.PointCloud.PointXYZI>();
                                        lidarPointCloud2MsgPublisherSerializer.SetSource(lidarSensor);
                                        var headerField = lidarPointCloud2MsgPublisherSerializer.GetType().GetField("_header", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                        headerField.SetValue(lidarPointCloud2MsgPublisherSerializer, new HeaderSerializer());
                                        var header = headerField.GetValue(lidarPointCloud2MsgPublisherSerializer);
                                        var headerSourceField = header.GetType().GetField("_source", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                        headerSourceField.SetValue(header, lidarSensor);
                                        var headerFrameIdField = header.GetType().GetField("_frame_id", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                        headerFrameIdField.SetValue(header, sensorLinkName);
                                        lidarPointCloud2MsgPublisherSerializerField.SetValue(lidarPointCloud2MsgPublisher, lidarPointCloud2MsgPublisherSerializer);
                                    }
                                    var lidarPointCloud2MsgPublisherTopicNameField = lidarPointCloud2MsgPublisher.GetType().GetField("_topicName", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                    lidarPointCloud2MsgPublisherTopicNameField.SetValue(lidarPointCloud2MsgPublisher, "/" + robotObject.name + "/" + sensorLinkName + "/scan");
                                }
                                break;
                            case "camera":
                                Debug.Log("sensor type 'camera' found");
                                RGBCameraSensor cameraSensor = targetObject.AddComponent<RGBCameraSensor>();
                                var fovField = cameraSensor.GetType().GetField("_fov", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                fovField.SetValue(cameraSensor, TryParseFloat(sensor.SelectSingleNode("horizontal_fov").InnerText) * 180.0f / 3.14f);
                                var resolutionField = cameraSensor.GetType().GetField("_resolution", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                int image_width, image_height;
                                int.TryParse(sensor.SelectSingleNode("image/width").InnerText, out image_width);
                                int.TryParse(sensor.SelectSingleNode("image/height").InnerText, out image_height);
                                resolutionField.SetValue(cameraSensor, new Vector2Int(image_width, image_height));
                                UnityEngine.Camera cameraComponent = targetObject.GetComponent<UnityEngine.Camera>();
                                if (cameraComponent != null)
                                {
                                    cameraComponent.targetDisplay = next_display_number;
                                    next_display_number++;
                                }
                                CameraInfoMsgPublisher cameraInfoPublisher = targetObject.AddComponent<CameraInfoMsgPublisher>();
                                CompressedImageMsgPublisher cameraImagePublisher = targetObject.AddComponent<CompressedImageMsgPublisher>();
                                var cameraInfoPublisherSerializerField = cameraInfoPublisher.GetType().GetField("_serializer", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                if (cameraInfoPublisherSerializerField != null)
                                {
                                    var cameraInfoPublisherSerializer = new CameraInfoMsgSerializer();
                                    var sourceField = cameraInfoPublisherSerializer.GetType().GetField("_source", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                    sourceField.SetValue(cameraInfoPublisherSerializer, cameraSensor);
                                    var headerField = cameraInfoPublisherSerializer.GetType().GetField("_header", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                    headerField.SetValue(cameraInfoPublisherSerializer, new HeaderSerializer());
                                    var header = headerField.GetValue(cameraInfoPublisherSerializer);
                                    var headerSourceField = header.GetType().GetField("_source", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                    headerSourceField.SetValue(header, cameraSensor);
                                    var headerFrameIdField = header.GetType().GetField("_frame_id", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                    headerFrameIdField.SetValue(header, sensorLinkName);
                                    cameraInfoPublisherSerializerField.SetValue(cameraInfoPublisher, cameraInfoPublisherSerializer);
                                }
                                var topicNameField = cameraInfoPublisher.GetType().GetField("_topicName", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                topicNameField.SetValue(cameraInfoPublisher, "/" + robotObject.name + "/" + sensorLinkName + "/camera_info");
                                var cameraImagePublisherSerializerField = cameraImagePublisher.GetType().GetField("_serializer", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                if (cameraImagePublisherSerializerField != null)
                                {
                                    var cameraImagePublisherSerializer = new CompressedImageMsgSerializer();
                                    var sourceField = cameraImagePublisherSerializer.GetType().GetField("_source", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                    sourceField.SetValue(cameraImagePublisherSerializer, cameraSensor);
                                    var headerField = cameraImagePublisherSerializer.GetType().GetField("_header", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                    headerField.SetValue(cameraImagePublisherSerializer, new HeaderSerializer());
                                    var header = headerField.GetValue(cameraImagePublisherSerializer);
                                    var headerSourceField = header.GetType().GetField("_source", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                    headerSourceField.SetValue(header, cameraSensor);
                                    var headerFrameIdField = header.GetType().GetField("_frame_id", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                    headerFrameIdField.SetValue(header, sensorLinkName);
                                    cameraImagePublisherSerializerField.SetValue(cameraImagePublisher, cameraImagePublisherSerializer);
                                }
                                var topicNameField2 = cameraImagePublisher.GetType().GetField("_topicName", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                topicNameField2.SetValue(cameraImagePublisher, "/" + robotObject.name + "/" + sensorLinkName + "/image_raw");
                                break;
                            case "depth_camera":
                                Debug.Log("sensor type 'depth_camera' found");
                                DepthCameraSensor depthCameraSensor = targetObject.AddComponent<DepthCameraSensor>();
                                var depthFovField = depthCameraSensor.GetType().GetField("_fov", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                depthFovField.SetValue(depthCameraSensor, TryParseFloat(sensor.SelectSingleNode("horizontal_fov").InnerText) * 180.0f / 3.14f);
                                var depthResolutionField = depthCameraSensor.GetType().GetField("_resolution", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                int depth_image_width, depth_image_height;
                                int.TryParse(sensor.SelectSingleNode("image/width").InnerText, out depth_image_width);
                                int.TryParse(sensor.SelectSingleNode("image/height").InnerText, out depth_image_height);
                                depthResolutionField.SetValue(depthCameraSensor, new Vector2Int(depth_image_width, depth_image_height));
                                UnityEngine.Camera depthCameraComponent = targetObject.GetComponent<UnityEngine.Camera>();
                                if (depthCameraComponent != null)
                                {
                                    depthCameraComponent.targetDisplay = next_display_number;
                                    next_display_number++;
                                }
                                CameraInfoMsgPublisher depthCameraInfoPublisher = targetObject.AddComponent<CameraInfoMsgPublisher>();
                                CompressedImageMsgPublisher depthCameraImagePublisher = targetObject.AddComponent<CompressedImageMsgPublisher>();
                                var depthCameraInfoPublisherSerializerField = depthCameraInfoPublisher.GetType().GetField("_serializer", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                if (depthCameraInfoPublisherSerializerField != null)
                                {
                                    var depthCameraInfoPublisherSerializer = new CameraInfoMsgSerializer();
                                    var sourceField = depthCameraInfoPublisherSerializer.GetType().GetField("_source", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                    sourceField.SetValue(depthCameraInfoPublisherSerializer, depthCameraSensor);
                                    var headerField = depthCameraInfoPublisherSerializer.GetType().GetField("_header", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                    headerField.SetValue(depthCameraInfoPublisherSerializer, new HeaderSerializer());
                                    var header = headerField.GetValue(depthCameraInfoPublisherSerializer);
                                    var headerSourceField = header.GetType().GetField("_source", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                    headerSourceField.SetValue(header, depthCameraSensor);
                                    var headerFrameIdField = header.GetType().GetField("_frame_id", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                    headerFrameIdField.SetValue(header, sensorLinkName);
                                    depthCameraInfoPublisherSerializerField.SetValue(depthCameraInfoPublisher, depthCameraInfoPublisherSerializer);
                                }
                                var depthTopicNameField = depthCameraInfoPublisher.GetType().GetField("_topicName", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                depthTopicNameField.SetValue(depthCameraInfoPublisher, "/" + robotObject.name + "/" + sensorLinkName + "/depth_camera_info");
                                var depthCameraImagePublisherSerializerField = depthCameraImagePublisher.GetType().GetField("_serializer", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                if (depthCameraImagePublisherSerializerField != null)
                                {
                                    var depthCameraImagePublisherSerializer = new CompressedImageMsgSerializer();
                                    var sourceField = depthCameraImagePublisherSerializer.GetType().GetField("_source", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                    sourceField.SetValue(depthCameraImagePublisherSerializer, depthCameraSensor);
                                    var headerField = depthCameraImagePublisherSerializer.GetType().GetField("_header", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                    headerField.SetValue(depthCameraImagePublisherSerializer, new HeaderSerializer());
                                    var header = headerField.GetValue(depthCameraImagePublisherSerializer);
                                    var headerSourceField = header.GetType().GetField("_source", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                    headerSourceField.SetValue(header, depthCameraSensor);
                                    var headerFrameIdField = header.GetType().GetField("_frame_id", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                    headerFrameIdField.SetValue(header, sensorLinkName);
                                    depthCameraImagePublisherSerializerField.SetValue(depthCameraImagePublisher, depthCameraImagePublisherSerializer);
                                }
                                var depthTopicNameField2 = depthCameraImagePublisher.GetType().GetField("_topicName", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                depthTopicNameField2.SetValue(depthCameraImagePublisher, "/" + robotObject.name + "/" + sensorLinkName + "/depth_image_raw");
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

        // Find a game object with the requested name
        GameObject gameObject = GameObject.Find(request.name);

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
    /// URDF 内の joint 要素から unity_drive_api の各パラメータを抽出します。
    /// </summary>
    public static Dictionary<string, float> GetUnityDriveApiParameters(XmlDocument xmlDoc, string targetJointName)
    {
        var parameters = new Dictionary<string, float>();

        XmlNode robotNode = xmlDoc.SelectSingleNode("/robot");
        if (robotNode != null)
        {
            XmlNodeList jointNodes = robotNode.SelectNodes("joint");
            foreach (XmlNode jointNode in jointNodes)
            {
                if (jointNode.Attributes["name"]?.Value == targetJointName)
                {
                    if (jointNode.Attributes["stiffness"]?.Value != null)
                    {
                        parameters["stiffness"] = TryParseFloat(jointNode.Attributes["stiffness"].Value);
                    }
                    else
                    {
                        Debug.Log("Joint stiffness attribute not found.");
                        parameters["stiffness"] = 0.0f;
                    }
                    if (jointNode.Attributes["damping"]?.Value != null)
                    {
                        parameters["damping"] = TryParseFloat(jointNode.Attributes["damping"].Value);
                    }
                    else
                    {
                        Debug.Log("Joint damping attribute not found.");
                        parameters["damping"] = 0.0f;
                    }
                    if (jointNode.Attributes["stiffness"]?.Value != null || jointNode.Attributes["damping"]?.Value != null)
                    {
                        parameters["force_limit"] = 1000000.0f; // デフォルトの力制限値
                    }
                    else
                    {
                        XmlNode unityDriveApiNode = jointNode.SelectSingleNode("unity_drive_api");
                        if (unityDriveApiNode != null)
                        {
                            parameters["stiffness"] = TryParseFloat(unityDriveApiNode.Attributes["stiffness"]?.Value);
                            parameters["damping"] = TryParseFloat(unityDriveApiNode.Attributes["damping"]?.Value);
                            parameters["force_limit"] = TryParseFloat(unityDriveApiNode.Attributes["force_limit"]?.Value);
                        }
                        else
                        {
                            Debug.Log("unity_drive_api element not found.");
                            parameters["stiffness"] = 0.0f;
                            parameters["damping"] = 0.0f;
                            parameters["force_limit"] = 0.0f;
                        }
                    }
                    return parameters;
                }
            }
            Debug.Log("Joint with the specified name not found.");
        }
        else
        {
            Debug.Log("Robot element not found.");
        }

        parameters["stiffness"] = 0.0f;
        parameters["damping"] = 0.0f;
        parameters["force_limit"] = 0.0f;
        return parameters;
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

    private static float TryParseFloat(string value)
    {
        return float.TryParse(value, out float result) ? result : 0f;
    }

    private void ResetAllEntitiesState()
    {
        foreach (GameObject entity in m_EntityList)
        {
            if (entity != null)
            {
                entity.transform.position = m_EntityInitialPose[entity.name];
                entity.transform.rotation = m_EntityInitialRotation[entity.name];

                List<GameObject> childObjectsWithUrdfLink = GetChildObjectsWithComponent<UrdfLink>(entity);
                foreach (GameObject child in childObjectsWithUrdfLink)
                {
                    UrdfLink link = child.GetComponent<UrdfLink>();
                    link.IsBaseLink = true;

                    ArticulationBody body = child.GetComponent<ArticulationBody>();
                    if (body != null)
                    {
                        body.TeleportRoot(m_EntityInitialPose[entity.name], m_EntityInitialRotation[entity.name]);
                        body.PublishTransform();
                    }
                    break;
                }
            }
        }
    }

    private void DespawnAllEntities()
    {
        foreach (GameObject entity in m_EntityList)
        {
            if (entity != null)
            {
                GameObject.Destroy(entity);
            }
        }
        m_EntityList.Clear();
        m_EntityInitialPose.Clear();
        m_EntityInitialRotation.Clear();
    }
}
