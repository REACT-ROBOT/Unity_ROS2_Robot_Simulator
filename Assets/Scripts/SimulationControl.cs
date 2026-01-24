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
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

using UnityMeshImporter;
using NaughtyWaterBuoyancy;

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

        // Buoyancy Material の収集と ArticulationFloatingObject の付与
        Dictionary<string, float> buoyancyMaterialDict = new Dictionary<string, float>();
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
        }

        // 全リンクに ArticulationFloatingObject を付与
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
                            // ArticulationFloatingObjectを追加
                            ArticulationFloatingObject floatingObj = colliderObject.GetComponent<ArticulationFloatingObject>();
                            if (floatingObj == null)
                            {
                                floatingObj = colliderObject.AddComponent<ArticulationFloatingObject>();
                            }

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

                            floatingObj.Density = density;
                            Debug.Log($"Added ArticulationFloatingObject to {linkName} with density {density}");
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
                                    laserScanMsgPublisher.topicName = "/" + robotObject.name + "/" + sensorLinkName + "/scan";
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
                                    lidarPointCloud2MsgPublisher.topicName = "/" + robotObject.name + "/" + sensorLinkName + "/scan";
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
                                cameraInfoPublisher.topicName = "/" + robotObject.name + "/" + sensorLinkName + "/camera_info";

                                var cameraImageHeader = new HeaderSerializer();
                                cameraImageHeader.Configure(cameraSensor, sensorLinkName);
                                var cameraImageSerializer = new ImageMsgSerializer();
                                cameraImageSerializer.Configure(cameraSensor, cameraImageHeader, 0, 0); // Texture0, RGB8
                                cameraImagePublisher.serializer = cameraImageSerializer;
                                cameraImagePublisher.topicName = "/" + robotObject.name + "/" + sensorLinkName + "/image_raw";
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
                                fisheyeCameraInfoPublisher.topicName = "/" + robotObject.name + "/" + sensorLinkName + "/fisheye_camera_info";

                                var fisheyeCameraImageHeader = new HeaderSerializer();
                                fisheyeCameraImageHeader.Configure(fisheyeCameraSensor, sensorLinkName);
                                var fisheyeCameraImageSerializer = new ImageMsgSerializer();
                                fisheyeCameraImageSerializer.Configure(fisheyeCameraSensor, fisheyeCameraImageHeader, 0, 0); // Texture0, RGB8
                                fisheyeCameraImagePublisher.serializer = fisheyeCameraImageSerializer;
                                fisheyeCameraImagePublisher.topicName = "/" + robotObject.name + "/" + sensorLinkName + "/fisheye_image_raw";
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
                                panoramicCameraInfoPublisher.topicName = "/" + robotObject.name + "/" + sensorLinkName + "/panoramic_camera_info";

                                var panoramicCameraImageHeader = new HeaderSerializer();
                                panoramicCameraImageHeader.Configure(panoramicCameraSensor, sensorLinkName);
                                var panoramicCameraImageSerializer = new CompressedImageMsgSerializer();
                                panoramicCameraImageSerializer.Configure(panoramicCameraSensor, panoramicCameraImageHeader, 0); // Texture0
                                panoramicCameraImagePublisher.serializer = panoramicCameraImageSerializer;
                                panoramicCameraImagePublisher.topicName = "/" + robotObject.name + "/" + sensorLinkName + "/panoramic_image_raw";
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
                                depthCameraInfoPublisher.topicName = "/" + robotObject.name + "/" + sensorLinkName + "/depth_camera_info";

                                var depthCameraImageHeader = new HeaderSerializer();
                                depthCameraImageHeader.Configure(depthCameraSensor, sensorLinkName);
                                var depthCameraImageSerializer = new ImageMsgSerializer();
                                depthCameraImageSerializer.Configure(depthCameraSensor, depthCameraImageHeader, 0, 1); // Texture0, 32FC1
                                depthCameraImagePublisher.serializer = depthCameraImageSerializer;
                                depthCameraImagePublisher.topicName = "/" + robotObject.name + "/" + sensorLinkName + "/depth_image_raw";
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
                                rgbdDepthCameraInfoPublisher.topicName = "/" + robotObject.name + "/" + sensorLinkName + "/depth/camera_info";

                                // Configure depth image publisher
                                var rgbdDepthImageHeader = new HeaderSerializer();
                                rgbdDepthImageHeader.Configure(rgbdCameraSensor, sensorLinkName);
                                var rgbdDepthImageSerializer = new ImageMsgSerializer();
                                rgbdDepthImageSerializer.Configure(rgbdCameraSensor, rgbdDepthImageHeader, 0, 1); // Texture0 (depth), 32FC1
                                rgbdDepthImagePublisher.serializer = rgbdDepthImageSerializer;
                                rgbdDepthImagePublisher.topicName = "/" + robotObject.name + "/" + sensorLinkName + "/depth/image_raw";

                                // Configure color camera info publisher
                                var rgbdColorCameraInfoHeader = new HeaderSerializer();
                                rgbdColorCameraInfoHeader.Configure(rgbdCameraSensor, sensorLinkName);
                                var rgbdColorCameraInfoSerializer = new CameraInfoMsgSerializer();
                                rgbdColorCameraInfoSerializer.Configure(rgbdCameraSensor, rgbdColorCameraInfoHeader);
                                rgbdColorCameraInfoPublisher.serializer = rgbdColorCameraInfoSerializer;
                                rgbdColorCameraInfoPublisher.topicName = "/" + robotObject.name + "/" + sensorLinkName + "/color/camera_info";

                                // Configure color image publisher
                                var rgbdColorImageHeader = new HeaderSerializer();
                                rgbdColorImageHeader.Configure(rgbdCameraSensor, sensorLinkName);
                                var rgbdColorImageSerializer = new ImageMsgSerializer();
                                rgbdColorImageSerializer.Configure(rgbdCameraSensor, rgbdColorImageHeader, 1, 0); // Texture1 (color), RGB8
                                rgbdColorImagePublisher.serializer = rgbdColorImageSerializer;
                                rgbdColorImagePublisher.topicName = "/" + robotObject.name + "/" + sensorLinkName + "/color/image_raw";
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
                                imuMsgPublisher.topicName = "/" + robotObject.name + "/" + sensorLinkName + "/imu";
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
                    XmlNode stiffnessNode = jointNode.SelectSingleNode("stiffness");
                    XmlNode dampingNode = jointNode.SelectSingleNode("damping");
                    
                    if (stiffnessNode != null)
                    {
                        parameters["stiffness"] = TryParseFloat(stiffnessNode.InnerText);
                    }
                    else
                    {
                        Debug.Log("Joint stiffness element not found.");
                        parameters["stiffness"] = 0.0f;
                    }
                    if (dampingNode != null)
                    {
                        parameters["damping"] = TryParseFloat(dampingNode.InnerText);
                    }
                    else
                    {
                        Debug.Log("Joint damping element not found.");
                        parameters["damping"] = 0.0f;
                    }
                    if (stiffnessNode != null || dampingNode != null)
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

    private static float TryParseFloat(string value)
    {
        return float.TryParse(value, out float result) ? result : 0f;
    }

    private static float TryParseFloat(string value, float defaultValue)
    {
        return float.TryParse(value, out float result) ? result : defaultValue;
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
