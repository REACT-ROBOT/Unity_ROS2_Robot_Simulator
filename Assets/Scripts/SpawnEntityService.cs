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
using UnityEngine.SceneManagement;
using Unity.Robotics.UrdfImporter;
using Unity.Robotics.UrdfImporter.Control;
//using UnitySensors.Sensor.Camera;
//using UnitySensors.Sensor.LiDAR;
//using UnitySensors.ROS.Publisher.Camera;
//using UnitySensors.ROS.Publisher.Sensor;
//using UnitySensors.ROS.Serializer.Sensor;

using RosMessageTypes.SimulationInterfaces;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

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

/// <summary>
/// Example demonstration of implementing a UnityService that receives a Request message from another ROS node and sends a Response back
/// </summary>
public class SpawnEntityService : MonoBehaviour
{
    [SerializeField]
    string m_ServiceName = "spawn_entity";

    void Start()
    {
        // register the service with ROS
        ROSConnection.GetOrCreateInstance().ImplementService<SpawnEntityRequest, SpawnEntityResponse>(m_ServiceName, SpawnEntity);
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

        string urdfFilePath = request.uri;
        double robot_x = request.initial_pose.pose.position.x;
        double robot_y = request.initial_pose.pose.position.y;
        double robot_z = request.initial_pose.pose.position.z;
        double q_x = request.initial_pose.pose.orientation.x;
        double q_y = request.initial_pose.pose.orientation.y;
        double q_z = request.initial_pose.pose.orientation.z;
        double q_w = request.initial_pose.pose.orientation.w;
        bool robot_fixed = false;

        Debug.Log("Received URDF path: " + urdfFilePath);

        // URDF からのロボット生成 (※ URDFRobotExtensions.Create はランタイム用のコルーチン実装が前提)
        ImportSettings settings = new ImportSettings();
        GameObject robotObject = UrdfRobotExtensions.CreateRuntime(urdfFilePath, settings);

        if (robotObject == null)
        {
            Debug.LogError("Failed to load robot from URDF.");
            spawnEntityResponse.result.result = SpawnEntityResponse.RESOURCE_PARSE_ERROR;
            spawnEntityResponse.result.error_message = "Failed to load robot from URDF.";
            return spawnEntityResponse;
        }

        // ロボットの位置・回転設定
        Vector3 newPosition = new Vector3(Convert.ToSingle(robot_x), Convert.ToSingle(robot_y), Convert.ToSingle(robot_z));
        robotObject.transform.position = newPosition;
        robotObject.transform.rotation = new Quaternion(Convert.ToSingle(q_x), Convert.ToSingle(q_y), Convert.ToSingle(q_z), Convert.ToSingle(q_w));

        // 最初に見つかった UrdfLink に対してベースリンク設定と固定フラグを適用
        List<GameObject> childObjectsWithUrdfLink = GetChildObjectsWithComponent<UrdfLink>(robotObject);
        foreach (GameObject child in childObjectsWithUrdfLink)
        {
            UrdfLink link = child.GetComponent<UrdfLink>();
            link.IsBaseLink = true;

            ArticulationBody body = child.GetComponent<ArticulationBody>();
            if (body != null)
            {
                body.immovable = robot_fixed;
            }
            break;
        }

        // URDFファイルの解析
        XmlDocument xmlDoc = new XmlDocument();
        xmlDoc.Load(urdfFilePath);

        // JointState 用の Publisher/Subscriber の設定
        JointStatePub jointStatePub = robotObject.AddComponent<JointStatePub>();
        JointStateSub jointStateSub = robotObject.AddComponent<JointStateSub>();
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

        // Physics Material の生成（ランタイムでは AssetDatabase は使用不可のため new で生成）
        string directoryPath = Path.GetDirectoryName(urdfFilePath);
        int assetsIndex = directoryPath.IndexOf("Assets");
        if (assetsIndex >= 0)
        {
            directoryPath = directoryPath.Substring(assetsIndex);
        }
        XmlNode robotNode = xmlDoc.SelectSingleNode("/robot");
        List<PhysicsMaterial> physicsMaterialList = new List<PhysicsMaterial>();
        if (robotNode != null)
        {
            XmlNodeList physicsMaterials = robotNode.SelectNodes("physics_material");
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
                    XmlNode physicsMaterial = collisionNode.SelectSingleNode("physics_material");
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
                                            foreach(PhysicsMaterial material in physicsMaterialList)
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

        /*
        // センサ設定 (URDF 内の <unity> 要素に基づく)
        int next_display_number = 1;
        if (robotNode != null)
        {
            XmlNode unityNode = robotNode.SelectSingleNode("unity");
            if (unityNode != null)
            {
                XmlNodeList unitySensors = unityNode.SelectNodes("sensor");
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
                                break;
                            case "camera":
                                Debug.Log("sensor type 'camera' found");
                                RGBCameraSensor cameraSensor = targetObject.AddComponent<RGBCameraSensor>();
                                cameraSensor._fov = TryParseFloat(sensor.SelectSingleNode("horizontal_fov").InnerText) * 180.0f / 3.14f;
                                int image_width, image_height;
                                int.TryParse(sensor.SelectSingleNode("image/width").InnerText, out image_width);
                                int.TryParse(sensor.SelectSingleNode("image/height").InnerText, out image_height);
                                cameraSensor._resolution = new Vector2(image_width, image_height);
                                UnityEngine.Camera cameraComponent = targetObject.GetComponent<UnityEngine.Camera>();
                                if (cameraComponent != null)
                                {
                                    cameraComponent.targetDisplay = next_display_number;
                                    next_display_number++;
                                }
                                CameraInfoMsgPublisher cameraInfoPublisher = targetObject.AddComponent<CameraInfoMsgPublisher>();
                                CompressedImageMsgPublisher cameraImagePublisher = targetObject.AddComponent<CompressedImageMsgPublisher>();
                                cameraInfoPublisher.serializer = new CameraInfoMsgSerializer();
                                cameraInfoPublisher.serializer.SetHeaderObject(cameraSensor);
                                cameraInfoPublisher.serializer.SetObject(cameraSensor);
                                cameraInfoPublisher.topicName = "/" + robotObject.name + "/" + sensorLinkName + "/camera_info";
                                cameraImagePublisher.serializer = new CompressedImageMsgSerializer();
                                cameraImagePublisher.serializer.SetHeaderObject(cameraSensor);
                                cameraImagePublisher.serializer.SetObject(cameraSensor);
                                cameraImagePublisher.topicName = "/" + robotObject.name + "/" + sensorLinkName + "/image_raw";
                                break;
                            case "depth_camera":
                                Debug.Log("sensor type 'depth_camera' found");
                                DepthCameraSensor depthCameraSensor = targetObject.AddComponent<DepthCameraSensor>();
                                depthCameraSensor._fov = TryParseFloat(sensor.SelectSingleNode("horizontal_fov").InnerText) * 180.0f / 3.14f;
                                int depth_image_width, depth_image_height;
                                int.TryParse(sensor.SelectSingleNode("image/width").InnerText, out depth_image_width);
                                int.TryParse(sensor.SelectSingleNode("image/height").InnerText, out depth_image_height);
                                depthCameraSensor._resolution = new Vector2(depth_image_width, depth_image_height);
                                UnityEngine.Camera depthCameraComponent = targetObject.GetComponent<UnityEngine.Camera>();
                                if (depthCameraComponent != null)
                                {
                                    depthCameraComponent.targetDisplay = next_display_number;
                                    next_display_number++;
                                }
                                CameraInfoMsgPublisher depthCameraInfoPublisher = targetObject.AddComponent<CameraInfoMsgPublisher>();
                                CompressedImageMsgPublisher depthCameraImagePublisher = targetObject.AddComponent<CompressedImageMsgPublisher>();
                                depthCameraInfoPublisher.serializer = new CameraInfoMsgSerializer();
                                depthCameraInfoPublisher.serializer.SetHeaderObject(depthCameraSensor);
                                depthCameraInfoPublisher.serializer.SetObject(depthCameraSensor);
                                depthCameraInfoPublisher.topicName = "/" + robotObject.name + "/" + sensorLinkName + "/depth_camera_info";
                                depthCameraImagePublisher.serializer = new CompressedImageMsgSerializer();
                                depthCameraImagePublisher.serializer.SetHeaderObject(depthCameraSensor);
                                depthCameraImagePublisher.serializer.SetObject(depthCameraSensor);
                                depthCameraImagePublisher.topicName = "/" + robotObject.name + "/" + sensorLinkName + "/depth_image_raw";
                                break;
                            default:
                                Debug.Log("undefined sensor type found");
                                break;
                        }
                    }
                }
            }
        }
        */

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

    private static float TryParseFloat(string value)
    {
        return float.TryParse(value, out float result) ? result : 0f;
    }
}
