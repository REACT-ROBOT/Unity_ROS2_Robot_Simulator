using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.UrdfImporter;

/// <summary>
/// エンティティ一覧パネル (EntityListPanel) 向けの読み取り専用スナップショット API。
/// </summary>
/// <remarks>
/// UI 側に m_EntityList を直接触らせないための境界。ここは読み取りだけを提供し、
/// エンティティの生成・削除はサービス実装 (SimulationEntityServices.cs) が持つ。
/// パネル自体はシーンファイルを編集せず、ClockPub と同じ方針でコードから常駐させる
/// (シーン YAML の手編集は事故りやすい)。
/// </remarks>
public partial class SimulationControl
{
    /// <summary>パネルが扱う可動関節の種別。URDF の joint type に対応する。</summary>
    public enum GuiJointKind
    {
        Revolute,
        Continuous,
        Prismatic,
    }

    /// <summary>
    /// 可動関節 1 つ分のスナップショット。単位はすべて SI (rad / m)。
    /// body / servo は破棄され得るので、使う側は毎回 null を確かめること。
    /// </summary>
    public struct GuiJointInfo
    {
        public string jointName;
        public ArticulationBody body;
        // ServoJointModel が付いている関節は、ROS 指令と同様にモデル経由で
        // 指令する必要がある (JointStateSub と同じ分岐)。
        public ServoJointModel servo;
        public GuiJointKind kind;
        // continuous は常に false。revolute/prismatic でも URDF が範囲を
        // 持たない場合は false になる。
        public bool hasLimits;
        public float lowerLimit; // rad / m
        public float upperLimit; // rad / m
        // ros2_control で effort 指令を宣言した関節。位置スライダでは指令
        // できないので、GUI では表示専用にする。
        public bool isEffortMode;
    }

    void Awake()
    {
        // エンティティ一覧パネルはコードから取り付ける (Start の ClockPub と同じ)。
        // Start は本体ファイル側で定義済みなので、partial 側は Awake を使う。
        if (GetComponent<EntityListPanel>() == null)
        {
            gameObject.AddComponent<EntityListPanel>();
        }
    }

    /// <summary>
    /// 現在スポーンされているエンティティを buffer へ詰める (破棄済みは除く)。
    /// 呼び出し側がリストを使い回すことで、ポーリングしてもアロケーションが出ない。
    /// </summary>
    public void GetEntitiesSnapshot(List<GameObject> buffer)
    {
        buffer.Clear();
        foreach (GameObject entity in m_EntityList)
        {
            if (entity != null)
            {
                buffer.Add(entity);
            }
        }
    }

    /// <summary>
    /// エンティティ配下の可動関節 (revolute / continuous / prismatic) を列挙する。
    /// </summary>
    /// <remarks>
    /// スポーン時の JointStatePub/Sub の組み立てと同じく ArticulationBody 基準で、
    /// 名前は UrdfJoint.jointName から取る。リミットは ArticulationBody.xDrive の
    /// lowerLimit/upperLimit (回転関節は度) を SI へ直して返す。
    /// </remarks>
    public static void GetMovableJoints(GameObject entity, List<GuiJointInfo> buffer)
    {
        buffer.Clear();
        if (entity == null)
        {
            return;
        }

        JointStateSub jointStateSub = entity.GetComponent<JointStateSub>();

        foreach (UrdfJoint urdfJoint in entity.GetComponentsInChildren<UrdfJoint>())
        {
            GuiJointKind kind;
            switch (urdfJoint.JointType)
            {
                case UrdfJoint.JointTypes.Revolute:
                    kind = GuiJointKind.Revolute;
                    break;
                case UrdfJoint.JointTypes.Continuous:
                    kind = GuiJointKind.Continuous;
                    break;
                case UrdfJoint.JointTypes.Prismatic:
                    kind = GuiJointKind.Prismatic;
                    break;
                default:
                    continue; // fixed / floating / planar は対象外
            }

            ArticulationBody body = urdfJoint.GetComponent<ArticulationBody>();
            if (body == null
                || body.jointType == ArticulationJointType.FixedJoint
                || body.dofCount < 1)
            {
                continue;
            }

            var info = new GuiJointInfo
            {
                jointName = urdfJoint.jointName,
                body = body,
                servo = urdfJoint.GetComponent<ServoJointModel>(),
                kind = kind,
                isEffortMode = jointStateSub != null && jointStateSub.IsEffortJoint(urdfJoint.jointName),
            };

            if (kind != GuiJointKind.Continuous)
            {
                // xDrive のリミットは回転関節が度、直動関節がメートル。
                ArticulationDrive drive = body.xDrive;
                if (drive.upperLimit > drive.lowerLimit)
                {
                    float toSi = kind == GuiJointKind.Prismatic ? 1f : Mathf.Deg2Rad;
                    info.hasLimits = true;
                    info.lowerLimit = drive.lowerLimit * toSi;
                    info.upperLimit = drive.upperLimit * toSi;
                }
            }

            buffer.Add(info);
        }
    }

    /// <summary>パネルが扱うセンサ可視化の種別。</summary>
    public enum GuiSensorVizKind
    {
        PointCloudXYZI,   // LiDAR (2D/3D)
        PointCloudXYZ,    // 深度カメラの点群
        PointCloudXYZRGB, // RGBD カメラの色付き点群
        ImageTexture0,    // カメラ画像 (texture0)
        ImageTexture1,    // RGBD のカラー画像 (texture1)
    }

    /// <summary>
    /// 可視化できるセンサ出力 1 つ分のスナップショット。1 センサから複数出る
    /// ことがある (RGBD は点群とカラー画像)。sensor は破棄され得るので、使う側は
    /// 毎回 null を確かめること。
    /// </summary>
    public struct GuiSensorInfo
    {
        public string label; // リンク名 + 出力種別 (パネルの行ラベル)
        public UnitySensors.Sensor.UnitySensor sensor;
        public GuiSensorVizKind kind;
    }

    /// <summary>
    /// エンティティ配下の可視化できるセンサ出力を列挙する。可視化があるのは
    /// 点群 (LiDAR/深度/RGBD) と画像 (RGB/魚眼/パノラマ/RGBD カラー) だけで、
    /// IMU・GNSS・接触などの数値系センサは対象外。深度カメラの深度画像
    /// (32 bit float, m 単位) はそのまま表示しても白飛びするので点群のみ出す。
    /// </summary>
    public static void GetVisualizableSensors(GameObject entity, List<GuiSensorInfo> buffer)
    {
        buffer.Clear();
        if (entity == null)
        {
            return;
        }

        foreach (UnitySensors.Sensor.UnitySensor sensor in
            entity.GetComponentsInChildren<UnitySensors.Sensor.UnitySensor>())
        {
            string link = sensor.gameObject.name;
            if (sensor is UnitySensors.Sensor.Camera.RGBDCameraSensor)
            {
                buffer.Add(new GuiSensorInfo
                {
                    label = link + " points",
                    sensor = sensor,
                    kind = GuiSensorVizKind.PointCloudXYZRGB,
                });
                buffer.Add(new GuiSensorInfo
                {
                    label = link + " color",
                    sensor = sensor,
                    kind = GuiSensorVizKind.ImageTexture1,
                });
            }
            else if (sensor is UnitySensors.Sensor.Camera.DepthCameraSensor)
            {
                buffer.Add(new GuiSensorInfo
                {
                    label = link + " points",
                    sensor = sensor,
                    kind = GuiSensorVizKind.PointCloudXYZ,
                });
            }
            else if (sensor is UnitySensors.Interface.Sensor.IPointCloudInterface<
                UnitySensors.DataType.Sensor.PointCloud.PointXYZI>)
            {
                buffer.Add(new GuiSensorInfo
                {
                    label = link + " points",
                    sensor = sensor,
                    kind = GuiSensorVizKind.PointCloudXYZI,
                });
            }
            else if (sensor is UnitySensors.Interface.Sensor.ITextureInterface)
            {
                buffer.Add(new GuiSensorInfo
                {
                    label = link + " image",
                    sensor = sensor,
                    kind = GuiSensorVizKind.ImageTexture0,
                });
            }
        }
    }

    /// <summary>
    /// 関節の現在位置を SI (rad / m) で返す。破棄済みなら false。
    /// </summary>
    public static bool TryGetJointPosition(GuiJointInfo joint, out float positionSi)
    {
        positionSi = 0f;
        if (joint.body == null || joint.body.dofCount < 1)
        {
            return false;
        }
        float value = joint.body.jointPosition[0];
        if (!float.IsFinite(value))
        {
            return false;
        }
        positionSi = value;
        return true;
    }
}
