using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

using RosMessageTypes.Geometry;
using RosMessageTypes.SimulationInterfaces;

/// <summary>
/// simulation_resources.json の bounds 要素。
/// </summary>
/// <remarks>
/// 座標はすべて ROS (右手系・Z 上・m)。Unity 座標へは読み込み側では変換せず、
/// Bounds は ROS のまま返すインターフェースなのでそのまま流す。
/// convex_hull は SimulationControl 側が未対応なのでここでも受け付けない。
/// </remarks>
[Serializable]
public class SimulationBoundsConfig
{
    public string type;      // "empty" | "box" | "sphere"
    public float[] min;      // box: 下側の角 [x, y, z]
    public float[] max;      // box: 上側の角 [x, y, z]
    public float[] center;   // sphere: 中心 [x, y, z]
    public float radius;     // sphere: 半径 [m]
}

/// <summary>
/// simulation_resources.json の named_poses 要素。
/// </summary>
[Serializable]
public class SimulationNamedPoseConfig
{
    public string name;
    public string description;
    public string[] tags;
    public float[] position;     // ROS 座標 [x, y, z]
    public float[] orientation;  // ROS クォータニオン [x, y, z, w]
    public float[] rpy;          // ROS RPY [rad]。orientation が無いときだけ見る
    public SimulationBoundsConfig bounds;
}

/// <summary>
/// simulation_resources.json 全体。
/// </summary>
[Serializable]
public class SimulationResourceConfig
{
    public string[] spawnable_paths;   // GetSpawnables が走査するディレクトリ
    public string[] world_paths;       // GetAvailableWorlds が走査するディレクトリ
    public SimulationNamedPoseConfig[] named_poses;
}

/// <summary>
/// GetSpawnables / GetNamedPoses / GetAvailableWorlds が返す中身の供給元。
/// </summary>
/// <remarks>
/// これらのサービスは「シミュレータがどこを見えているか」に依存するが、Unity の
/// プレイヤーには ROS のパッケージ検索パスに相当する仕組みが無いので、設定ファイルで
/// 明示的に与える。探索先を環境変数だけで渡さないのは、named pose が姿勢・タグ・
/// bounds を持つ構造データで、環境変数には収まらないため。結局ファイルが要るなら
/// 検索パスも同じファイルに置いたほうが設定が 1 か所にまとまる。
///
/// 探索順は
///   1. 環境変数 SIMULATION_RESOURCES_CONFIG が指すパス
///   2. プレイヤー実行ファイルと同じディレクトリの simulation_resources.json
///      (エディタではプロジェクトルート)
///   3. Application.persistentDataPath/simulation_resources.json
/// で、どれも無ければ「何も登録されていない」として空を返す。設定なしで起動できることを
/// 優先しているので、ファイルが無いこと自体はエラーにしない。
/// </remarks>
public static class SimulationResources
{
    public const string ConfigFileName = "simulation_resources.json";
    public const string ConfigPathEnvVar = "SIMULATION_RESOURCES_CONFIG";

    private static SimulationResourceConfig s_Config = new SimulationResourceConfig();
    private static List<NamedPoseMsg> s_NamedPoses = new List<NamedPoseMsg>();
    private static Dictionary<string, BoundsMsg> s_NamedPoseBounds = new Dictionary<string, BoundsMsg>();
    private static bool s_Loaded;

    /// <summary>読み込んだ設定ファイルのパス。見つからなかった場合は空。</summary>
    public static string LoadedPath { get; private set; } = "";

    /// <summary>読み込みに失敗した理由。成功時・ファイルが無い場合は空。</summary>
    public static string LoadError { get; private set; } = "";

    public static IReadOnlyList<NamedPoseMsg> NamedPoses
    {
        get { EnsureLoaded(); return s_NamedPoses; }
    }

    public static string[] SpawnablePaths
    {
        get { EnsureLoaded(); return s_Config.spawnable_paths ?? Array.Empty<string>(); }
    }

    public static string[] WorldPaths
    {
        get { EnsureLoaded(); return s_Config.world_paths ?? Array.Empty<string>(); }
    }

    /// <summary>
    /// 名前付き姿勢の bounds。設定に bounds が無ければ TYPE_EMPTY を返す。
    /// 名前自体が無い場合は false。
    /// </summary>
    public static bool TryGetNamedPoseBounds(string name, out BoundsMsg bounds)
    {
        EnsureLoaded();
        return s_NamedPoseBounds.TryGetValue(name, out bounds);
    }

    private static void EnsureLoaded()
    {
        if (!s_Loaded)
        {
            Reload();
        }
    }

    /// <summary>設定ファイルを読み直す。LoadWorld などで中身が変わりうるので公開しておく。</summary>
    public static void Reload()
    {
        s_Loaded = true;
        s_Config = new SimulationResourceConfig();
        s_NamedPoses = new List<NamedPoseMsg>();
        s_NamedPoseBounds = new Dictionary<string, BoundsMsg>();
        LoadedPath = "";
        LoadError = "";

        string path = ResolveConfigPath();
        if (string.IsNullOrEmpty(path))
        {
            Debug.Log($"[SimulationResources] {ConfigFileName} が見つからないので、" +
                      "spawnables / named poses / worlds は空として扱う");
            return;
        }

        try
        {
            SimulationResourceConfig parsed =
                JsonUtility.FromJson<SimulationResourceConfig>(File.ReadAllText(path));
            if (parsed == null)
            {
                LoadError = $"{path} を設定として解釈できなかった";
                Debug.LogError("[SimulationResources] " + LoadError);
                return;
            }
            s_Config = parsed;
            LoadedPath = path;
        }
        catch (Exception e)
        {
            LoadError = $"{path} の読み込みに失敗: {e.Message}";
            Debug.LogError("[SimulationResources] " + LoadError);
            return;
        }

        BuildNamedPoses();
        Debug.Log($"[SimulationResources] {path} を読み込んだ " +
                  $"(spawnable_paths={SpawnablePaths.Length}, world_paths={WorldPaths.Length}, " +
                  $"named_poses={s_NamedPoses.Count})");
    }

    private static string ResolveConfigPath()
    {
        string fromEnv = Environment.GetEnvironmentVariable(ConfigPathEnvVar);
        if (!string.IsNullOrEmpty(fromEnv))
        {
            if (File.Exists(fromEnv))
            {
                return fromEnv;
            }
            // 明示的に指定されたのに無いのは設定ミスなので黙って無視しない。
            LoadError = $"{ConfigPathEnvVar} が指す {fromEnv} が存在しない";
            Debug.LogWarning("[SimulationResources] " + LoadError);
        }

        // Application.dataPath はプレイヤーでは <実行ファイル>_Data、エディタでは
        // <プロジェクト>/Assets なので、親がそれぞれ実行ファイルの隣とプロジェクトルート。
        string nextToPlayer = Path.Combine(
            Path.GetDirectoryName(Application.dataPath) ?? ".", ConfigFileName);
        if (File.Exists(nextToPlayer))
        {
            return nextToPlayer;
        }

        string inPersistentData = Path.Combine(Application.persistentDataPath, ConfigFileName);
        if (File.Exists(inPersistentData))
        {
            return inPersistentData;
        }

        return "";
    }

    private static void BuildNamedPoses()
    {
        if (s_Config.named_poses == null)
        {
            return;
        }

        foreach (SimulationNamedPoseConfig entry in s_Config.named_poses)
        {
            if (entry == null || string.IsNullOrEmpty(entry.name))
            {
                Debug.LogWarning("[SimulationResources] name の無い named_pose を読み飛ばした");
                continue;
            }
            if (s_NamedPoseBounds.ContainsKey(entry.name))
            {
                // NamedPose.name は一意と決まっているので、後勝ちにせず最初の 1 つを残す。
                Debug.LogWarning($"[SimulationResources] named_pose '{entry.name}' が重複している。後のものを読み飛ばした");
                continue;
            }

            var pose = new NamedPoseMsg
            {
                name = entry.name,
                description = entry.description ?? "",
                tags = entry.tags ?? Array.Empty<string>(),
                pose = new PoseMsg
                {
                    position = ToPoint(entry.position),
                    orientation = ToQuaternion(entry.orientation, entry.rpy)
                }
            };
            s_NamedPoses.Add(pose);
            s_NamedPoseBounds[entry.name] = ToBounds(entry.bounds, entry.name);
        }
    }

    private static PointMsg ToPoint(float[] xyz)
    {
        if (xyz == null || xyz.Length < 3)
        {
            return new PointMsg();
        }
        return new PointMsg(xyz[0], xyz[1], xyz[2]);
    }

    private static Vector3Msg ToVector3(float[] xyz)
    {
        if (xyz == null || xyz.Length < 3)
        {
            return new Vector3Msg();
        }
        return new Vector3Msg(xyz[0], xyz[1], xyz[2]);
    }

    private static QuaternionMsg ToQuaternion(float[] xyzw, float[] rpy)
    {
        if (xyzw != null && xyzw.Length >= 4)
        {
            return new QuaternionMsg(xyzw[0], xyzw[1], xyzw[2], xyzw[3]);
        }
        if (rpy != null && rpy.Length >= 3)
        {
            // ROS の RPY は固定軸 XYZ (= 回転軸順 ZYX) 回り。
            double cr = Math.Cos(rpy[0] * 0.5), sr = Math.Sin(rpy[0] * 0.5);
            double cp = Math.Cos(rpy[1] * 0.5), sp = Math.Sin(rpy[1] * 0.5);
            double cy = Math.Cos(rpy[2] * 0.5), sy = Math.Sin(rpy[2] * 0.5);
            return new QuaternionMsg(
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
                cr * cp * cy + sr * sp * sy);
        }
        return new QuaternionMsg(0, 0, 0, 1);
    }

    private static BoundsMsg ToBounds(SimulationBoundsConfig config, string poseName)
    {
        var bounds = new BoundsMsg
        {
            type = BoundsMsg.TYPE_EMPTY,
            points = Array.Empty<Vector3Msg>()
        };
        if (config == null || string.IsNullOrEmpty(config.type))
        {
            return bounds;
        }

        switch (config.type.ToLowerInvariant())
        {
            case "empty":
                return bounds;
            case "box":
                if (config.min == null || config.max == null)
                {
                    Debug.LogWarning($"[SimulationResources] named_pose '{poseName}' の box bounds に min/max が無い。bounds 無しとして扱う");
                    return bounds;
                }
                bounds.type = BoundsMsg.TYPE_BOX;
                // Bounds.msg の box は「upper right, lower left」の順。
                bounds.points = new[] { ToVector3(config.max), ToVector3(config.min) };
                return bounds;
            case "sphere":
                bounds.type = BoundsMsg.TYPE_SPHERE;
                // sphere は 1 点目が中心、2 点目の x が半径 (y, z は無視される)。
                bounds.points = new[] { ToVector3(config.center), new Vector3Msg(config.radius, 0, 0) };
                return bounds;
            default:
                Debug.LogWarning($"[SimulationResources] named_pose '{poseName}' の bounds type '{config.type}' は未対応。bounds 無しとして扱う");
                return bounds;
        }
    }
}
