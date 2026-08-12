using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using SdfWorld;

/// <summary>
/// SdfWorldImporter (SDF → 景観記述) と既存のシーン JSON パイプライン
/// (SavedSceneData → ObjectSpawner.LoadSceneData) をつなぐ変換層。
/// LoadWorld サービスと GUI のロードボタンの両方からここを通る。
/// </summary>
public static class SdfWorldLoading
{
    /// <summary>拡張子が SDF ワールドのものか。</summary>
    public static bool IsSdfWorldPath(string path)
    {
        string ext = Path.GetExtension(path);
        return string.Equals(ext, ".sdf", StringComparison.OrdinalIgnoreCase)
            || string.Equals(ext, ".world", StringComparison.OrdinalIgnoreCase);
    }

    /// <summary>
    /// model:// の検索ルート。ワールドファイルのディレクトリを最優先にして、
    /// simulation_resources.json の検索パスと Gazebo の環境変数を続ける。
    /// </summary>
    public static List<string> BuildModelSearchRoots(string worldDir)
    {
        var roots = new List<string>();
        if (!string.IsNullOrEmpty(worldDir))
        {
            roots.Add(worldDir);
            // Gazebo の慣習: ワールドファイルの隣の models/ にモデルを置く
            roots.Add(Path.Combine(worldDir, "models"));
        }
        roots.AddRange(SimulationResources.WorldPaths);
        roots.AddRange(SimulationResources.SpawnablePaths);
        AddEnvPaths(roots, "GZ_SIM_RESOURCE_PATH");
        AddEnvPaths(roots, "GAZEBO_MODEL_PATH");
        return roots;
    }

    private static void AddEnvPaths(List<string> roots, string envVar)
    {
        string value = Environment.GetEnvironmentVariable(envVar);
        if (string.IsNullOrEmpty(value))
        {
            return;
        }
        foreach (string path in value.Split(':'))
        {
            if (!string.IsNullOrEmpty(path))
            {
                roots.Add(path);
            }
        }
    }

    /// <summary>
    /// SDF ワールドのテキストを SavedSceneData へ変換する。構文が壊れている場合だけ
    /// false。要素単位の問題 (未対応要素・見つからない参照) は preReport に載せて
    /// 続行する — LoadWorld が ignore_missing_or_unsupported_assets /
    /// fail_on_unsupported_element で扱いを決められるようにするため。
    /// </summary>
    public static bool TryParseSdfWorld(
        string xml, string baseDir,
        out SavedSceneData sceneData, out SceneLoadReport preReport, out string error)
    {
        sceneData = null;
        preReport = null;

        List<string> roots = BuildModelSearchRoots(baseDir);
        if (!SdfWorldImporter.TryParseWorld(xml, baseDir, roots, out SdfWorldData data, out error))
        {
            return false;
        }

        sceneData = new SavedSceneData
        {
            name = data.worldName,
            description = $"SDF world with {data.objects.Count} object(s)",
            tags = Array.Empty<string>()
        };
        foreach (SdfSceneObject obj in data.objects)
        {
            SavedObjectMotion motion = null;
            if (obj.motionWaypoints != null && obj.motionWaypoints.Count >= 2)
            {
                motion = new SavedObjectMotion
                {
                    useTimes = true,
                    loop = obj.motionPingPong ? "pingpong" : "loop",
                };
                foreach (SdfMotionWaypoint w in obj.motionWaypoints)
                {
                    motion.waypoints.Add(new SavedMotionWaypoint
                    {
                        position = new[] { w.position.x, w.position.y, w.position.z },
                        yawDeg = w.yawDeg,
                        time = w.time,
                    });
                }
            }

            sceneData.objects.Add(new SavedObjectData
            {
                type = obj.type,
                position = new[] { obj.position.x, obj.position.y, obj.position.z },
                rotationEuler = new[] { obj.rotationEuler.x, obj.rotationEuler.y, obj.rotationEuler.z },
                scale = new[] { obj.scale.x, obj.scale.y, obj.scale.z },
                meshPath = obj.meshPath,
                color = obj.hasColor
                    ? new[] { obj.color.r, obj.color.g, obj.color.b, obj.color.a }
                    : null,
                motion = motion,
                isActive = true
            });
        }

        preReport = new SceneLoadReport
        {
            missingAssets = data.hasMissingAssets,
            unsupportedElements = data.hasUnsupportedElements
        };
        preReport.messages.AddRange(data.messages);
        return true;
    }
}
