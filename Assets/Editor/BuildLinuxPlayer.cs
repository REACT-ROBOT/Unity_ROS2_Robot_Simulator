using System;
using System.Linq;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;

/// <summary>
/// バッチモードから Linux プレイヤーをビルドするためのエントリポイント。
/// </summary>
/// <remarks>
/// サービス適合性テスト (docs/Service-Conformance-Test-ja.md) は実行中の
/// シミュレータへ接続して検証するため、変更を検証するには毎回プレイヤーを
/// 作り直す必要がある。CI から次のように呼ぶ:
///
///   Unity -batchmode -nographics -quit -projectPath . \
///     -executeMethod BuildLinuxPlayer.Build -buildOutput &lt;dir&gt;/Unity_ROS2_Robot_Simulator.x86_64
/// </remarks>
public static class BuildLinuxPlayer
{
    public static void Build()
    {
        // -buildTarget Windows64 で Windows (Mono) のクロスビルド。既定は Linux。
        // Windows ビルドには PlaybackEngines/WindowsStandaloneSupport が要る
        // (Linux エディタでは Mac 向け .pkg として配布されるモジュールを展開して置く)。
        string targetArg = GetArg("-playerTarget") ?? "Linux64";
        BuildTarget target;
        string defaultOutput;
        switch (targetArg)
        {
            case "Windows64":
                target = BuildTarget.StandaloneWindows64;
                defaultOutput = "Builds/Windows/Unity_ROS2_Robot_Simulator.exe";
                break;
            case "Linux64":
                target = BuildTarget.StandaloneLinux64;
                defaultOutput = "Builds/Linux/Unity_ROS2_Robot_Simulator.x86_64";
                break;
            default:
                throw new Exception($"Unknown -playerTarget '{targetArg}' (Linux64 | Windows64)");
        }

        string output = GetArg("-buildOutput") ?? defaultOutput;

        string[] scenes = EditorBuildSettings.scenes
            .Where(scene => scene.enabled)
            .Select(scene => scene.path)
            .ToArray();

        if (scenes.Length == 0)
        {
            throw new Exception("EditorBuildSettings に有効なシーンが 1 つも無い");
        }

        var options = new BuildPlayerOptions
        {
            scenes = scenes,
            locationPathName = output,
            target = target,
            targetGroup = BuildTargetGroup.Standalone,
            options = BuildOptions.None,
        };

        BuildReport report = BuildPipeline.BuildPlayer(options);
        BuildSummary summary = report.summary;

        Debug.Log($"[BuildLinuxPlayer] result={summary.result} output={output} " +
                  $"size={summary.totalSize} errors={summary.totalErrors}");

        if (summary.result != BuildResult.Succeeded)
        {
            // バッチモードでは例外を投げないと終了コードが 0 のままになる。
            throw new Exception($"Build failed: {summary.result} ({summary.totalErrors} errors)");
        }
    }

    static string GetArg(string name)
    {
        string[] args = Environment.GetCommandLineArgs();
        for (int i = 0; i < args.Length - 1; i++)
        {
            if (args[i] == name)
            {
                return args[i + 1];
            }
        }
        return null;
    }
}
