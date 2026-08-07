using System;
using System.Collections.Generic;
using System.IO;
using RosMessageGeneration = Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEditor;
using UnityEngine;

/// <summary>
/// ROS のインターフェース定義 (.msg/.srv/.action) から Assets/RosMessages 以下の
/// C# クラスを再生成するバッチモード用エントリポイント。
/// </summary>
/// <remarks>
/// simulation_interfaces を更新したら、エディタの Robotics メニューを手で辿らずに
/// これを回して差分を確認する。バージョン間の破壊的変更 (2.0.0 の
/// uri/resource_string -> Resource entity_resource など) は、生成物の diff を見るのが
/// 一番確実。
///
///   Unity -batchmode -nographics -quit -projectPath . \
///     -executeMethod GenerateRosMessages.Generate \
///     -messageInput &lt;ROS パッケージのディレクトリ&gt;
///
/// -messageInput は msg/ srv/ action/ を含むパッケージのルート。省略時は
/// Unity_ROS2_sample の simulation_interfaces を見る。
/// </remarks>
public static class GenerateRosMessages
{
    const string k_DefaultInput = "../Unity_ROS2_sample/colcon_ws/src/simulation_interfaces";
    const string k_OutputPath = "Assets/RosMessages";

    public static void Generate()
    {
        string input = GetArg("-messageInput") ?? k_DefaultInput;
        string output = GetArg("-messageOutput") ?? k_OutputPath;

        if (!Directory.Exists(input))
        {
            throw new Exception($"定義ディレクトリが見つからない: {Path.GetFullPath(input)}");
        }

        var warnings = new List<string>();
        int generated = 0;

        // msg -> srv の順。srv の生成は msg の型解決に依存する。
        foreach (string kind in new[] { "msg", "srv", "action" })
        {
            string dir = Path.Combine(input, kind);
            if (!Directory.Exists(dir))
            {
                continue;
            }

            string[] files = Directory.GetFiles(dir, "*." + kind);
            generated += files.Length;
            Debug.Log($"[GenerateRosMessages] {kind}: {files.Length} 件");

            switch (kind)
            {
                case "msg":
                    warnings.AddRange(RosMessageGeneration.MessageAutoGen.GeneratePackageMessages(input, output));
                    break;
                case "srv":
                    warnings.AddRange(RosMessageGeneration.ServiceAutoGen.GeneratePackageServices(input, output));
                    break;
                case "action":
                    warnings.AddRange(RosMessageGeneration.ActionAutoGen.GeneratePackageActions(input, output));
                    break;
            }
        }

        foreach (string warning in warnings)
        {
            Debug.LogWarning("[GenerateRosMessages] " + warning);
        }

        AssetDatabase.Refresh();
        Debug.Log($"[GenerateRosMessages] done: input={Path.GetFullPath(input)} " +
                  $"output={output} files={generated} warnings={warnings.Count}");
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
