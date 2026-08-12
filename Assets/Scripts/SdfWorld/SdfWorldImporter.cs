using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Xml;
using UnityEngine;

namespace SdfWorld
{
    /// <summary>
    /// SDF ワールドから生成する景観オブジェクト 1 個ぶんの記述。
    /// 座標・回転・スケールはすべて Unity 座標系へ変換済み。
    /// type は ObjectSpawner のシーン JSON (SavedObjectData.type) と同じ語彙で、
    /// メッシュだけ "RosMesh" (ROS/Gazebo 慣習の Z-up メッシュ) を使う。
    /// </summary>
    /// <summary>動く障害物 (actor) のウェイポイント。座標は Unity 系。</summary>
    public struct SdfMotionWaypoint
    {
        public Vector3 position;
        public float yawDeg;
        public float time; // 経路開始からの秒
    }

    public class SdfSceneObject
    {
        public string type;
        public Vector3 position;
        public Vector3 rotationEuler;
        public Vector3 scale = Vector3.one;
        public string meshPath;
        public bool hasColor;
        public Color color;
        public string sourceName; // 由来の model/light 名 (メッセージ用)
        // actor の trajectory。2 点以上あれば動く障害物として生成される
        public List<SdfMotionWaypoint> motionWaypoints;
        public bool motionPingPong; // SDF に往復は無いので通常 false
    }

    /// <summary>SDF ワールドの解釈結果。</summary>
    public class SdfWorldData
    {
        public string worldName = "";
        public List<SdfSceneObject> objects = new List<SdfSceneObject>();
        public List<string> messages = new List<string>();
        // 「景観として意味のある内容を落とした」= LoadWorld の UNSUPPORTED_ELEMENTS 相当
        public bool hasUnsupportedElements;
        // include / mesh の参照先が見つからない = MISSING_ASSETS 相当
        public bool hasMissingAssets;
    }

    /// <summary>
    /// Gazebo の SDF ワールド (.sdf / .world) の静的サブセットを解釈して、
    /// 景観オブジェクトのリストへ変換する。物理・プラグイン・アクター等は扱わない
    /// (このシミュレータの「ワールド」は景観であり、動くものはエンティティとして
    /// URDF からスポーンする、という既存の整理に合わせる)。
    /// </summary>
    /// <remarks>
    /// 変換規則:
    /// - pose は model → (nested model) → link → visual の順で合成し、最後に
    ///   ROS(FLU, Z-up) → Unity(RUF, Y-up) 変換する (URDF-Importer の Ros2Unity と同式)。
    /// - box/cylinder/sphere/plane は Unity プリミティブへ (寸法は軸を入れ替えて scale へ)。
    /// - mesh は "RosMesh" として絶対パスへ解決する (model:// は検索ルートから解決)。
    /// - static でない model も静的景観として置く (報告はする)。動的な物体が要るなら
    ///   URDF でスポーンするのが本筋なので、ここでは剛体化しない。
    /// </remarks>
    public static class SdfWorldImporter
    {
        // include の入れ子は 1 段の include が別の include を含むケース。循環を
        // 断ち切るための上限で、実用上のワールドで困らない深さにしてある。
        private const int k_MaxIncludeDepth = 4;

        private struct Pose
        {
            public Vector3 position;   // ROS 座標系
            public Quaternion rotation; // ROS 座標系

            public static readonly Pose Identity = new Pose
            {
                position = Vector3.zero,
                rotation = Quaternion.identity
            };

            public Pose Compose(Pose child)
            {
                return new Pose
                {
                    position = position + rotation * child.position,
                    rotation = rotation * child.rotation
                };
            }
        }

        private class Context
        {
            public SdfWorldData data;
            public IReadOnlyList<string> modelSearchRoots;
            // 「skipped <physics>」の類を要素名ごとに 1 回だけ報告するための集合
            public HashSet<string> notedSkips = new HashSet<string>();
        }

        /// <summary>
        /// テキストが SDF ワールドらしいかの軽い判定。GetAvailableWorlds の候補選別用。
        /// </summary>
        public static bool LooksLikeSdfWorld(string text)
        {
            if (string.IsNullOrEmpty(text))
            {
                return false;
            }
            return text.IndexOf("<sdf", StringComparison.OrdinalIgnoreCase) >= 0
                && text.IndexOf("<world", StringComparison.OrdinalIgnoreCase) >= 0;
        }

        /// <summary>
        /// SDF ワールドの XML を解釈する。構文が壊れている・ワールドが無い場合だけ
        /// false (error に理由)。要素単位の問題は data のフラグとメッセージで返す。
        /// </summary>
        /// <param name="xml">SDF ファイルの中身。</param>
        /// <param name="baseDir">相対パスの基準 (ワールドファイルのあるディレクトリ)。resource_string 経由なら null。</param>
        /// <param name="modelSearchRoots">model:// を解決する検索ルート。</param>
        public static bool TryParseWorld(
            string xml, string baseDir, IReadOnlyList<string> modelSearchRoots,
            out SdfWorldData data, out string error)
        {
            data = null;
            error = "";

            var doc = new XmlDocument();
            try
            {
                doc.LoadXml(xml);
            }
            catch (Exception e)
            {
                error = $"SDF の XML を解釈できなかった: {e.Message}";
                return false;
            }

            XmlElement root = doc.DocumentElement;
            if (root == null || !string.Equals(root.Name, "sdf", StringComparison.OrdinalIgnoreCase))
            {
                error = $"ルート要素が <sdf> ではない (<{root?.Name}>)";
                return false;
            }
            XmlElement world = null;
            foreach (XmlNode child in root.ChildNodes)
            {
                if (child is XmlElement el && el.Name == "world")
                {
                    world = el;
                    break;
                }
            }
            if (world == null)
            {
                error = "<sdf> の中に <world> が無い (モデル単体の SDF はワールドとして読めない)";
                return false;
            }

            var ctx = new Context
            {
                data = new SdfWorldData(),
                modelSearchRoots = modelSearchRoots ?? Array.Empty<string>()
            };
            ctx.data.worldName = world.GetAttribute("name");

            foreach (XmlNode node in world.ChildNodes)
            {
                if (!(node is XmlElement el))
                {
                    continue;
                }
                switch (el.Name)
                {
                    case "model":
                        ParseModel(el, Pose.Identity, baseDir, ctx, null, null, null, 0);
                        break;
                    case "include":
                        ParseInclude(el, Pose.Identity, ctx, 0);
                        break;
                    case "light":
                        ParseLight(el, ctx);
                        break;
                    case "actor":
                        ParseActor(el, baseDir, ctx);
                        break;
                    // 景観の見た目・当たりに寄与しない設定類は、読み飛ばしても
                    // 「ワールドの内容が欠けた」ことにはならないので報告だけにする。
                    case "physics":
                    case "scene":
                    case "gui":
                    case "audio":
                    case "wind":
                    case "atmosphere":
                    case "magnetic_field":
                    case "gravity":
                    case "spherical_coordinates":
                    case "plugin":
                        NoteSkip(ctx, el.Name);
                        break;
                    default:
                        // actor や population などの内容要素。落とすと世界の中身が
                        // 変わるので unsupported として数える。
                        ctx.data.hasUnsupportedElements = true;
                        ctx.data.messages.Add($"未対応の要素 <{el.Name}> を読み飛ばした");
                        break;
                }
            }

            data = ctx.data;
            return true;
        }

        private static void NoteSkip(Context ctx, string elementName)
        {
            if (ctx.notedSkips.Add(elementName))
            {
                ctx.data.messages.Add($"<{elementName}> はこのシミュレータでは使わないので無視した");
            }
        }

        // ====================================================================
        // model / include
        // ====================================================================

        private static void ParseModel(
            XmlElement model, Pose parentPose, string baseDir, Context ctx,
            string nameOverride, Pose? poseOverride, bool? staticOverride, int depth)
        {
            string name = string.IsNullOrEmpty(nameOverride) ? model.GetAttribute("name") : nameOverride;

            Pose modelPose = poseOverride ?? ParsePose(SelectChild(model, "pose"), name, ctx);
            Pose worldPose = parentPose.Compose(modelPose);

            bool isStatic = staticOverride ?? ParseBool(SelectChild(model, "static"), false);
            if (!isStatic)
            {
                // Gazebo なら重力で動く物体だが、ここでは静的景観として置く。
                // 動く物体はエンティティ (URDF) の領分、という整理を崩さないため。
                ctx.data.messages.Add($"model '{name}' は static でないが、静的な景観として配置した");
            }

            foreach (XmlNode node in model.ChildNodes)
            {
                if (!(node is XmlElement el))
                {
                    continue;
                }
                switch (el.Name)
                {
                    case "link":
                        ParseLink(el, worldPose, baseDir, ctx, name);
                        break;
                    case "model": // SDF 1.8+ の入れ子モデル
                        ParseModel(el, worldPose, baseDir, ctx, null, null, isStatic ? (bool?)true : null, depth);
                        break;
                    case "include":
                        ParseInclude(el, worldPose, ctx, depth + 1);
                        break;
                    case "joint":
                        // 全リンクを as-authored の pose で固定配置するので、fixed 相当の
                        // 見た目にはなる。可動はしないことだけ伝える。
                        ctx.data.messages.Add($"model '{name}' の <joint> は無視した (リンクは記述どおりの位置に固定)");
                        break;
                    case "pose":
                    case "static":
                        break; // 解釈済み
                    case "plugin":
                    case "enable_wind":
                    case "self_collide":
                    case "allow_auto_disable":
                        NoteSkip(ctx, el.Name);
                        break;
                    default:
                        NoteSkip(ctx, el.Name);
                        break;
                }
            }
        }

        private static void ParseInclude(XmlElement include, Pose parentPose, Context ctx, int depth)
        {
            string uri = SelectChild(include, "uri")?.InnerText?.Trim();
            if (string.IsNullOrEmpty(uri))
            {
                ctx.data.hasUnsupportedElements = true;
                ctx.data.messages.Add("<include> に <uri> が無い");
                return;
            }
            if (depth > k_MaxIncludeDepth)
            {
                ctx.data.hasUnsupportedElements = true;
                ctx.data.messages.Add($"include の入れ子が深すぎる ({uri})");
                return;
            }

            if (!TryResolveModelDir(uri, ctx.modelSearchRoots, out string modelDir, out string sdfPath))
            {
                ctx.data.hasMissingAssets = true;
                ctx.data.messages.Add(
                    $"include '{uri}' を解決できなかった (検索ルート: {string.Join(", ", ctx.modelSearchRoots)})");
                return;
            }

            XmlDocument doc = new XmlDocument();
            try
            {
                doc.Load(sdfPath);
            }
            catch (Exception e)
            {
                ctx.data.hasMissingAssets = true;
                ctx.data.messages.Add($"include '{uri}' の {sdfPath} を読めなかった: {e.Message}");
                return;
            }

            XmlElement model = doc.DocumentElement != null
                ? SelectChild(doc.DocumentElement, "model") : null;
            if (doc.DocumentElement == null || model == null)
            {
                ctx.data.hasUnsupportedElements = true;
                ctx.data.messages.Add($"include '{uri}' の {sdfPath} に <model> が無い");
                return;
            }

            string nameOverride = SelectChild(include, "name")?.InnerText?.Trim();
            // include の pose / static はモデル自身の宣言を置き換える (Gazebo と同じ)。
            XmlElement poseEl = SelectChild(include, "pose");
            Pose? poseOverride = poseEl != null
                ? ParsePose(poseEl, nameOverride ?? uri, ctx) : (Pose?)null;
            XmlElement staticEl = SelectChild(include, "static");
            bool? staticOverride = staticEl != null ? ParseBool(staticEl, false) : (bool?)null;

            // include 先のメッシュ相対パスはモデルディレクトリ基準で解決する。
            ParseModel(model, parentPose, modelDir, ctx, nameOverride, poseOverride, staticOverride, depth);
        }

        /// <summary>
        /// model:// URI かディレクトリ名を、検索ルートから model.sdf のあるディレクトリへ解決する。
        /// </summary>
        private static bool TryResolveModelDir(
            string uri, IReadOnlyList<string> roots, out string modelDir, out string sdfPath)
        {
            modelDir = null;
            sdfPath = null;

            string relative;
            const string modelScheme = "model://";
            if (uri.StartsWith(modelScheme, StringComparison.OrdinalIgnoreCase))
            {
                relative = uri.Substring(modelScheme.Length);
            }
            else if (uri.StartsWith("file://", StringComparison.OrdinalIgnoreCase))
            {
                // file:// はディレクトリまたは .sdf を直接指す
                string path = new Uri(uri).LocalPath;
                return TryUseModelPath(path, ref modelDir, ref sdfPath);
            }
            else
            {
                relative = uri;
            }

            foreach (string root in roots)
            {
                if (string.IsNullOrEmpty(root))
                {
                    continue;
                }
                string candidate = Path.Combine(root, relative);
                if (TryUseModelPath(candidate, ref modelDir, ref sdfPath))
                {
                    return true;
                }
            }
            return false;
        }

        private static bool TryUseModelPath(string path, ref string modelDir, ref string sdfPath)
        {
            if (File.Exists(path) && path.EndsWith(".sdf", StringComparison.OrdinalIgnoreCase))
            {
                modelDir = Path.GetDirectoryName(path);
                sdfPath = path;
                return true;
            }
            if (!Directory.Exists(path))
            {
                return false;
            }
            string direct = Path.Combine(path, "model.sdf");
            if (File.Exists(direct))
            {
                modelDir = path;
                sdfPath = direct;
                return true;
            }
            // model.config だけ名前が違うモデルもあるので、*.sdf を 1 個だけ許す
            string[] anySdf = Directory.GetFiles(path, "*.sdf");
            if (anySdf.Length > 0)
            {
                Array.Sort(anySdf, StringComparer.Ordinal);
                modelDir = path;
                sdfPath = anySdf[0];
                return true;
            }
            return false;
        }

        // ====================================================================
        // link / visual / geometry
        // ====================================================================

        private static void ParseLink(
            XmlElement link, Pose modelPose, string baseDir, Context ctx, string modelName)
        {
            string linkName = link.GetAttribute("name");
            string label = $"{modelName}/{linkName}";
            Pose linkPose = modelPose.Compose(ParsePose(SelectChild(link, "pose"), label, ctx));

            var visuals = SelectChildren(link, "visual");
            if (visuals.Count == 0)
            {
                // 見た目の無いリンクでも当たり判定はワールドの一部なので、collision
                // から形を起こす (lidar の遮蔽物としては同じ)。
                visuals = SelectChildren(link, "collision");
                if (visuals.Count > 0)
                {
                    ctx.data.messages.Add($"link '{label}' に <visual> が無いので <collision> の形状を使った");
                }
            }

            foreach (XmlElement visual in visuals)
            {
                string visualLabel = $"{label}/{visual.GetAttribute("name")}";
                Pose pose = linkPose.Compose(ParsePose(SelectChild(visual, "pose"), visualLabel, ctx));
                XmlElement geometry = SelectChild(visual, "geometry");
                if (geometry == null)
                {
                    ctx.data.hasUnsupportedElements = true;
                    ctx.data.messages.Add($"'{visualLabel}' に <geometry> が無い");
                    continue;
                }
                ParseGeometry(geometry, visual, pose, baseDir, ctx, visualLabel);
            }
        }

        private static void ParseGeometry(
            XmlElement geometry, XmlElement visual, Pose pose, string baseDir,
            Context ctx, string label)
        {
            XmlElement box = SelectChild(geometry, "box");
            XmlElement cylinder = SelectChild(geometry, "cylinder");
            XmlElement sphere = SelectChild(geometry, "sphere");
            XmlElement plane = SelectChild(geometry, "plane");
            XmlElement mesh = SelectChild(geometry, "mesh");

            var obj = new SdfSceneObject { sourceName = label };
            ApplyMaterial(visual, obj);

            if (box != null)
            {
                Vector3 size = ParseVector3(SelectChild(box, "size")?.InnerText, Vector3.one);
                obj.type = "Cube";
                obj.scale = Ros2UnityScale(size);
                SetPose(obj, pose);
            }
            else if (cylinder != null)
            {
                float radius = ParseFloat(SelectChild(cylinder, "radius")?.InnerText, 0.5f);
                float length = ParseFloat(SelectChild(cylinder, "length")?.InnerText, 1f);
                obj.type = "Cylinder";
                // Unity の Cylinder は直径 1 × 高さ 2 (Y 軸)。SDF の円柱軸 Z は
                // Unity の Y へ写るので、向きは pose の変換だけで合う。
                obj.scale = new Vector3(radius * 2f, length * 0.5f, radius * 2f);
                SetPose(obj, pose);
            }
            else if (sphere != null)
            {
                float radius = ParseFloat(SelectChild(sphere, "radius")?.InnerText, 0.5f);
                obj.type = "Sphere";
                obj.scale = Vector3.one * (radius * 2f);
                SetPose(obj, pose);
            }
            else if (plane != null)
            {
                Vector2 size = ParseVector2(SelectChild(plane, "size")?.InnerText, Vector2.one);
                Vector3 normal = ParseVector3(SelectChild(plane, "normal")?.InnerText, new Vector3(0, 0, 1));
                obj.type = "Plane";
                // Unity の Plane は 10×10 m (XZ 面、法線 +Y)。
                obj.scale = new Vector3(size.y / 10f, 1f, size.x / 10f);
                obj.position = Ros2Unity(pose.position);
                Quaternion align = Quaternion.FromToRotation(Vector3.up, Ros2Unity(normal).normalized);
                obj.rotationEuler = (Ros2Unity(pose.rotation) * align).eulerAngles;
            }
            else if (mesh != null)
            {
                string uri = SelectChild(mesh, "uri")?.InnerText?.Trim();
                if (!TryResolveMeshPath(uri, baseDir, ctx.modelSearchRoots, out string meshPath))
                {
                    ctx.data.hasMissingAssets = true;
                    ctx.data.messages.Add($"'{label}' のメッシュ '{uri}' が見つからない");
                    return;
                }
                obj.type = "RosMesh";
                obj.meshPath = meshPath;
                obj.scale = Ros2UnityScale(ParseVector3(SelectChild(mesh, "scale")?.InnerText, Vector3.one));
                SetPose(obj, pose);
            }
            else
            {
                XmlElement first = null;
                foreach (XmlNode n in geometry.ChildNodes)
                {
                    if (n is XmlElement e) { first = e; break; }
                }
                ctx.data.hasUnsupportedElements = true;
                ctx.data.messages.Add($"'{label}' の未対応 geometry <{first?.Name ?? "?"}> を読み飛ばした");
                return;
            }

            ctx.data.objects.Add(obj);
        }

        private static void SetPose(SdfSceneObject obj, Pose pose)
        {
            obj.position = Ros2Unity(pose.position);
            obj.rotationEuler = Ros2Unity(pose.rotation).eulerAngles;
        }

        private static void ApplyMaterial(XmlElement visual, SdfSceneObject obj)
        {
            XmlElement material = SelectChild(visual, "material");
            if (material == null)
            {
                return;
            }
            // diffuse を優先、無ければ ambient。script (Gazebo/Grey 等) は素材定義が
            // Gazebo 側にしか無いので解釈しない。
            string rgba = SelectChild(material, "diffuse")?.InnerText
                ?? SelectChild(material, "ambient")?.InnerText;
            if (rgba == null)
            {
                return;
            }
            float[] c = ParseFloats(rgba);
            if (c.Length >= 3)
            {
                obj.hasColor = true;
                obj.color = new Color(c[0], c[1], c[2], c.Length >= 4 ? c[3] : 1f);
            }
        }

        private static bool TryResolveMeshPath(
            string uri, string baseDir, IReadOnlyList<string> roots, out string meshPath)
        {
            meshPath = null;
            if (string.IsNullOrEmpty(uri))
            {
                return false;
            }
            const string modelScheme = "model://";
            if (uri.StartsWith(modelScheme, StringComparison.OrdinalIgnoreCase))
            {
                string relative = uri.Substring(modelScheme.Length);
                // model://name/meshes/x.stl → 検索ルート直下の name/ から辿る
                foreach (string root in roots)
                {
                    if (string.IsNullOrEmpty(root))
                    {
                        continue;
                    }
                    string candidate = Path.Combine(root, relative);
                    if (File.Exists(candidate))
                    {
                        meshPath = Path.GetFullPath(candidate);
                        return true;
                    }
                }
                return false;
            }
            if (uri.StartsWith("file://", StringComparison.OrdinalIgnoreCase))
            {
                string path = new Uri(uri).LocalPath;
                if (File.Exists(path))
                {
                    meshPath = path;
                    return true;
                }
                return false;
            }
            if (Path.IsPathRooted(uri))
            {
                if (File.Exists(uri))
                {
                    meshPath = uri;
                    return true;
                }
                return false;
            }
            if (!string.IsNullOrEmpty(baseDir))
            {
                string candidate = Path.Combine(baseDir, uri);
                if (File.Exists(candidate))
                {
                    meshPath = Path.GetFullPath(candidate);
                    return true;
                }
            }
            return false;
        }

        // ====================================================================
        // actor (動く障害物)
        // ====================================================================

        /// <summary>
        /// <actor> のサブセット: <link> の形状 + <script><trajectory> のウェイポイント
        /// 補間で動く障害物にする。スケルタルアニメーション (skin/animation) は扱わない。
        /// </summary>
        private static void ParseActor(XmlElement actor, string baseDir, Context ctx)
        {
            string name = actor.GetAttribute("name");

            XmlElement link = SelectChild(actor, "link");
            if (link == null)
            {
                ctx.data.hasUnsupportedElements = true;
                ctx.data.messages.Add($"actor '{name}' に <link> が無い (skin のみの actor は未対応)");
                return;
            }

            // ウェイポイントを先に読む (無ければ静止した景観として置くだけ)
            var waypoints = new List<SdfMotionWaypoint>();
            bool loop = true;
            XmlElement script = SelectChild(actor, "script");
            if (script != null)
            {
                XmlElement loopEl = SelectChild(script, "loop");
                if (loopEl != null)
                {
                    loop = ParseBool(loopEl, true);
                }
                XmlElement trajectory = SelectChild(script, "trajectory");
                if (trajectory != null)
                {
                    foreach (XmlElement wp in SelectChildren(trajectory, "waypoint"))
                    {
                        float time = ParseFloat(SelectChild(wp, "time")?.InnerText, -1f);
                        float[] p = ParseFloats(SelectChild(wp, "pose")?.InnerText);
                        if (time < 0f || p.Length < 3)
                        {
                            continue;
                        }
                        float yawRos = p.Length >= 6 ? p[5] : 0f;
                        waypoints.Add(new SdfMotionWaypoint
                        {
                            position = Ros2Unity(new Vector3(p[0], p[1], p[2])),
                            // ROS の +yaw (Z 上向き, 反時計) は Unity では Y まわり -deg
                            yawDeg = -yawRos * Mathf.Rad2Deg,
                            time = time,
                        });
                    }
                }
            }
            if (!loop)
            {
                // 一度きりの再生は未対応。周回として扱う (テスト用途では実害が薄い)
                ctx.data.messages.Add($"actor '{name}' の loop=false は未対応 (周回として扱う)");
            }

            int before = ctx.data.objects.Count;
            Pose startPose = waypoints.Count > 0
                ? new Pose
                {
                    position = Vector3.zero,
                    rotation = Quaternion.identity
                }
                : ParsePose(SelectChild(actor, "pose"), name, ctx);
            ParseLink(link, startPose, baseDir, ctx, name);

            if (waypoints.Count >= 2)
            {
                // 動きは最初の形状オブジェクトに載せる。複数 visual の actor は
                // 個別に絶対座標へ動かせないので、2 個目以降は静止のまま残す。
                if (ctx.data.objects.Count > before)
                {
                    ctx.data.objects[before].motionWaypoints = waypoints;
                    if (ctx.data.objects.Count > before + 1)
                    {
                        ctx.data.messages.Add(
                            $"actor '{name}' の 2 個目以降の visual は動かない (最初の形状のみ trajectory に追従)");
                    }
                }
            }
            else if (script != null)
            {
                ctx.data.messages.Add($"actor '{name}' に有効な trajectory が無いので静止させた");
            }
        }

        // ====================================================================
        // light
        // ====================================================================

        private static void ParseLight(XmlElement light, Context ctx)
        {
            string name = light.GetAttribute("name");
            string type = light.GetAttribute("type");
            var obj = new SdfSceneObject { sourceName = name };
            switch (type)
            {
                case "directional":
                    obj.type = "Directional Light";
                    break;
                case "point":
                    obj.type = "Point Light";
                    break;
                case "spot":
                    obj.type = "Spot Light";
                    break;
                default:
                    ctx.data.hasUnsupportedElements = true;
                    ctx.data.messages.Add($"light '{name}' の未対応 type '{type}' を読み飛ばした");
                    return;
            }

            Pose pose = ParsePose(SelectChild(light, "pose"), name, ctx);
            obj.position = Ros2Unity(pose.position);

            // directional / spot の向きは <direction> (ワールド系のベクトル) が正。
            XmlElement direction = SelectChild(light, "direction");
            if (direction != null)
            {
                Vector3 dir = Ros2Unity(ParseVector3(direction.InnerText, new Vector3(0, 0, -1)));
                if (dir.sqrMagnitude > 1e-6f)
                {
                    obj.rotationEuler = Quaternion.LookRotation(dir.normalized).eulerAngles;
                }
            }
            else
            {
                obj.rotationEuler = Ros2Unity(pose.rotation).eulerAngles;
            }

            string rgba = SelectChild(light, "diffuse")?.InnerText;
            if (rgba != null)
            {
                float[] c = ParseFloats(rgba);
                if (c.Length >= 3)
                {
                    obj.hasColor = true;
                    obj.color = new Color(c[0], c[1], c[2], 1f);
                }
            }
            ctx.data.objects.Add(obj);
        }

        // ====================================================================
        // 低レベルの解釈と座標変換
        // ====================================================================

        private static XmlElement SelectChild(XmlElement parent, string name)
        {
            foreach (XmlNode node in parent.ChildNodes)
            {
                if (node is XmlElement el && el.Name == name)
                {
                    return el;
                }
            }
            return null;
        }

        private static List<XmlElement> SelectChildren(XmlElement parent, string name)
        {
            var result = new List<XmlElement>();
            foreach (XmlNode node in parent.ChildNodes)
            {
                if (node is XmlElement el && el.Name == name)
                {
                    result.Add(el);
                }
            }
            return result;
        }

        private static Pose ParsePose(XmlElement poseEl, string label, Context ctx)
        {
            if (poseEl == null)
            {
                return Pose.Identity;
            }
            string relativeTo = poseEl.GetAttribute("relative_to");
            if (!string.IsNullOrEmpty(relativeTo))
            {
                // フレームグラフまでは追わない。親要素基準として読む。
                ctx.data.messages.Add(
                    $"'{label}' の pose relative_to='{relativeTo}' は未対応 (親要素基準として解釈)");
            }
            float[] v = ParseFloats(poseEl.InnerText);
            if (v.Length < 6)
            {
                if (v.Length >= 3)
                {
                    return new Pose
                    {
                        position = new Vector3(v[0], v[1], v[2]),
                        rotation = Quaternion.identity
                    };
                }
                return Pose.Identity;
            }
            return new Pose
            {
                position = new Vector3(v[0], v[1], v[2]),
                rotation = RpyToQuaternion(v[3], v[4], v[5])
            };
        }

        /// <summary>ROS/SDF の RPY (固定軸 XYZ 回り、ラジアン) → ROS 系の四元数。</summary>
        private static Quaternion RpyToQuaternion(float roll, float pitch, float yaw)
        {
            double cr = Math.Cos(roll * 0.5), sr = Math.Sin(roll * 0.5);
            double cp = Math.Cos(pitch * 0.5), sp = Math.Sin(pitch * 0.5);
            double cy = Math.Cos(yaw * 0.5), sy = Math.Sin(yaw * 0.5);
            return new Quaternion(
                (float)(sr * cp * cy - cr * sp * sy),
                (float)(cr * sp * cy + sr * cp * sy),
                (float)(cr * cp * sy - sr * sp * cy),
                (float)(cr * cp * cy + sr * sp * sy));
        }

        // URDF-Importer の BuiltInExtensions.Ros2Unity と同じ変換。
        private static Vector3 Ros2Unity(Vector3 v) => new Vector3(-v.y, v.z, v.x);
        private static Quaternion Ros2Unity(Quaternion q) => new Quaternion(q.y, -q.z, -q.x, q.w);
        private static Vector3 Ros2UnityScale(Vector3 v) => new Vector3(v.y, v.z, v.x);

        private static bool ParseBool(XmlElement el, bool fallback)
        {
            if (el == null)
            {
                return fallback;
            }
            string t = el.InnerText.Trim().ToLowerInvariant();
            return t == "1" || t == "true";
        }

        private static float[] ParseFloats(string text)
        {
            if (string.IsNullOrWhiteSpace(text))
            {
                return Array.Empty<float>();
            }
            string[] parts = text.Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
            var result = new List<float>(parts.Length);
            foreach (string part in parts)
            {
                if (float.TryParse(part, NumberStyles.Float, CultureInfo.InvariantCulture, out float f))
                {
                    result.Add(f);
                }
                else
                {
                    return Array.Empty<float>();
                }
            }
            return result.ToArray();
        }

        private static float ParseFloat(string text, float fallback)
        {
            if (text != null && float.TryParse(text.Trim(), NumberStyles.Float,
                    CultureInfo.InvariantCulture, out float f))
            {
                return f;
            }
            return fallback;
        }

        private static Vector3 ParseVector3(string text, Vector3 fallback)
        {
            float[] v = ParseFloats(text);
            return v.Length >= 3 ? new Vector3(v[0], v[1], v[2]) : fallback;
        }

        private static Vector2 ParseVector2(string text, Vector2 fallback)
        {
            float[] v = ParseFloats(text);
            return v.Length >= 2 ? new Vector2(v[0], v[1]) : fallback;
        }
    }
}
