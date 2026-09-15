using System;
using System.Collections.Generic;
using System.Globalization;
using System.Xml;
using UnityEngine;
using UnitySensors.DataType.Sensor;
using UnitySensors.Sensor.MagneticGuide;

namespace UrdfProperties
{
    /// <summary>
    /// URDF の独自要素 <c>&lt;collision_material&gt;</c> を読んで、対応するコライダへ
    /// 摩擦係数と contact offset を設定する。<c>&lt;sensor_only&gt;</c> を持つ定義は
    /// コライダをトリガ (接触応答なし) にする。
    /// </summary>
    /// <remarks>
    /// URDF Importer はこの要素を知らないので、インポート後にこちらで適用する。
    /// 定義は <c>&lt;robot&gt;</c> 直下に置き、各 <c>&lt;collision&gt;</c> から名前で参照する:
    ///
    /// <code>
    /// &lt;robot&gt;
    ///   &lt;collision_material name="wheel"&gt;
    ///     &lt;friction static="1.0" dynamic="1.0" combine="maximum"/&gt;
    ///     &lt;contact_offset value="0.02"/&gt;
    ///   &lt;/collision_material&gt;
    ///   &lt;collision_material name="weed"&gt;
    ///     &lt;sensor_only value="true"/&gt;   // LiDAR には映るが、触れても押し返さない
    ///   &lt;/collision_material&gt;
    ///   &lt;link name="wheel"&gt;
    ///     &lt;collision&gt;... &lt;collision_material name="wheel"/&gt;&lt;/collision&gt;
    ///   &lt;/link&gt;
    /// &lt;/robot&gt;
    /// </code>
    ///
    /// <para><b>combine について</b>: Unity は接触する 2 つのマテリアルのうち
    /// 列挙値が大きいほうの combine を採用する (Average &lt; Minimum &lt; Multiply &lt;
    /// Maximum)。既定の Average では、床のマテリアルとの平均が実効値になるため
    /// <c>static="1.0"</c> と書いても 1.0 にはならない。相手によらず指定値を効かせたい
    /// 場合は <c>combine="maximum"</c> を指定すること。</para>
    ///
    /// <para><b>sensor_only について</b>: Unity の <c>Collider.isTrigger</c> を立てる。
    /// トリガは物理接触を起こさないが、プロジェクト設定 <c>Queries Hit Triggers</c> が
    /// 有効なのでレイキャスト (UnitySensors の LiDAR など) には当たる。雑草のように
    /// 「センサには見えるが走行を妨げない」物体に使う。トリガにできるのは convex な
    /// コライダだけなので、非 convex の MeshCollider は convex に変えてから立てる。</para>
    ///
    /// <para><b>magnetic_tape について</b>: <c>&lt;magnetic_tape polarity="track|marker"/&gt;</c>
    /// はコライダに <see cref="MagneticTape"/> を付け、磁気誘導センサ
    /// (<see cref="MagneticGuideSensor"/>) から見える磁気テープにする。テープは踏んで走る
    /// ものなので sensor_only も含意する (トリガになる)。</para>
    /// </remarks>
    public static class CollisionMaterialApplier
    {
        /// <summary>1 つの &lt;collision_material&gt; 定義。</summary>
        public class Definition
        {
            public string Name;
            public float StaticFriction;
            public float DynamicFriction;
            public PhysicsMaterialCombine FrictionCombine = PhysicsMaterialCombine.Average;
            public bool HasContactOffset;
            public float ContactOffset;
            /// <summary>true ならコライダをトリガにする (センサにだけ見える物体)。</summary>
            public bool SensorOnly;
            /// <summary>true なら磁気テープ。<see cref="TapePolarity"/> が極性。sensor_only を含意する。</summary>
            public bool IsMagneticTape;
            public MagneticPolarity TapePolarity = MagneticPolarity.Track;
        }

        /// <summary>&lt;robot&gt; 直下の定義をすべて読む。</summary>
        public static List<Definition> ParseDefinitions(XmlNode robotNode)
        {
            var definitions = new List<Definition>();
            if (robotNode == null)
            {
                return definitions;
            }

            XmlNodeList nodes = robotNode.SelectNodes("collision_material");
            if (nodes == null || nodes.Count == 0)
            {
                // 旧称。既存の URDF を読めなくしないために残してある。
                nodes = robotNode.SelectNodes("physics_material");
                if (nodes != null && nodes.Count > 0)
                {
                    Debug.LogWarning("<physics_material> is deprecated. Use <collision_material> instead.");
                }
            }
            if (nodes == null)
            {
                return definitions;
            }

            foreach (XmlNode node in nodes)
            {
                var definition = new Definition { Name = node.Attributes?["name"]?.Value };
                if (string.IsNullOrEmpty(definition.Name))
                {
                    Debug.LogWarning("[CollisionMaterial] name の無い定義を読み飛ばした");
                    continue;
                }

                XmlNode friction = node.SelectSingleNode("friction");
                if (friction != null)
                {
                    definition.StaticFriction = ParseFloat(friction.Attributes?["static"]?.Value, 0f);
                    definition.DynamicFriction = ParseFloat(friction.Attributes?["dynamic"]?.Value, 0f);
                    definition.FrictionCombine = ParseCombine(friction.Attributes?["combine"]?.Value);
                }

                XmlNode contactOffset = node.SelectSingleNode("contact_offset");
                if (contactOffset != null)
                {
                    definition.HasContactOffset = true;
                    definition.ContactOffset = ParseFloat(contactOffset.Attributes?["value"]?.Value, 0f);
                }

                XmlNode sensorOnly = node.SelectSingleNode("sensor_only");
                if (sensorOnly != null)
                {
                    // 要素があれば true。value="false" で明示的に打ち消せる。
                    definition.SensorOnly = ParseBool(sensorOnly.Attributes?["value"]?.Value, true);
                }

                XmlNode magneticTape = node.SelectSingleNode("magnetic_tape");
                if (magneticTape != null)
                {
                    definition.IsMagneticTape = true;
                    definition.SensorOnly = true;
                    definition.TapePolarity = ParsePolarity(magneticTape.Attributes?["polarity"]?.Value);
                }

                definitions.Add(definition);
            }
            return definitions;
        }

        /// <summary>
        /// インポート済みのロボットへ定義を適用する。適用できたコライダの数を返す。
        /// </summary>
        /// <remarks>
        /// URDF の <c>&lt;collision&gt;</c> は 1 リンクに複数書けて、Importer はその順に
        /// <c>Collisions</c> の子を作る。i 番目の <c>&lt;collision&gt;</c> は i 番目の子に
        /// 対応させ、その配下のコライダすべてへ設定する。1 つ目のコライダだけを見る作りに
        /// すると、複数形状のリンクで一部だけ摩擦が効かない状態になり、しかも黙って
        /// そうなるので気づけない。
        /// </remarks>
        public static int Apply(GameObject robotRoot, XmlNode robotNode)
        {
            if (robotRoot == null || robotNode == null)
            {
                return 0;
            }

            List<Definition> definitions = ParseDefinitions(robotNode);
            if (definitions.Count == 0)
            {
                return 0;
            }

            var byName = new Dictionary<string, Definition>();
            foreach (Definition definition in definitions)
            {
                byName[definition.Name] = definition;
            }

            int applied = 0;
            XmlNodeList links = robotNode.SelectNodes("link");
            if (links == null)
            {
                return 0;
            }

            foreach (XmlNode link in links)
            {
                string linkName = link.Attributes?["name"]?.Value;
                if (string.IsNullOrEmpty(linkName))
                {
                    continue;
                }

                XmlNodeList collisions = link.SelectNodes("collision");
                if (collisions == null || collisions.Count == 0)
                {
                    continue;
                }

                Transform linkTransform = FindInChildrenByName(robotRoot.transform, linkName);
                if (linkTransform == null)
                {
                    Debug.LogWarning($"[CollisionMaterial] link '{linkName}' がシーンに見つからない");
                    continue;
                }
                Transform collisionsRoot = linkTransform.Find("Collisions");
                if (collisionsRoot == null)
                {
                    continue;
                }

                for (int i = 0; i < collisions.Count; i++)
                {
                    string materialName = MaterialNameOf(collisions[i]);
                    if (string.IsNullOrEmpty(materialName))
                    {
                        continue;
                    }
                    Definition definition;
                    if (!byName.TryGetValue(materialName, out definition))
                    {
                        Debug.LogWarning(
                            $"[CollisionMaterial] link '{linkName}' が未定義の '{materialName}' を参照している");
                        continue;
                    }
                    if (i >= collisionsRoot.childCount)
                    {
                        Debug.LogWarning(
                            $"[CollisionMaterial] link '{linkName}' の {i} 番目の collision に対応する " +
                            "オブジェクトが無い");
                        continue;
                    }

                    applied += ApplyTo(collisionsRoot.GetChild(i), definition, linkName);
                }
            }
            return applied;
        }

        static int ApplyTo(Transform collisionObject, Definition definition, string linkName)
        {
            var material = new PhysicsMaterial(definition.Name)
            {
                staticFriction = definition.StaticFriction,
                dynamicFriction = definition.DynamicFriction,
                frictionCombine = definition.FrictionCombine
            };

            int applied = 0;
            foreach (Collider collider in collisionObject.GetComponentsInChildren<Collider>())
            {
                collider.material = material;
                if (definition.HasContactOffset)
                {
                    collider.contactOffset = definition.ContactOffset;
                }
                if (definition.SensorOnly)
                {
                    MakeSensorOnly(collider, definition.Name, linkName);
                }
                if (definition.IsMagneticTape)
                {
                    MagneticTape tape = collider.gameObject.GetComponent<MagneticTape>()
                                        ?? collider.gameObject.AddComponent<MagneticTape>();
                    tape.polarity = definition.TapePolarity;
                }
                applied++;
            }

            if (applied == 0)
            {
                Debug.LogWarning(
                    $"[CollisionMaterial] link '{linkName}' の collision にコライダが無く " +
                    $"'{definition.Name}' を適用できなかった");
            }
            else
            {
                Debug.Log($"[CollisionMaterial] Applied '{definition.Name}' to '{linkName}' " +
                          $"({applied} collider(s), static={definition.StaticFriction}, " +
                          $"dynamic={definition.DynamicFriction}, combine={definition.FrictionCombine}" +
                          (definition.SensorOnly ? ", sensor_only" : "") +
                          (definition.IsMagneticTape ? ", magnetic_tape=" + definition.TapePolarity : "") + ")");
            }
            return applied;
        }

        /// <summary>
        /// コライダをトリガにする。Unity は非 convex の MeshCollider をトリガにできない
        /// (設定しても効かず、エラーログが出る) ので、その場合は先に convex へ変える。
        /// URDF Importer が作るコライダは通常 convex なので、ここに来るのは import 設定を
        /// 変えたときだけ。
        /// </summary>
        static void MakeSensorOnly(Collider collider, string materialName, string linkName)
        {
            var meshCollider = collider as MeshCollider;
            if (meshCollider != null && !meshCollider.convex)
            {
                Debug.LogWarning(
                    $"[CollisionMaterial] link '{linkName}' の MeshCollider は convex でないため " +
                    $"'{materialName}' (sensor_only) の適用にあたり convex に変更した。形状は凸包になる");
                meshCollider.convex = true;
            }
            collider.isTrigger = true;
        }

        /// <summary>
        /// ロボットのコライダがすべてトリガか (1 つも無い場合も true)。
        /// 呼び出し側は、これが true でルートが immovable でなければ落下し続けるので警告する。
        /// </summary>
        public static bool AllCollidersAreSensorOnly(GameObject robotRoot)
        {
            if (robotRoot == null)
            {
                return true;
            }
            foreach (Collider collider in robotRoot.GetComponentsInChildren<Collider>(true))
            {
                if (!collider.isTrigger)
                {
                    return false;
                }
            }
            return true;
        }

        static string MaterialNameOf(XmlNode collisionNode)
        {
            XmlNode reference = collisionNode.SelectSingleNode("collision_material")
                                ?? collisionNode.SelectSingleNode("physics_material");
            return reference?.Attributes?["name"]?.Value;
        }

        static PhysicsMaterialCombine ParseCombine(string value)
        {
            switch (value?.ToLowerInvariant())
            {
                case "multiply": return PhysicsMaterialCombine.Multiply;
                case "maximum": return PhysicsMaterialCombine.Maximum;
                case "minimum": return PhysicsMaterialCombine.Minimum;
                case "average": return PhysicsMaterialCombine.Average;
                case null: return PhysicsMaterialCombine.Average;
                default:
                    Debug.LogWarning($"[CollisionMaterial] 未知の combine '{value}'。average として扱う");
                    return PhysicsMaterialCombine.Average;
            }
        }

        static MagneticPolarity ParsePolarity(string value)
        {
            switch (value?.Trim().ToLowerInvariant())
            {
                case "marker": return MagneticPolarity.Marker;
                case "track": case null: case "": return MagneticPolarity.Track;
                default:
                    Debug.LogWarning($"[CollisionMaterial] 未知の polarity '{value}'。track として扱う");
                    return MagneticPolarity.Track;
            }
        }

        static bool ParseBool(string value, bool fallback)
        {
            switch (value?.Trim().ToLowerInvariant())
            {
                case "true": case "1": case "yes": return true;
                case "false": case "0": case "no": return false;
                case null: case "": return fallback;
                default:
                    Debug.LogWarning($"[CollisionMaterial] 真偽値として読めない '{value}'。{fallback} として扱う");
                    return fallback;
            }
        }

        static float ParseFloat(string value, float fallback)
        {
            float parsed;
            return float.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out parsed)
                ? parsed
                : fallback;
        }

        static Transform FindInChildrenByName(Transform parent, string name)
        {
            if (parent.name == name)
            {
                return parent;
            }
            foreach (Transform child in parent)
            {
                Transform found = FindInChildrenByName(child, name);
                if (found != null)
                {
                    return found;
                }
            }
            return null;
        }
    }
}
