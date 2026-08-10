using System;
using System.Collections.Generic;
using System.Globalization;
using System.Xml;
using UnityEngine;

namespace UrdfProperties
{
    /// <summary>
    /// URDF の独自要素 <c>&lt;collision_material&gt;</c> を読んで、対応するコライダへ
    /// 摩擦係数と contact offset を設定する。
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
                          $"dynamic={definition.DynamicFriction}, combine={definition.FrictionCombine})");
            }
            return applied;
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
