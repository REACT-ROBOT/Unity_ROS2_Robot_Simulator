using System;
using System.Collections.Generic;
using System.IO;
using System.Text.RegularExpressions;
using UnityEngine;

using Unity.Robotics.Core;
using Unity.Robotics.UrdfImporter;

using RosMessageTypes.Geometry;
using RosMessageTypes.SimulationInterfaces;
using RosMessageTypes.Std;

/// <summary>
/// simulation_interfaces のエンティティ系サービスと、スポーン可能リソース /
/// 名前付き姿勢の問い合わせ。
/// </summary>
/// <remarks>
/// ここで言う「エンティティ」は spawn_entity / spawn_entities で生成したものだけ
/// (m_EntityList の中身)。UI から置いた床や障害物はワールド側の景観として扱い、
/// エンティティには含めない。そうしないと DeleteEntity や SetEntityState の対象が
/// 「ROS から作ったもの」と「GUI で置いたもの」で二重になり、LoadWorld が景観を
/// 差し替えるという整理と噛み合わなくなる。
/// </remarks>
public partial class SimulationControl
{
    // エンティティごとの EntityInfo (category / description / tags)。
    // スポーン時に既定値を入れ、SetEntityInfo で上書きする。
    private Dictionary<string, EntityInfoMsg> m_EntityInfo = new Dictionary<string, EntityInfoMsg>();

    /// <summary>ルートボディの速度と、そこから差分で求めた加速度。</summary>
    private struct EntityMotion
    {
        public Vector3 LinearVelocity;
        public Vector3 AngularVelocity;
        public Vector3 LinearAcceleration;
        public Vector3 AngularAcceleration;
        public bool HasPrevious;
    }

    // 加速度は Unity から直接読めないので、物理ステップごとに速度の差分を取る。
    private Dictionary<string, EntityMotion> m_EntityMotion = new Dictionary<string, EntityMotion>();

    /// <summary>
    /// 各エンティティのルートボディの加速度を、速度の 1 階差分として更新する。
    /// </summary>
    /// <remarks>
    /// FixedUpdate なので刻みは fixedDeltaTime で一定。平滑化はしていない
    /// (生の差分をそのまま返す) ため、接触の瞬間などは大きく振れる。均した値が
    /// 要るなら受け取り側で処理する前提。timeScale = 0 の間は FixedUpdate 自体が
    /// 回らないので、停止中は最後に計算した値がそのまま残る。
    /// </remarks>
    private void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;
        if (dt <= 0f)
        {
            return;
        }

        foreach (GameObject entity in m_EntityList)
        {
            if (entity == null)
            {
                continue;
            }
            ArticulationBody body = GetEntityRootBody(entity);
            if (body == null)
            {
                continue;
            }

            Vector3 linear = body.linearVelocity;
            Vector3 angular = body.angularVelocity;

            EntityMotion motion;
            m_EntityMotion.TryGetValue(entity.name, out motion);
            if (motion.HasPrevious)
            {
                motion.LinearAcceleration = (linear - motion.LinearVelocity) / dt;
                motion.AngularAcceleration = (angular - motion.AngularVelocity) / dt;
            }
            motion.LinearVelocity = linear;
            motion.AngularVelocity = angular;
            motion.HasPrevious = true;
            m_EntityMotion[entity.name] = motion;
        }
    }

    // GetSpawnables が拾うファイル。MeshImporter が読めるものと URDF。
    private static readonly string[] k_SpawnableExtensions =
        { ".urdf", ".obj", ".stl", ".dae", ".fbx", ".ply" };

    // 走査するファイル数の上限。設定を間違えて / を指したときに固まらないための保険で、
    // 打ち切ったときは黙って減らさず error_message に出す。
    private const int k_MaxScannedFiles = 2000;

    /// <summary>
    /// ディレクトリ走査の打ち切り管理。複数のソースをまたいで 1 つを共有する。
    /// </summary>
    /// <remarks>
    /// 拾えた件数ではなく「見たファイル数」を数える。拡張子で弾いたものを数えないと、
    /// 巨大なツリーを指されたときに上限が効かない。
    /// </remarks>
    private class ScanBudget
    {
        public int Remaining = k_MaxScannedFiles;
        public bool Exhausted;
    }

    // ====================================================================
    // エンティティの問い合わせ
    // ====================================================================

    /// <summary>get_entities サービス。フィルタに合うエンティティ名を返す。</summary>
    private GetEntitiesResponse GetEntities(GetEntitiesRequest request)
    {
        var response = new GetEntitiesResponse();
        var matched = new List<GameObject>();
        if (!TryFilterEntities(request.filters, matched, out ResultMsg error))
        {
            response.result = error;
            response.entities = Array.Empty<string>();
            return response;
        }

        response.result.result = ResultMsg.RESULT_OK;
        response.entities = new string[matched.Count];
        for (int i = 0; i < matched.Count; i++)
        {
            response.entities[i] = matched[i].name;
        }
        return response;
    }

    /// <summary>get_entities_states サービス。get_entities に状態を足しただけ。</summary>
    private GetEntitiesStatesResponse GetEntitiesStates(GetEntitiesStatesRequest request)
    {
        var response = new GetEntitiesStatesResponse();
        var matched = new List<GameObject>();
        if (!TryFilterEntities(request.filters, matched, out ResultMsg error))
        {
            response.result = error;
            response.entities = Array.Empty<string>();
            response.states = Array.Empty<EntityStateMsg>();
            return response;
        }

        response.result.result = ResultMsg.RESULT_OK;
        response.entities = new string[matched.Count];
        response.states = new EntityStateMsg[matched.Count];
        for (int i = 0; i < matched.Count; i++)
        {
            response.entities[i] = matched[i].name;
            response.states[i] = BuildEntityState(matched[i]);
        }
        return response;
    }

    /// <summary>get_entity_state サービス。</summary>
    private GetEntityStateResponse GetEntityState(GetEntityStateRequest request)
    {
        var response = new GetEntityStateResponse();
        if (!TryFindEntity(request.entity, out GameObject entity))
        {
            response.result.result = ResultMsg.RESULT_NOT_FOUND;
            response.result.error_message = $"No entity named '{request.entity}'";
            response.state = new EntityStateMsg();
            return response;
        }

        response.result.result = ResultMsg.RESULT_OK;
        response.state = BuildEntityState(entity);
        return response;
    }

    /// <summary>
    /// set_entity_state サービス。姿勢・速度を即座に書き換える。
    /// </summary>
    /// <remarks>
    /// acceleration は Unity 側に「加速度を与える」入口が無い (力を与えるしかなく、
    /// 質量が分からないと等価にならない) ため無視する。EntityState.msg が
    /// 「シミュレータによっては無視される」と明記している項目なので、
    /// 要求されたら結果は OK のまま error_message に無視した旨を残す。
    /// </remarks>
    private SetEntityStateResponse SetEntityState(SetEntityStateRequest request)
    {
        var response = new SetEntityStateResponse();
        if (!TryFindEntity(request.entity, out GameObject entity))
        {
            response.result.result = ResultMsg.RESULT_NOT_FOUND;
            response.result.error_message = $"No entity named '{request.entity}'";
            return response;
        }

        EntityStateMsg state = request.state ?? new EntityStateMsg();
        ArticulationBody rootBody = GetEntityRootBody(entity);

        if (request.set_pose)
        {
            if (state.pose == null || !IsValidPose(state.pose))
            {
                response.result.result = SetEntityStateResponse.INVALID_POSE;
                response.result.error_message =
                    "pose is invalid (non-finite value or non-unit quaternion)";
                return response;
            }

            Vector3 position = RosToUnityPosition(state.pose.position);
            Quaternion rotation = RosToUnityRotation(state.pose.orientation);
            entity.transform.position = position;
            entity.transform.rotation = rotation;
            if (rootBody != null)
            {
                // ArticulationBody はソルバが姿勢を持つので transform を書いても動かない。
                // ReSetAllEntitiesState と同じく TeleportRoot 経由で入れる。
                rootBody.TeleportRoot(position, rotation);
                rootBody.PublishTransform();
            }
        }

        if (request.set_twist)
        {
            if (state.twist == null)
            {
                response.result.result = ResultMsg.RESULT_OPERATION_FAILED;
                response.result.error_message = "set_twist was requested but twist is empty";
                return response;
            }

            Vector3 linear = RosToUnityVector(state.twist.linear);
            Vector3 angular = RosToUnityAngular(state.twist.angular);
            if (rootBody == null)
            {
                // 速度を持てない静的オブジェクトにゼロ以外を要求されたら失敗、
                // というのが SetEntityState.srv の規定。
                if (linear.sqrMagnitude > 0f || angular.sqrMagnitude > 0f)
                {
                    response.result.result = ResultMsg.RESULT_OPERATION_FAILED;
                    response.result.error_message =
                        $"Entity '{entity.name}' is static; a non-zero twist cannot be set";
                    return response;
                }
            }
            else
            {
                rootBody.linearVelocity = linear;
                rootBody.angularVelocity = angular;
            }
        }

        response.result.result = ResultMsg.RESULT_OK;
        if (request.set_acceleration)
        {
            response.result.error_message =
                "acceleration is ignored by this simulator; pose/twist were applied";
        }
        return response;
    }

    /// <summary>get_entity_info サービス。</summary>
    private GetEntityInfoResponse GetEntityInfo(GetEntityInfoRequest request)
    {
        var response = new GetEntityInfoResponse();
        if (!TryFindEntity(request.entity, out GameObject entity))
        {
            response.result.result = ResultMsg.RESULT_NOT_FOUND;
            response.result.error_message = $"No entity named '{request.entity}'";
            response.info = NewEntityInfo(EntityCategoryMsg.CATEGORY_OBJECT, "");
            return response;
        }

        response.result.result = ResultMsg.RESULT_OK;
        response.info = GetOrCreateEntityInfo(entity.name);
        return response;
    }

    /// <summary>set_entity_info サービス。category / description / tags を丸ごと差し替える。</summary>
    private SetEntityInfoResponse SetEntityInfo(SetEntityInfoRequest request)
    {
        var response = new SetEntityInfoResponse();
        if (!TryFindEntity(request.entity, out GameObject entity))
        {
            response.result.result = ResultMsg.RESULT_NOT_FOUND;
            response.result.error_message = $"No entity named '{request.entity}'";
            return response;
        }

        EntityInfoMsg info = request.info ?? new EntityInfoMsg();
        byte category = info.category != null ? info.category.category : EntityCategoryMsg.CATEGORY_OBJECT;
        if (!IsKnownCategory(category))
        {
            response.result.result = ResultMsg.RESULT_OPERATION_FAILED;
            response.result.error_message = $"Unknown entity category {category}";
            return response;
        }

        m_EntityInfo[entity.name] = new EntityInfoMsg
        {
            category = new EntityCategoryMsg { category = category },
            description = info.description ?? "",
            tags = info.tags ?? Array.Empty<string>()
        };
        response.result.result = ResultMsg.RESULT_OK;
        return response;
    }

    /// <summary>
    /// get_entity_bounds サービス。エンティティの基準リンク座標系での AABB を返す。
    /// </summary>
    private GetEntityBoundsResponse GetEntityBounds(GetEntityBoundsRequest request)
    {
        var response = new GetEntityBoundsResponse();
        response.bounds = new BoundsMsg { type = BoundsMsg.TYPE_EMPTY, points = Array.Empty<Vector3Msg>() };

        if (!TryFindEntity(request.entity, out GameObject entity))
        {
            response.result.result = ResultMsg.RESULT_NOT_FOUND;
            response.result.error_message = $"No entity named '{request.entity}'";
            return response;
        }

        Transform frame = GetEntityFrame(entity);
        if (!TryComputeLocalBounds(entity, frame, out Vector3 min, out Vector3 max))
        {
            // 形を持たないエンティティ。Bounds.msg では TYPE_EMPTY が「境界なし」なので
            // 失敗にはせず、そのまま返す。
            response.result.result = ResultMsg.RESULT_OK;
            response.result.error_message = $"Entity '{entity.name}' has no renderable geometry";
            return response;
        }

        ToRosAxisAlignedBox(min, max, out Vector3Msg rosMin, out Vector3Msg rosMax);
        response.result.result = ResultMsg.RESULT_OK;
        // Bounds.msg の box は「upper right, lower left」の順。
        response.bounds.type = BoundsMsg.TYPE_BOX;
        response.bounds.points = new[] { rosMax, rosMin };
        return response;
    }

    /// <summary>delete_entity サービス。1 体だけデスポーンする。</summary>
    private DeleteEntityResponse DeleteEntity(DeleteEntityRequest request)
    {
        var response = new DeleteEntityResponse();
        if (!TryFindEntity(request.entity, out GameObject entity))
        {
            response.result.result = ResultMsg.RESULT_NOT_FOUND;
            response.result.error_message = $"No entity named '{request.entity}'";
            return response;
        }

        DespawnEntity(entity);
        m_EntityList.Remove(entity);
        response.result.result = ResultMsg.RESULT_OK;
        return response;
    }

    // ====================================================================
    // スポーン可能リソース / 名前付き姿勢
    // ====================================================================

    /// <summary>
    /// get_spawnables サービス。設定ファイルの spawnable_paths と要求の sources を走査する。
    /// </summary>
    /// <remarks>
    /// Unity のプレイヤーには ament のパッケージ検索パスに相当する仕組みが無いので、
    /// 「どこを見えていることにするか」は simulation_resources.json で与える
    /// (SimulationResources のコメントを参照)。設定が無ければ空リストを OK で返す。
    /// </remarks>
    private GetSpawnablesResponse GetSpawnables(GetSpawnablesRequest request)
    {
        var response = new GetSpawnablesResponse();
        var problems = new List<string>();
        var roots = new List<string>();

        foreach (string path in SimulationResources.SpawnablePaths)
        {
            AddSearchRoot(path, roots, problems, "spawnable_paths");
        }
        if (request.sources != null)
        {
            foreach (string source in request.sources)
            {
                AddSearchRoot(source, roots, problems, "sources");
            }
        }

        var spawnables = new List<SpawnableMsg>();
        var seen = new HashSet<string>();
        var budget = new ScanBudget();
        foreach (string root in roots)
        {
            foreach (string file in EnumerateFiles(root, problems, budget))
            {
                if (Array.IndexOf(k_SpawnableExtensions, Path.GetExtension(file).ToLowerInvariant()) < 0)
                {
                    continue;
                }
                if (!seen.Add(file))
                {
                    continue;
                }
                spawnables.Add(new SpawnableMsg
                {
                    entity_resource = new ResourceMsg { uri = ToFileUri(file), resource_string = "" },
                    description = DescribeSpawnable(file),
                    // 個々のモデルの大きさは読み込まないと分からないので申告しない。
                    spawn_bounds = new BoundsMsg { type = BoundsMsg.TYPE_EMPTY, points = Array.Empty<Vector3Msg>() }
                });
            }
        }

        if (budget.Exhausted)
        {
            problems.Add($"stopped after scanning {k_MaxScannedFiles} files; narrow spawnable_paths");
        }
        if (!string.IsNullOrEmpty(SimulationResources.LoadError))
        {
            problems.Add(SimulationResources.LoadError);
        }

        // GetSpawnables.srv では、読めなかったソースは error_message に出しつつ
        // 成功を妨げない。
        response.result.result = ResultMsg.RESULT_OK;
        response.result.error_message = string.Join("; ", problems);
        response.spawnables = spawnables.ToArray();
        return response;
    }

    /// <summary>get_named_poses サービス。</summary>
    private GetNamedPosesResponse GetNamedPoses(GetNamedPosesRequest request)
    {
        var response = new GetNamedPosesResponse();
        if (!IsKnownTagFilterMode(request.tags))
        {
            response.result.result = ResultMsg.RESULT_OPERATION_FAILED;
            response.result.error_message =
                $"Unknown tags filter_mode {request.tags.filter_mode}; expected FILTER_MODE_ANY(0) or FILTER_MODE_ALL(1)";
            response.poses = Array.Empty<NamedPoseMsg>();
            return response;
        }

        var poses = new List<NamedPoseMsg>();
        foreach (NamedPoseMsg pose in SimulationResources.NamedPoses)
        {
            if (MatchesTags(pose.tags, request.tags))
            {
                poses.Add(pose);
            }
        }

        response.result.result = ResultMsg.RESULT_OK;
        response.result.error_message = SimulationResources.LoadError;
        response.poses = poses.ToArray();
        return response;
    }

    /// <summary>get_named_pose_bounds サービス。</summary>
    private GetNamedPoseBoundsResponse GetNamedPoseBounds(GetNamedPoseBoundsRequest request)
    {
        var response = new GetNamedPoseBoundsResponse();
        if (!SimulationResources.TryGetNamedPoseBounds(request.name, out BoundsMsg bounds))
        {
            response.result.result = ResultMsg.RESULT_NOT_FOUND;
            response.result.error_message = $"No named pose called '{request.name}'";
            response.bounds = new BoundsMsg { type = BoundsMsg.TYPE_EMPTY, points = Array.Empty<Vector3Msg>() };
            return response;
        }

        response.result.result = ResultMsg.RESULT_OK;
        response.bounds = bounds;
        return response;
    }

    // ====================================================================
    // エンティティ台帳
    // ====================================================================

    private bool TryFindEntity(string name, out GameObject entity)
    {
        entity = null;
        if (string.IsNullOrEmpty(name))
        {
            return false;
        }
        foreach (GameObject candidate in m_EntityList)
        {
            if (candidate != null && candidate.name == name)
            {
                entity = candidate;
                return true;
            }
        }
        return false;
    }

    /// <summary>スポーン時に EntityInfo の既定値を入れる。</summary>
    private void RegisterEntityInfo(string entityName, byte category, string description)
    {
        m_EntityInfo[entityName] = NewEntityInfo(category, description);
    }

    private EntityInfoMsg GetOrCreateEntityInfo(string entityName)
    {
        if (!m_EntityInfo.TryGetValue(entityName, out EntityInfoMsg info))
        {
            info = NewEntityInfo(EntityCategoryMsg.CATEGORY_OBJECT, "");
            m_EntityInfo[entityName] = info;
        }
        return info;
    }

    private static EntityInfoMsg NewEntityInfo(byte category, string description)
    {
        return new EntityInfoMsg
        {
            category = new EntityCategoryMsg { category = category },
            description = description ?? "",
            tags = Array.Empty<string>()
        };
    }

    private static bool IsKnownCategory(byte category)
    {
        return category == EntityCategoryMsg.CATEGORY_OBJECT
            || category == EntityCategoryMsg.CATEGORY_ROBOT
            || category == EntityCategoryMsg.CATEGORY_HUMAN
            || category == EntityCategoryMsg.CATEGORY_DYNAMIC_OBJECT
            || category == EntityCategoryMsg.CATEGORY_STATIC_OBJECT;
    }

    // ====================================================================
    // フィルタ
    // ====================================================================

    /// <summary>
    /// EntityFilters を適用する。未対応のフィルタを渡されたときだけ false を返し、
    /// そのときの結果コードを error に入れる。
    /// </summary>
    private bool TryFilterEntities(EntityFiltersMsg filters, List<GameObject> matched, out ResultMsg error)
    {
        error = new ResultMsg();

        Regex nameFilter = null;
        if (filters != null && !string.IsNullOrEmpty(filters.filter))
        {
            try
            {
                // 仕様上は POSIX 拡張正規表現。.NET の Regex は上位互換なので
                // 素直な表現はそのまま通る。名前全体との一致を見るのは
                // simulation_interfaces の参照実装 (std::regex_match) に合わせたもの。
                nameFilter = new Regex(@"\A(?:" + filters.filter + @")\z");
            }
            catch (ArgumentException e)
            {
                error.result = ResultMsg.RESULT_OPERATION_FAILED;
                error.error_message = $"Invalid name filter '{filters.filter}': {e.Message}";
                return false;
            }
        }

        if (filters != null && !IsKnownTagFilterMode(filters.tags))
        {
            error.result = ResultMsg.RESULT_OPERATION_FAILED;
            error.error_message =
                $"Unknown tags filter_mode {filters.tags.filter_mode}; expected FILTER_MODE_ANY(0) or FILTER_MODE_ALL(1)";
            return false;
        }

        BoundsMsg bounds = filters != null ? filters.bounds : null;
        if (bounds != null && bounds.type != BoundsMsg.TYPE_EMPTY)
        {
            if (bounds.type != BoundsMsg.TYPE_BOX
                && bounds.type != BoundsMsg.TYPE_SPHERE
                && bounds.type != BoundsMsg.TYPE_CONVEX_HULL)
            {
                error.result = ResultMsg.RESULT_FEATURE_UNSUPPORTED;
                error.error_message =
                    $"Bounds type {bounds.type} is not supported; " +
                    "only TYPE_BOX, TYPE_SPHERE and TYPE_CONVEX_HULL are";
                return false;
            }
            // Bounds.msg では box / sphere は 2 点、凸包は「3 点以上」。
            int requiredPoints = bounds.type == BoundsMsg.TYPE_CONVEX_HULL ? 3 : 2;
            if (bounds.points == null || bounds.points.Length < requiredPoints)
            {
                error.result = ResultMsg.RESULT_OPERATION_FAILED;
                error.error_message = bounds.type == BoundsMsg.TYPE_CONVEX_HULL
                    ? "TYPE_CONVEX_HULL needs at least three points"
                    : "Bounds filter needs two points";
                return false;
            }
        }
        else
        {
            bounds = null;
        }

        foreach (GameObject entity in m_EntityList)
        {
            if (entity == null)
            {
                continue;
            }
            if (nameFilter != null && !nameFilter.IsMatch(entity.name))
            {
                continue;
            }

            EntityInfoMsg info = GetOrCreateEntityInfo(entity.name);
            if (!MatchesCategories(info.category.category, filters != null ? filters.categories : null))
            {
                continue;
            }
            if (!MatchesTags(info.tags, filters != null ? filters.tags : null))
            {
                continue;
            }
            if (bounds != null && !OverlapsBounds(entity, bounds))
            {
                continue;
            }
            matched.Add(entity);
        }
        return true;
    }

    private static bool MatchesCategories(byte category, EntityCategoryMsg[] categories)
    {
        if (categories == null || categories.Length == 0)
        {
            return true;
        }
        foreach (EntityCategoryMsg wanted in categories)
        {
            if (wanted != null && wanted.category == category)
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>
    /// TagsFilter の filter_mode が解釈できる値か。タグが空なら絞り込まないので常に true。
    /// </summary>
    /// <remarks>
    /// 未知の filter_mode を黙って ANY として扱うと、要求とは違う絞り込み結果を
    /// 「正常」として返してしまう。呼び出し側はこれで弾いてから MatchesTags を使う。
    /// </remarks>
    private static bool IsKnownTagFilterMode(TagsFilterMsg filter)
    {
        if (filter == null || filter.tags == null || filter.tags.Length == 0)
        {
            return true;
        }
        return filter.filter_mode == TagsFilterMsg.FILTER_MODE_ANY
            || filter.filter_mode == TagsFilterMsg.FILTER_MODE_ALL;
    }

    private static bool MatchesTags(string[] entityTags, TagsFilterMsg filter)
    {
        if (filter == null || filter.tags == null || filter.tags.Length == 0)
        {
            return true;
        }
        string[] tags = entityTags ?? Array.Empty<string>();

        if (filter.filter_mode == TagsFilterMsg.FILTER_MODE_ALL)
        {
            foreach (string wanted in filter.tags)
            {
                if (Array.IndexOf(tags, wanted) < 0)
                {
                    return false;
                }
            }
            return true;
        }

        foreach (string wanted in filter.tags)
        {
            if (Array.IndexOf(tags, wanted) >= 0)
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>フィルタ bounds (ROS ワールド座標) とエンティティの AABB が重なるか。</summary>
    private static bool OverlapsBounds(GameObject entity, BoundsMsg bounds)
    {
        if (!TryComputeLocalBounds(entity, null, out Vector3 unityMin, out Vector3 unityMax))
        {
            // 形が取れないものは位置だけの点として扱う。
            unityMin = unityMax = entity.transform.position;
        }
        ToRosAxisAlignedBox(unityMin, unityMax, out Vector3Msg rosMinMsg, out Vector3Msg rosMaxMsg);
        Vector3 min = new Vector3((float)rosMinMsg.x, (float)rosMinMsg.y, (float)rosMinMsg.z);
        Vector3 max = new Vector3((float)rosMaxMsg.x, (float)rosMaxMsg.y, (float)rosMaxMsg.z);

        if (bounds.type == BoundsMsg.TYPE_BOX)
        {
            // points は upper right, lower left の順だが、取り違えた要求でも
            // 判定が壊れないよう成分ごとに min/max を取り直す。
            Vector3 a = ToVector3(bounds.points[0]);
            Vector3 b = ToVector3(bounds.points[1]);
            Vector3 filterMin = Vector3.Min(a, b);
            Vector3 filterMax = Vector3.Max(a, b);
            return min.x <= filterMax.x && max.x >= filterMin.x
                && min.y <= filterMax.y && max.y >= filterMin.y
                && min.z <= filterMax.z && max.z >= filterMin.z;
        }

        if (bounds.type == BoundsMsg.TYPE_CONVEX_HULL)
        {
            Vector3[] hull = new Vector3[bounds.points.Length];
            for (int i = 0; i < hull.Length; i++)
            {
                hull[i] = ToVector3(bounds.points[i]);
            }
            return ConvexHullOverlapsBox(hull, min, max);
        }

        // TYPE_SPHERE: 1 点目が中心、2 点目の x が半径。
        Vector3 center = ToVector3(bounds.points[0]);
        float radius = (float)bounds.points[1].x;
        Vector3 closest = new Vector3(
            Mathf.Clamp(center.x, min.x, max.x),
            Mathf.Clamp(center.y, min.y, max.y),
            Mathf.Clamp(center.z, min.z, max.z));
        return (closest - center).sqrMagnitude <= radius * radius;
    }

    // ====================================================================
    // 凸包と AABB の交差判定 (GJK)
    // ====================================================================
    //
    // Bounds.msg の TYPE_CONVEX_HULL が持っているのは**頂点だけ**で、どの頂点が
    // どの面を作るかは入っていない。分離軸判定 (SAT) には凸包の面法線が要るので、
    // 使うには 3 次元凸包を組み立てる必要がある (退化した入力の扱いも含めて面倒)。
    //
    // GJK は形状を「向きを与えると最も遠い点を返す関数 (support)」としてしか見ない。
    // 頂点集合の support は内積が最大の頂点、AABB の support は符号で角を選ぶだけなので、
    // 面の情報なしにそのまま判定できる。同一平面上の点しか無い凸包 (三角形) や
    // 一直線上の点でも、退化した凸集合としてそのまま扱える。

    // 収束しない入力は想定していないが、万一に備えて反復数を切る。
    private const int k_GjkMaxIterations = 32;

    private static Vector3 SupportOfPoints(Vector3[] points, Vector3 direction)
    {
        Vector3 best = points[0];
        float bestDot = Vector3.Dot(best, direction);
        for (int i = 1; i < points.Length; i++)
        {
            float dot = Vector3.Dot(points[i], direction);
            if (dot > bestDot)
            {
                bestDot = dot;
                best = points[i];
            }
        }
        return best;
    }

    private static Vector3 SupportOfBox(Vector3 min, Vector3 max, Vector3 direction)
    {
        return new Vector3(
            direction.x >= 0f ? max.x : min.x,
            direction.y >= 0f ? max.y : min.y,
            direction.z >= 0f ? max.z : min.z);
    }

    /// <summary>ミンコフスキー差の support。原点を含むかどうかが交差判定になる。</summary>
    private static Vector3 SupportOfDifference(Vector3[] hull, Vector3 min, Vector3 max, Vector3 direction)
    {
        return SupportOfPoints(hull, direction) - SupportOfBox(min, max, -direction);
    }

    private static bool ConvexHullOverlapsBox(Vector3[] hull, Vector3 min, Vector3 max)
    {
        Vector3 seed = (min + max) * 0.5f - hull[0];
        Vector3 direction = seed.sqrMagnitude > 1e-12f ? seed : Vector3.right;

        var simplex = new List<Vector3>(4) { SupportOfDifference(hull, min, max, direction) };
        direction = -simplex[0];

        for (int i = 0; i < k_GjkMaxIterations; i++)
        {
            if (direction.sqrMagnitude < 1e-14f)
            {
                // 探索方向が消えるのは原点がシンプレックス上にあるとき = 接触。
                return true;
            }

            Vector3 next = SupportOfDifference(hull, min, max, direction);
            if (Vector3.Dot(next, direction) < 0f)
            {
                // その向きにミンコフスキー差が原点へ届かない = 分離軸が見つかった。
                return false;
            }

            simplex.Insert(0, next);
            if (SimplexContainsOrigin(simplex, ref direction))
            {
                return true;
            }
        }

        // ここへ来ることは想定していない。取りこぼすより余計に返すほうが害が小さいので
        // 重なり扱いにする。
        Debug.LogWarning("[GetEntities] convex hull overlap test did not converge; treating as overlapping");
        return true;
    }

    /// <summary>
    /// GJK のシンプレックス更新。原点を含んでいれば true、そうでなければ
    /// 次に探索すべき向きを direction に入れて false を返す。
    /// </summary>
    /// <remarks>simplex[0] が最後に足した点。</remarks>
    private static bool SimplexContainsOrigin(List<Vector3> simplex, ref Vector3 direction)
    {
        Vector3 a = simplex[0];
        Vector3 toOrigin = -a;

        if (simplex.Count == 2)
        {
            Vector3 ab = simplex[1] - a;
            if (Vector3.Dot(ab, toOrigin) > 0f)
            {
                direction = Vector3.Cross(Vector3.Cross(ab, toOrigin), ab);
                if (direction.sqrMagnitude < 1e-14f)
                {
                    // 原点が線分上に載っている。
                    return true;
                }
            }
            else
            {
                simplex.RemoveAt(1);
                direction = toOrigin;
            }
            return false;
        }

        if (simplex.Count == 3)
        {
            Vector3 ab = simplex[1] - a;
            Vector3 ac = simplex[2] - a;
            Vector3 abc = Vector3.Cross(ab, ac);

            if (Vector3.Dot(Vector3.Cross(abc, ac), toOrigin) > 0f)
            {
                if (Vector3.Dot(ac, toOrigin) > 0f)
                {
                    simplex.RemoveAt(1);
                    direction = Vector3.Cross(Vector3.Cross(ac, toOrigin), ac);
                    return false;
                }
                simplex.RemoveAt(2);
                return SimplexContainsOrigin(simplex, ref direction);
            }
            if (Vector3.Dot(Vector3.Cross(ab, abc), toOrigin) > 0f)
            {
                simplex.RemoveAt(2);
                return SimplexContainsOrigin(simplex, ref direction);
            }

            float side = Vector3.Dot(abc, toOrigin);
            if (side > 0f)
            {
                direction = abc;
            }
            else if (side < 0f)
            {
                Vector3 swap = simplex[1];
                simplex[1] = simplex[2];
                simplex[2] = swap;
                direction = -abc;
            }
            else
            {
                // 原点が三角形の平面上。上の 2 つの判定を抜けているので内側。
                return true;
            }
            return false;
        }

        // 四面体。原点がどの面の外側にあるかを調べ、その面から遠い頂点を落として
        // 三角形の判定へ戻す。どの面の外側でもなければ原点を含んでいる。
        {
            Vector3 ab = simplex[1] - a;
            Vector3 ac = simplex[2] - a;
            Vector3 ad = simplex[3] - a;

            if (Vector3.Dot(Vector3.Cross(ab, ac), toOrigin) > 0f)
            {
                simplex.RemoveAt(3);
                return SimplexContainsOrigin(simplex, ref direction);
            }
            if (Vector3.Dot(Vector3.Cross(ac, ad), toOrigin) > 0f)
            {
                simplex.RemoveAt(1);
                return SimplexContainsOrigin(simplex, ref direction);
            }
            if (Vector3.Dot(Vector3.Cross(ad, ab), toOrigin) > 0f)
            {
                simplex.RemoveAt(2);
                return SimplexContainsOrigin(simplex, ref direction);
            }
            return true;
        }
    }

    // ====================================================================
    // 状態と形状の取り出し
    // ====================================================================

    /// <summary>
    /// エンティティの基準となる Transform。URDF ロボットならベースリンク、
    /// それ以外はエンティティ自身。
    /// </summary>
    /// <remarks>
    /// URDF ロボットはルートの GameObject ではなくベースリンクの ArticulationBody が
    /// 動くので、ルートの transform を読むとスポーン位置から動かないように見える。
    /// GroundTruthPub が targetObject にベースリンクを取るのと同じ理由。
    /// </remarks>
    private static Transform GetEntityFrame(GameObject entity)
    {
        ArticulationBody rootBody = GetEntityRootBody(entity);
        return rootBody != null ? rootBody.transform : entity.transform;
    }

    private static ArticulationBody GetEntityRootBody(GameObject entity)
    {
        foreach (GameObject link in GetChildObjectsWithComponent<UrdfLink>(entity))
        {
            ArticulationBody body = link.GetComponent<ArticulationBody>();
            if (body != null)
            {
                return body;
            }
        }
        return entity.GetComponent<ArticulationBody>();
    }

    private EntityStateMsg BuildEntityState(GameObject entity)
    {
        Transform frame = GetEntityFrame(entity);
        ArticulationBody rootBody = GetEntityRootBody(entity);

        Vector3 linear = rootBody != null ? rootBody.linearVelocity : Vector3.zero;
        Vector3 angular = rootBody != null ? rootBody.angularVelocity : Vector3.zero;

        return new EntityStateMsg
        {
            header = new HeaderMsg
            {
                // frame_id が空なら world と決まっているが、明示したほうが読み手に優しい。
                frame_id = "world",
                stamp = new TimeStamp(Clock.time)
            },
            pose = new PoseMsg
            {
                position = UnityToRosPoint(frame.position),
                orientation = UnityToRosQuaternion(frame.rotation)
            },
            twist = new TwistMsg
            {
                linear = UnityToRosVector(linear),
                angular = UnityToRosAngular(angular)
            },
            // 加速度は FixedUpdate で速度の差分から求めたもの (Unity に直接読む口が無い)。
            acceleration = BuildAcceleration(entity.name)
        };
    }

    /// <summary>差分から求めた加速度を ROS 座標系で返す。まだ 1 ステップも回っていなければゼロ。</summary>
    private AccelMsg BuildAcceleration(string entityName)
    {
        EntityMotion motion;
        if (!m_EntityMotion.TryGetValue(entityName, out motion) || !motion.HasPrevious)
        {
            return new AccelMsg();
        }
        return new AccelMsg
        {
            linear = UnityToRosVector(motion.LinearAcceleration),
            angular = UnityToRosAngular(motion.AngularAcceleration)
        };
    }

    /// <summary>
    /// エンティティの AABB を求める。frame が null ならワールド座標、
    /// そうでなければその Transform のローカル座標 (どちらも Unity 軸)。
    /// </summary>
    private static bool TryComputeLocalBounds(GameObject entity, Transform frame, out Vector3 min, out Vector3 max)
    {
        min = Vector3.positiveInfinity;
        max = Vector3.negativeInfinity;
        bool any = false;

        // メッシュのローカル AABB の 8 頂点を目的の座標系へ移してから包む。
        // Renderer.bounds (ワールド AABB) を経由すると、回転している部品のぶん
        // 二重に膨らむ。
        foreach (MeshFilter meshFilter in entity.GetComponentsInChildren<MeshFilter>(true))
        {
            Mesh mesh = meshFilter.sharedMesh;
            if (mesh == null)
            {
                continue;
            }
            Matrix4x4 toTarget = frame != null
                ? frame.worldToLocalMatrix * meshFilter.transform.localToWorldMatrix
                : meshFilter.transform.localToWorldMatrix;
            AccumulateBoxCorners(mesh.bounds, toTarget, ref min, ref max);
            any = true;
        }

        if (!any)
        {
            // メッシュが無い (プリミティブコライダーだけ、など) 場合の受け皿。
            foreach (Collider collider in entity.GetComponentsInChildren<Collider>(true))
            {
                Bounds worldBounds = collider.bounds;
                Matrix4x4 toTarget = frame != null ? frame.worldToLocalMatrix : Matrix4x4.identity;
                AccumulateBoxCorners(worldBounds, toTarget, ref min, ref max);
                any = true;
            }
        }
        return any;
    }

    private static void AccumulateBoxCorners(Bounds box, Matrix4x4 transform, ref Vector3 min, ref Vector3 max)
    {
        Vector3 c = box.center;
        Vector3 e = box.extents;
        for (int i = 0; i < 8; i++)
        {
            Vector3 corner = new Vector3(
                c.x + ((i & 1) != 0 ? e.x : -e.x),
                c.y + ((i & 2) != 0 ? e.y : -e.y),
                c.z + ((i & 4) != 0 ? e.z : -e.z));
            Vector3 mapped = transform.MultiplyPoint3x4(corner);
            min = Vector3.Min(min, mapped);
            max = Vector3.Max(max, mapped);
        }
    }

    /// <summary>
    /// Unity 軸の AABB を ROS 軸の AABB に読み替える。軸の入れ替えと符号反転だけなので
    /// 角を写して成分ごとに取り直せば正確。
    /// </summary>
    private static void ToRosAxisAlignedBox(Vector3 unityMin, Vector3 unityMax, out Vector3Msg rosMin, out Vector3Msg rosMax)
    {
        Vector3 a = new Vector3(unityMin.z, -unityMin.x, unityMin.y);
        Vector3 b = new Vector3(unityMax.z, -unityMax.x, unityMax.y);
        Vector3 lo = Vector3.Min(a, b);
        Vector3 hi = Vector3.Max(a, b);
        rosMin = new Vector3Msg(lo.x, lo.y, lo.z);
        rosMax = new Vector3Msg(hi.x, hi.y, hi.z);
    }

    // ====================================================================
    // リソース走査
    // ====================================================================

    private static void AddSearchRoot(string source, List<string> roots, List<string> problems, string origin)
    {
        if (string.IsNullOrWhiteSpace(source))
        {
            return;
        }

        string path = source;
        if (Uri.TryCreate(source, UriKind.Absolute, out Uri uri) && uri.IsFile)
        {
            path = uri.LocalPath;
        }

        if (Directory.Exists(path))
        {
            roots.Add(path);
        }
        else if (File.Exists(path))
        {
            // ディレクトリでなくファイルを直接指されることもある。
            roots.Add(path);
        }
        else
        {
            problems.Add($"unrecognized {origin} entry '{source}'");
        }
    }

    /// <summary>
    /// ディレクトリを再帰的にたどる。読めないディレクトリで全体を落とさないよう
    /// 1 階層ずつ列挙して例外を握る。
    /// </summary>
    private static IEnumerable<string> EnumerateFiles(string root, List<string> problems, ScanBudget budget)
    {
        if (budget.Exhausted)
        {
            yield break;
        }
        if (File.Exists(root))
        {
            budget.Remaining--;
            yield return root;
            yield break;
        }

        var pending = new Stack<string>();
        pending.Push(root);
        while (pending.Count > 0)
        {
            string directory = pending.Pop();

            string[] files;
            try
            {
                files = Directory.GetFiles(directory);
                foreach (string sub in Directory.GetDirectories(directory))
                {
                    pending.Push(sub);
                }
            }
            catch (Exception e)
            {
                problems.Add($"cannot read '{directory}': {e.Message}");
                continue;
            }

            foreach (string file in files)
            {
                if (budget.Remaining <= 0)
                {
                    budget.Exhausted = true;
                    yield break;
                }
                budget.Remaining--;
                yield return file;
            }
        }
    }

    private static string DescribeSpawnable(string filePath)
    {
        string extension = Path.GetExtension(filePath).ToLowerInvariant();
        if (extension != ".urdf")
        {
            return $"{extension.TrimStart('.')} mesh: {Path.GetFileName(filePath)}";
        }

        // URDF は robot 名がそのままエンティティ名の既定値になるので、それを見せる。
        try
        {
            var fileInfo = new FileInfo(filePath);
            if (fileInfo.Length <= 4 * 1024 * 1024)
            {
                Match match = Regex.Match(File.ReadAllText(filePath), "<robot[^>]*name\\s*=\\s*\"([^\"]*)\"");
                if (match.Success)
                {
                    return $"URDF robot '{match.Groups[1].Value}': {Path.GetFileName(filePath)}";
                }
            }
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[GetSpawnables] {filePath} の robot 名を読めなかった: {e.Message}");
        }
        return $"URDF: {Path.GetFileName(filePath)}";
    }

    private static string ToFileUri(string path)
    {
        return new Uri(Path.GetFullPath(path)).AbsoluteUri;
    }

    // ====================================================================
    // ROS (右手系・Z 上) と Unity (左手系・Y 上) の読み替え
    // ====================================================================
    //
    // GroundTruthPub が publish しているのと同じ対応:
    //   位置ベクトル  ros = ( uz, -ux,  uy)
    //   クォータニオン ros = (-uz,  ux, -uy, uw)
    //   角速度        ros = (-uz,  ux, -uy)   位置と符号が逆なのは左右系の反転のため

    private static PointMsg UnityToRosPoint(Vector3 v) => new PointMsg(v.z, -v.x, v.y);

    private static Vector3Msg UnityToRosVector(Vector3 v) => new Vector3Msg(v.z, -v.x, v.y);

    private static Vector3Msg UnityToRosAngular(Vector3 v) => new Vector3Msg(-v.z, v.x, -v.y);

    private static QuaternionMsg UnityToRosQuaternion(Quaternion q) => new QuaternionMsg(-q.z, q.x, -q.y, q.w);

    private static Vector3 RosToUnityPosition(PointMsg p) =>
        new Vector3((float)-p.y, (float)p.z, (float)p.x);

    private static Vector3 RosToUnityVector(Vector3Msg v) =>
        new Vector3((float)-v.y, (float)v.z, (float)v.x);

    private static Vector3 RosToUnityAngular(Vector3Msg v) =>
        new Vector3((float)v.y, (float)-v.z, (float)-v.x);

    private static Quaternion RosToUnityRotation(QuaternionMsg q) =>
        new Quaternion((float)q.y, (float)-q.z, (float)-q.x, (float)q.w);

    private static Vector3 ToVector3(Vector3Msg v) => new Vector3((float)v.x, (float)v.y, (float)v.z);

    /// <summary>姿勢として使える値か (非有限値と、正規化されていないクォータニオンを弾く)。</summary>
    private static bool IsValidPose(PoseMsg pose)
    {
        if (pose.position == null || pose.orientation == null)
        {
            return false;
        }
        if (!IsFinite(pose.position.x) || !IsFinite(pose.position.y) || !IsFinite(pose.position.z))
        {
            return false;
        }
        QuaternionMsg q = pose.orientation;
        if (!IsFinite(q.x) || !IsFinite(q.y) || !IsFinite(q.z) || !IsFinite(q.w))
        {
            return false;
        }
        double norm = Math.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        return Math.Abs(norm - 1.0) <= 1e-3;
    }

    private static bool IsFinite(double value)
    {
        return !double.IsNaN(value) && !double.IsInfinity(value);
    }
}
