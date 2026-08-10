using System;
using System.IO;
using UnityEngine;

using RosMessageTypes.SimulationInterfaces;

/// <summary>
/// GUI (サイドバーの「Spawn Robot (URDF)」ボタン) からロボットをスポーンする入口。
/// </summary>
/// <remarks>
/// /spawn_entity サービスと同じ SpawnEntityCore を通す。これにより GUI 生成の
/// ロボットもサービス生成と完全に同格のエンティティになる:
///   - m_EntityList に載るので get_entities / get_entities_states に出る
///   - /joint_states 等のトピックが生え、デスポーン時に解除される
///   - delete_entity で消せる
///   - reset_simulation の SCOPE_SPAWNED で一括デスポーンされる
/// GUI 専用の別経路を作るとこの対称性が壊れるので、ここではサービス実装へ
/// 委譲するだけで独自のスポーン処理を持たない。サービス側の挙動は適合試験で
/// 固定されているため、この委譲は読み取り専用の呼び出しに徹する。
/// </remarks>
public partial class SimulationControl
{
    /// <summary>
    /// GUI からのスポーンが今できるか。サービスと同じ条件 (ワールドがロード済み)。
    /// SpawnEntityCore も同じ検査をするので、これはボタンの活性表示用。
    /// </summary>
    public bool CanSpawnFromGui => IsWorldLoaded;

    /// <summary>
    /// URDF ファイルからロボットを 1 体スポーンする。エンティティ名はファイル名
    /// (拡張子なし) を要求し、衝突したらサービスと同じ規則で _1, _2 と連番になる。
    /// 名前空間は付けない。初期姿勢は地面 (z=0) の空いた場所・無回転。
    /// </summary>
    /// <returns>成功したら true。失敗の理由は errorMessage に入る。</returns>
    public bool TrySpawnRobotFromUrdf(string urdfPath, out string entityName, out string errorMessage)
    {
        entityName = null;
        errorMessage = null;

        ResourceMsg resource;
        try
        {
            // SpawnEntityCore は file:// URI を受け取る。空白などを含むパスを
            // 手組みで壊さないよう Uri にエンコードさせる。
            resource = new ResourceMsg(new Uri(Path.GetFullPath(urdfPath)).AbsoluteUri, "");
        }
        catch (Exception e)
        {
            errorMessage = "Invalid file path: " + e.Message;
            return false;
        }

        // 初期姿勢: 地面の上の空いた場所。orientation は QuaternionMsg の既定値
        // (単位クォータニオン) をそのまま使う。
        Vector3 spot = FindFreeGroundSpotUnity();
        var initialPose = new RosMessageTypes.Geometry.PoseStampedMsg();
        // Unity (左手系, Y 上) → ROS (右手系, Z 上)。SpawnEntityFromFile の
        // 逆変換: Unity(x,y,z) = ROS(-y, z, x)。
        initialPose.pose.position.x = spot.z;
        initialPose.pose.position.y = -spot.x;
        initialPose.pose.position.z = spot.y;

        SpawnResultMsg result = SpawnEntityCore(
            Path.GetFileNameWithoutExtension(urdfPath), // 要求名はファイル名由来
            true,                                       // 衝突時はサービスの連番規則に任せる
            resource,
            "",                                        // 名前空間なし
            initialPose);

        if (result.result.result != ResultMsg.RESULT_OK)
        {
            errorMessage = string.IsNullOrEmpty(result.result.error_message)
                ? "Spawn failed (result code " + result.result.result + ")"
                : result.result.error_message;
            return false;
        }

        entityName = result.entity_name;
        return true;
    }

    /// <summary>
    /// 地面 (Unity y=0) の空いた場所を Unity 座標で返す。原点から ROS x 方向
    /// (Unity +z) へ 1 m 刻みでずらし、既存エンティティの水平距離 1 m 以内に
    /// 入らない最初の場所を選ぶ。景観 (GUI のプリミティブ等) までは見ない。
    /// </summary>
    private Vector3 FindFreeGroundSpotUnity()
    {
        for (int i = 0; i < 32; i++)
        {
            var candidate = new Vector3(0f, 0f, i);
            bool occupied = false;
            foreach (GameObject entity in m_EntityList)
            {
                if (entity == null)
                {
                    continue;
                }
                Vector3 p = entity.transform.position;
                float dx = p.x - candidate.x;
                float dz = p.z - candidate.z;
                if (dx * dx + dz * dz < 1f)
                {
                    occupied = true;
                    break;
                }
            }
            if (!occupied)
            {
                return candidate;
            }
        }
        // 32 m 先まで全部埋まっているなら諦めて原点 (重なりはユーザーが解決する)。
        return Vector3.zero;
    }
}
