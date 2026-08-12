using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 景観オブジェクトをウェイポイント経路に沿ってキネマティックに動かす
/// (動く障害物)。回避・追従シナリオのテスト用。
/// </summary>
/// <remarks>
/// - kinematic な Rigidbody を付けて MovePosition/MoveRotation で動かすので、
///   ロボット (ArticulationBody) をきちんと押し、接触イベントも発生する。
/// - 時間は FixedUpdate で自前に積算する。timeScale=0 (停止・一時停止) では
///   止まり、step_simulation とも整合するので、テストは決定論的に書ける。
/// - 各ウェイポイントに time (経路開始からの秒) を持たせるか、time を使わず
///   speed [m/s] で一定速移動にするかを選べる。loop は周回 (最後→最初へ戻る) と
///   往復 (pingpong)。ResetMotion() で開始位置へ戻る (reset_simulation が呼ぶ)。
/// </remarks>
public class WaypointMover : MonoBehaviour
{
    public struct Waypoint
    {
        public Vector3 position;   // Unity ワールド座標
        public float yawDeg;       // Unity Y 軸まわり [deg]
        public float time;         // 経路開始からの秒。速度指定のときは無視
    }

    private readonly List<Waypoint> m_Waypoints = new List<Waypoint>();
    private float[] m_CumulativeTimes; // 各ウェイポイント到達時刻
    private float m_TotalTime;
    private bool m_PingPong;
    private float m_Elapsed;
    private Rigidbody m_Rigidbody;

    public IReadOnlyList<Waypoint> Waypoints => m_Waypoints;
    public bool PingPong => m_PingPong;
    public float Speed { get; private set; }
    public bool UsesTimes { get; private set; }

    /// <summary>
    /// 経路を設定する。useTimes が真なら各ウェイポイントの time (単調増加) を、
    /// 偽なら speed から区間時間を作る。呼び出し時に最初のウェイポイントへ移動する。
    /// </summary>
    public void Configure(List<Waypoint> waypoints, float speed, bool pingPong, bool useTimes)
    {
        if (waypoints == null || waypoints.Count < 2)
        {
            Debug.LogWarning($"[WaypointMover] '{name}' needs at least 2 waypoints; disabling");
            enabled = false;
            return;
        }

        m_Waypoints.Clear();
        m_Waypoints.AddRange(waypoints);
        m_PingPong = pingPong;
        Speed = Mathf.Max(0.01f, speed);
        UsesTimes = useTimes;

        m_CumulativeTimes = new float[m_Waypoints.Count];
        if (useTimes)
        {
            float previous = 0f;
            for (int i = 0; i < m_Waypoints.Count; i++)
            {
                // 単調増加を強制する (壊れた入力でも前へ進む)
                previous = Mathf.Max(previous + (i == 0 ? 0f : 1e-3f), m_Waypoints[i].time);
                m_CumulativeTimes[i] = previous;
            }
        }
        else
        {
            m_CumulativeTimes[0] = 0f;
            for (int i = 1; i < m_Waypoints.Count; i++)
            {
                float distance = Vector3.Distance(
                    m_Waypoints[i - 1].position, m_Waypoints[i].position);
                m_CumulativeTimes[i] = m_CumulativeTimes[i - 1] + distance / Speed;
            }
        }
        m_TotalTime = Mathf.Max(m_CumulativeTimes[m_Waypoints.Count - 1], 1e-3f);

        // 物理と接触が正しく効くよう kinematic Rigidbody で動かす
        m_Rigidbody = GetComponent<Rigidbody>();
        if (m_Rigidbody == null)
        {
            m_Rigidbody = gameObject.AddComponent<Rigidbody>();
        }
        m_Rigidbody.isKinematic = true;

        ResetMotion();
    }

    /// <summary>経路の先頭へ戻す (reset_simulation から呼ばれる)。</summary>
    public void ResetMotion()
    {
        m_Elapsed = 0f;
        if (m_Waypoints.Count > 0)
        {
            transform.SetPositionAndRotation(
                m_Waypoints[0].position, Quaternion.Euler(0f, m_Waypoints[0].yawDeg, 0f));
            if (m_Rigidbody != null)
            {
                m_Rigidbody.position = m_Waypoints[0].position;
                m_Rigidbody.rotation = Quaternion.Euler(0f, m_Waypoints[0].yawDeg, 0f);
            }
        }
    }

    void FixedUpdate()
    {
        if (m_Waypoints.Count < 2 || m_Rigidbody == null)
        {
            return;
        }
        m_Elapsed += Time.fixedDeltaTime;

        // 周回または往復の経路時刻へ折り畳む
        float t;
        if (m_PingPong)
        {
            float cycle = Mathf.Repeat(m_Elapsed, m_TotalTime * 2f);
            t = cycle <= m_TotalTime ? cycle : m_TotalTime * 2f - cycle;
        }
        else
        {
            t = Mathf.Repeat(m_Elapsed, m_TotalTime);
        }

        // t を挟む区間を探して線形補間 (ウェイポイント数は少ない前提の線形走査)
        int segment = 1;
        while (segment < m_Waypoints.Count - 1 && m_CumulativeTimes[segment] < t)
        {
            segment++;
        }
        float t0 = m_CumulativeTimes[segment - 1];
        float t1 = m_CumulativeTimes[segment];
        float u = Mathf.InverseLerp(t0, t1, t);

        Waypoint a = m_Waypoints[segment - 1];
        Waypoint b = m_Waypoints[segment];
        Vector3 position = Vector3.Lerp(a.position, b.position, u);
        Quaternion rotation = Quaternion.Slerp(
            Quaternion.Euler(0f, a.yawDeg, 0f), Quaternion.Euler(0f, b.yawDeg, 0f), u);

        m_Rigidbody.MovePosition(position);
        m_Rigidbody.MoveRotation(rotation);
    }
}
