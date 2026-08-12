using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.SimulationExtraInterfaces;

/// <summary>
/// simulation_extra_interfaces の衝突記録サービス (get_contact_events)。
/// 各リンクの ContactReporter から届くイベントを (entity, link, 相手) 単位で
/// 集計する。テストの「走行中にどこかへぶつからなかったか」を 1 クエリで
/// 判定できるようにするのが目的。
/// </summary>
/// <remarks>
/// 車輪が床に触れている類の「期待どおりの接触」も記録される (相手名は組み込み
/// 床なら "Plane")。何を無視するかは呼び出し側が相手名で選ぶ。記録は
/// reset_simulation (SCOPE_STATE) と、サービスの clear フラグで消える。
/// </remarks>
public partial class SimulationControl
{
    [SerializeField]
    private string m_GetContactEventsServiceName = "get_contact_events";

    private class ContactAggregate
    {
        public uint count;
        public double firstTime;
        public double lastTime;
        public double maxImpulse;
    }

    private readonly Dictionary<(string entity, string link, string other), ContactAggregate>
        m_ContactRecords = new Dictionary<(string, string, string), ContactAggregate>();

    private void ImplementContactServices()
    {
        ROSConnection.GetOrCreateInstance()
            .ImplementService<GetContactEventsRequest, GetContactEventsResponse>(
                m_GetContactEventsServiceName, GetContactEvents);
    }

    /// <summary>スポーン時に全リンクへ ContactReporter を取り付ける。</summary>
    private void AttachContactReporters(GameObject robotObject)
    {
        foreach (GameObject abObject in FindArticulationBodyObjectsInChildren(robotObject))
        {
            ContactReporter reporter = abObject.AddComponent<ContactReporter>();
            reporter.entityName = robotObject.name;
            // リンク名は UrdfLink/UrdfJoint 由来の GameObject 名と一致する
            reporter.linkName = abObject.name;
            reporter.control = this;
        }
    }

    /// <summary>ContactReporter からの転送先。集計だけ行う (毎接触ステップ呼ばれる)。</summary>
    public void ReportContact(string entity, string link, Collision collision, bool isEnter)
    {
        string other = ResolveContactOtherName(collision);
        // 自分自身のリンク同士は報告しない (URDF の隣接リンクは元々衝突しないが、
        // 自己干渉が有効な構成では起き得る。テスト用途では雑音なので落とす)
        if (other == entity)
        {
            return;
        }

        var key = (entity, link, other);
        double now = Clock.Now;
        if (!m_ContactRecords.TryGetValue(key, out ContactAggregate agg))
        {
            agg = new ContactAggregate { firstTime = now };
            m_ContactRecords[key] = agg;
        }
        if (isEnter)
        {
            agg.count++;
        }
        agg.lastTime = now;
        float impulse = collision.impulse.magnitude;
        if (impulse > agg.maxImpulse)
        {
            agg.maxImpulse = impulse;
        }
    }

    /// <summary>
    /// 衝突相手の表示名: スポーン済みエンティティならその名前、
    /// それ以外 (景観・組み込みシーン) はルート GameObject 名。
    /// </summary>
    private string ResolveContactOtherName(Collision collision)
    {
        Transform other = collision.transform;
        foreach (GameObject entity in m_EntityList)
        {
            if (entity != null && other.IsChildOf(entity.transform))
            {
                return entity.name;
            }
        }
        return other.root.name;
    }

    /// <summary>get_contact_events サービス。</summary>
    private GetContactEventsResponse GetContactEvents(GetContactEventsRequest request)
    {
        var response = new GetContactEventsResponse();
        bool all = string.IsNullOrEmpty(request.entity);

        var matched = m_ContactRecords
            .Where(kv => all || kv.Key.entity == request.entity)
            .ToList();

        response.contacts = matched.Select(kv => new ContactRecordMsg
        {
            entity = kv.Key.entity,
            link = kv.Key.link,
            other = kv.Key.other,
            count = kv.Value.count,
            first_time = kv.Value.firstTime,
            last_time = kv.Value.lastTime,
            max_impulse = kv.Value.maxImpulse,
        }).ToArray();

        if (request.clear)
        {
            foreach (var kv in matched)
            {
                m_ContactRecords.Remove(kv.Key);
            }
        }

        response.result = GetContactEventsResponse.RESULT_OK;
        return response;
    }

    /// <summary>リセット時に全記録を消す (記録は「状態」の一部)。</summary>
    private void ClearContactRecords()
    {
        m_ContactRecords.Clear();
    }
}
