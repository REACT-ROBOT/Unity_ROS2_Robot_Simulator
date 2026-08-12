using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.SimulationExtraInterfaces;

/// <summary>
/// simulation_extra_interfaces の外乱注入サービス (apply_link_wrench)。
/// テストシナリオから「突風・衝撃・押され」を再現するための口で、
/// リンク重心へワールド座標系 (ROS 慣習) のレンチを一定シミュ時間印加する。
/// </summary>
/// <remarks>
/// 持続時間は物理ステップ数で数える。FixedUpdate は timeScale=0 (停止・一時停止)
/// では回らないので、PLAYING でも PAUSED+step_simulation でも同じステップ数だけ
/// 効く = テストが決定論的になる。リセット時は保持中のレンチを消す。
/// </remarks>
public partial class SimulationControl
{
    [SerializeField]
    private string m_ApplyLinkWrenchServiceName = "apply_link_wrench";

    private class ActiveWrench
    {
        public ArticulationBody body;
        public Vector3 force;   // Unity 座標系・ワールドフレーム [N]
        public Vector3 torque;  // Unity 座標系 [N·m]
        public int remainingSteps;
    }

    private readonly List<ActiveWrench> m_ActiveWrenches = new List<ActiveWrench>();

    private void ImplementDisturbanceServices()
    {
        ROSConnection.GetOrCreateInstance()
            .ImplementService<ApplyLinkWrenchRequest, ApplyLinkWrenchResponse>(
                m_ApplyLinkWrenchServiceName, ApplyLinkWrench);
    }

    /// <summary>apply_link_wrench サービス。</summary>
    private ApplyLinkWrenchResponse ApplyLinkWrench(ApplyLinkWrenchRequest request)
    {
        var response = new ApplyLinkWrenchResponse();

        if (!TryFindEntity(request.entity, out GameObject entity))
        {
            response.result = ApplyLinkWrenchResponse.RESULT_NOT_FOUND;
            response.error_message = $"Entity '{request.entity}' not found";
            return response;
        }

        ArticulationBody body;
        if (string.IsNullOrEmpty(request.link))
        {
            body = GetEntityRootBody(entity);
        }
        else
        {
            GameObject linkObject = FindInChildrenByName(entity.transform, request.link);
            body = linkObject != null
                ? (linkObject.GetComponent<ArticulationBody>()
                    ?? linkObject.GetComponentInParent<ArticulationBody>())
                : null;
        }
        if (body == null)
        {
            response.result = ApplyLinkWrenchResponse.RESULT_NOT_FOUND;
            response.error_message =
                $"Link '{request.link}' with an ArticulationBody not found on '{request.entity}'";
            return response;
        }

        Vector3 force = RosToUnityVector(request.wrench.force);
        Vector3 torque = RosToUnityAngular(request.wrench.torque);
        if (!IsFinite(force) || !IsFinite(torque))
        {
            response.result = ApplyLinkWrenchResponse.RESULT_OPERATION_FAILED;
            response.error_message = "wrench contains non-finite values";
            return response;
        }

        // duration はシミュ秒 → 物理ステップ数。0 はちょうど 1 ステップ。
        int steps = request.duration <= 0.0
            ? 1
            : Mathf.Max(1, Mathf.CeilToInt((float)request.duration / Time.fixedDeltaTime));

        m_ActiveWrenches.Add(new ActiveWrench
        {
            body = body,
            force = force,
            torque = torque,
            remainingSteps = steps,
        });

        response.result = ApplyLinkWrenchResponse.RESULT_OK;
        return response;
    }

    /// <summary>FixedUpdate (SimulationEntityServices.cs) から毎ステップ呼ばれる。</summary>
    private void ApplyActiveWrenches()
    {
        if (m_ActiveWrenches.Count == 0)
        {
            return;
        }
        for (int i = m_ActiveWrenches.Count - 1; i >= 0; i--)
        {
            ActiveWrench wrench = m_ActiveWrenches[i];
            // 対象が消えた (デスポーン等) ら黙って破棄する
            if (wrench.body == null || wrench.remainingSteps <= 0)
            {
                m_ActiveWrenches.RemoveAt(i);
                continue;
            }
            if (wrench.force.sqrMagnitude > 0f)
            {
                wrench.body.AddForce(wrench.force); // 重心に作用
            }
            if (wrench.torque.sqrMagnitude > 0f)
            {
                wrench.body.AddTorque(wrench.torque);
            }
            wrench.remainingSteps--;
            if (wrench.remainingSteps <= 0)
            {
                m_ActiveWrenches.RemoveAt(i);
            }
        }
    }

    /// <summary>
    /// root 配下 (null なら全部) のリンクへの保持レンチを破棄する。リセット時に
    /// 呼ばないと、初期状態へ戻した直後に残りの外乱が再適用される。
    /// </summary>
    private void ClearActiveWrenches(GameObject root = null)
    {
        m_ActiveWrenches.RemoveAll(w => w.body == null
            || root == null
            || w.body.transform.IsChildOf(root.transform));
    }

    private static bool IsFinite(Vector3 v)
    {
        return !(float.IsNaN(v.x) || float.IsInfinity(v.x)
            || float.IsNaN(v.y) || float.IsInfinity(v.y)
            || float.IsNaN(v.z) || float.IsInfinity(v.z));
    }
}
