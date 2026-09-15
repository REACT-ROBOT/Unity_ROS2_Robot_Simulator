using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.Core;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.SimulationExtraInterfaces;
using RosMessageTypes.SimulationInterfaces;

/// <summary>
/// 学習ループ向けの simulation_extra_interfaces サービス (step_and_observe)。
/// </summary>
/// <remarks>
/// 強化学習の 1 制御周期は「指令を出す → N ステップ進める → 関節状態を読む」で、
/// これを joint_command トピック + step_simulation + joint_states トピックで組むと
/// 往復が 3 本になり、しかも「どの状態がどのステップの結果か」がトピックの
/// タイミング次第になる。このサービスは同じ 3 つを 1 往復にまとめ、応答に
/// ステップ直後の関節状態を載せる。
///
/// 既存の挙動には触らない: 指令は JointStateSub の Callback と同じ経路、
/// ステップは step_simulation と同じ RunSteps、観測は JointStatePub が持つ
/// 関節配列をその場で読む (JointStatePub 自身の 30 Hz 配信はそのまま続く)。
/// </remarks>
public partial class SimulationControl
{
    [SerializeField]
    private string m_StepAndObserveServiceName = "step_and_observe";

    /// <summary>
    /// エンティティごとの前回観測 (位置と sim 時刻)。速度は位置差分で作る。
    /// reset_simulation (SCOPE_STATE) で関節が飛ぶので、そのときは消す。
    /// </summary>
    private class StepObserveBaseline
    {
        public double[] positions;
        public double simTime;
    }

    private readonly Dictionary<string, StepObserveBaseline> m_StepObserveBaselines =
        new Dictionary<string, StepObserveBaseline>();

    private void ImplementLearningServices()
    {
        ROSConnection.GetOrCreateInstance()
            .ImplementService<StepAndObserveRequest, StepAndObserveResponse>(
                m_StepAndObserveServiceName, StepAndObserve);
        // 直結 TCP サーバ (SimulationLearningServer.cs)。設定が無ければ何もしない。
        StartLearningServerIfConfigured();
    }

    /// <summary>step_and_observe サービス。</summary>
    private async Task<StepAndObserveResponse> StepAndObserve(StepAndObserveRequest request)
    {
        var response = new StepAndObserveResponse { joint_states = new JointStateMsg() };

        if (!IsWorldLoaded)
        {
            response.result = StepAndObserveResponse.RESULT_INCORRECT_STATE;
            response.error_message = "No world is loaded";
            return response;
        }
        if (m_SimulationState != SimulationStateMsg.STATE_PAUSED)
        {
            response.result = StepAndObserveResponse.RESULT_OPERATION_FAILED;
            response.error_message =
                $"Simulation must be paused to step; it is in state {m_SimulationState}";
            return response;
        }
        if (m_Stepping)
        {
            response.result = StepAndObserveResponse.RESULT_OPERATION_FAILED;
            response.error_message = "Another step is still running";
            return response;
        }
        if (request.steps > (ulong)k_MaxStepsPerCall)
        {
            response.result = StepAndObserveResponse.RESULT_OPERATION_FAILED;
            response.error_message =
                $"steps={request.steps} exceeds the per-call limit of {k_MaxStepsPerCall}";
            return response;
        }
        if (!TryFindEntity(request.entity, out GameObject entity))
        {
            response.result = StepAndObserveResponse.RESULT_NOT_FOUND;
            response.error_message = $"No entity named '{request.entity}'";
            return response;
        }
        JointStatePub pub = entity.GetComponent<JointStatePub>();
        if (pub == null || pub.articulationBodies == null || pub.jointName == null)
        {
            response.result = StepAndObserveResponse.RESULT_NOT_FOUND;
            response.error_message =
                $"Entity '{request.entity}' has no joint state publisher (no <ros2_control> in its URDF)";
            return response;
        }

        if (request.command != null && request.command.name != null && request.command.name.Length > 0)
        {
            JointStateSub sub = entity.GetComponent<JointStateSub>();
            if (sub == null)
            {
                response.result = StepAndObserveResponse.RESULT_OPERATION_FAILED;
                response.error_message = $"Entity '{request.entity}' has no joint command subscriber";
                return response;
            }
            sub.ApplyCommand(request.command);
        }

        if (request.steps > 0)
        {
            bool completed = await RunSteps(request.steps, null);
            if (!completed)
            {
                response.result = StepAndObserveResponse.RESULT_OPERATION_FAILED;
                response.error_message = "Stepping was interrupted by another state change";
                return response;
            }
        }

        // エンティティが消えていたら (ステップ中の despawn) ここで気付く。
        if (entity == null || pub == null)
        {
            response.result = StepAndObserveResponse.RESULT_NOT_FOUND;
            response.error_message = $"Entity '{request.entity}' disappeared while stepping";
            return response;
        }

        response.joint_states = BuildObservation(request.entity, pub, request.steps);
        response.sim_time = Clock.Now;
        response.result = StepAndObserveResponse.RESULT_OK;
        return response;
    }

    private JointStateMsg BuildObservation(string entityName, JointStatePub pub, ulong steps)
    {
        int n = Mathf.Min(pub.articulationBodies.Length, pub.jointName.Length);
        var msg = new JointStateMsg
        {
            header = new HeaderMsg { frame_id = pub.frameId, stamp = new TimeMsg() },
            name = new string[n],
            position = new double[n],
            velocity = new double[n],
            effort = new double[n],
        };
        double now = Clock.Now;
        var stamp = new TimeStamp(now);
        msg.header.stamp.sec = stamp.Seconds;
        msg.header.stamp.nanosec = stamp.NanoSeconds;

        m_StepObserveBaselines.TryGetValue(entityName, out StepObserveBaseline baseline);
        bool haveBaseline = steps > 0 && baseline != null && baseline.positions.Length == n;
        double dt = steps * (double)Time.fixedDeltaTime;

        var positions = new double[n];
        for (int i = 0; i < n; i++)
        {
            ArticulationBody ab = pub.articulationBodies[i];
            msg.name[i] = pub.jointName[i];
            if (ab == null || ab.dofCount == 0)
            {
                continue;
            }
            double pos = ab.jointPosition[0];
            positions[i] = pos;
            msg.position[i] = pos;
            // JointStatePub と同じ流儀: 速度は位置差分 (PhysX の jointVelocity は
            // 静止中も定常残差を返す)。ここでは差分の時間幅が「進めたステップ数」で
            // 正確に分かる。
            msg.velocity[i] = haveBaseline ? (pos - baseline.positions[i]) / dt : 0.0;
            msg.effort[i] = ab.driveForce[0] + ab.jointForce[0];
        }
        m_StepObserveBaselines[entityName] = new StepObserveBaseline { positions = positions, simTime = now };
        return msg;
    }

    /// <summary>関節が不連続に飛んだあと (リセット) に、偽の速度を返さないための無効化。</summary>
    private void ForgetStepObserveBaseline(GameObject root)
    {
        if (root != null)
        {
            m_StepObserveBaselines.Remove(root.name);
        }
    }
}
