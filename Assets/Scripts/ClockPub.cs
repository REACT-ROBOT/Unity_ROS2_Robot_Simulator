using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Rosgraph;

using Unity.Robotics.Core;

/// <summary>
/// シミュレーション時刻を /clock (rosgraph_msgs/Clock) へ publish する。
/// use_sim_time で動く ROS 2 ノードはこのトピックを時計にする。
/// </summary>
/// <remarks>
/// /clock はロボット単位ではなくシミュレータ全体で 1 本のグローバルトピック。
/// 起動時に SimulationControl が 1 つだけ取り付け、名前空間も付けず、
/// デスポーンや reset_simulation でも登録を解除しない。時刻は publish のたびに
/// Clock.Now を読むので、SCOPE_TIME でのリセットは次の publish から自動で反映される。
/// </remarks>
public class ClockPub : MonoBehaviour
{
    public string topicName = "/clock";

    [Header("publish レート [Hz] (実時間基準)")]
    public float publishRate = 100.0f;

    private ROSConnection ros;
    private float _elapsed;

    // Pre-allocated message to avoid GC allocations
    private ClockMsg _clockMsg;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        // 二重登録を避ける (理由は JointStatePub.Start のコメントを参照)。
        RosTopicState topicState = ros.GetTopic(topicName);
        if (topicState == null || !topicState.IsPublisher)
        {
            ros.RegisterPublisher<ClockMsg>(topicName, 15);
        }

        _clockMsg = new ClockMsg();
    }

    // FixedUpdate は Time.timeScale = 0 (停止・一時停止) の間は回らないので、
    // Update から実時間 (unscaledDeltaTime) の周期で publish する。Gazebo と同じく
    // 一時停止中も凍結した値を流し続けることで、use_sim_time なノードが
    // 時計を失って止まるのを防ぐ。
    void Update()
    {
        if (publishRate <= 0.0f) return;
        float interval = 1.0f / publishRate;

        _elapsed += Time.unscaledDeltaTime;
        if (_elapsed < interval) return;
        _elapsed -= interval;
        // フレーム落ちで大きく遅れても次のフレームで連射しない。周期は 1 個分だけ繰り越す。
        if (_elapsed > interval) _elapsed = interval;

        // Clock.Now は timeScale が 0 の間は進まないため、一時停止中は
        // 最後の値のまま publish され続ける。
        var timestamp = new TimeStamp(Clock.Now);
        _clockMsg.clock.sec = timestamp.Seconds;
        _clockMsg.clock.nanosec = timestamp.NanoSeconds;

        ros.Publish(topicName, _clockMsg);
    }
}
