using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;
using Unity.Robotics.Core;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Sensor;
using RosMessageTypes.SimulationInterfaces;
using RosMessageTypes.Std;

/// <summary>
/// 学習ループ向けの直結 TCP サーバ。ROS 2 を通さずに
/// 「複数エンティティへの関節指令 → N ステップ → 全エンティティの関節状態」を
/// 1 往復で行う。
/// </summary>
/// <remarks>
/// step_and_observe サービス (SimulationLearningServices.cs) と同じ意味の操作を、
/// ROS-TCP-Endpoint と DDS を経由せずに提供する。往復の固定費が数 ms〜10 ms から
/// 1 ms 未満になり、1 往復で何体でも束ねられる (ステップ中は 1 フレームに
/// 1 物理ステップなので、エンティティ数を増やしてもフレーム数は増えない)。
///
/// 有効化: settings.learning_port (simulation_resources.json) か環境変数
/// SIM_LEARNING_PORT。既定は無効で、既存の挙動には何も影響しない。
/// 指令の適用は JointStateSub.ApplyCommand、ステップは RunSteps、観測は
/// BuildObservation と、ROS 経路と同じ関数を使う。
///
/// プロトコル (リトルエンディアン、1 フレーム = uint32 長 + 本文):
///   要求: uint8 op (1=INFO, 2=RESET, 3=STEP, 4=PING)
///         uint16 n, n × 文字列 (uint16 長 + UTF-8) = エンティティ名
///         STEP のみ: uint32 steps, n × { uint16 m, m × { 文字列 関節名, float32 pos, vel, eff } }
///                    (pos/vel/eff は NaN で「指定しない」)
///   応答: uint8 status (0=OK), status != 0 なら 文字列 error
///         OK なら uint16 n, n × { uint16 k, k × { 文字列 関節名, float64 pos, vel, eff } }, float64 sim_time
/// </remarks>
public partial class SimulationControl
{
    public const string LearningPortEnvVar = "SIM_LEARNING_PORT";

    private const byte k_OpInfo = 1;
    private const byte k_OpReset = 2;
    private const byte k_OpStep = 3;
    private const byte k_OpPing = 4;
    private const int k_MaxFrameBytes = 64 * 1024 * 1024;

    private class LearningRequest
    {
        public byte[] payload;
        public byte[] response;
        public readonly ManualResetEventSlim done = new ManualResetEventSlim(false);
    }

    private TcpListener m_LearningListener;
    private Thread m_LearningThread;
    private volatile bool m_LearningStop;
    private readonly ConcurrentQueue<LearningRequest> m_LearningQueue = new ConcurrentQueue<LearningRequest>();

    private void StartLearningServerIfConfigured()
    {
        int port = 0;
        string fromEnv = Environment.GetEnvironmentVariable(LearningPortEnvVar);
        if (!string.IsNullOrEmpty(fromEnv))
        {
            int.TryParse(fromEnv, out port);
        }
        else if (SimulationResources.Settings != null)
        {
            port = SimulationResources.Settings.learning_port;
        }
        if (port <= 0)
        {
            return;
        }

        try
        {
            m_LearningListener = new TcpListener(IPAddress.Any, port);
            m_LearningListener.Start();
        }
        catch (Exception e)
        {
            Debug.LogError($"[LearningServer] could not listen on port {port}: {e.Message}");
            m_LearningListener = null;
            return;
        }
        m_LearningStop = false;
        m_LearningThread = new Thread(LearningAcceptLoop) { IsBackground = true, Name = "LearningServer" };
        m_LearningThread.Start();
        StartCoroutine(LearningMainLoop());
        Debug.Log($"[LearningServer] listening on port {port}");
    }

    private void OnApplicationQuit()
    {
        StopLearningServer();
    }

    private void StopLearningServer()
    {
        m_LearningStop = true;
        try { m_LearningListener?.Stop(); } catch (Exception) { }
        m_LearningListener = null;
        // 待っている要求は空応答で解放する (スレッドが送信で失敗しても閉じるだけ)。
        while (m_LearningQueue.TryDequeue(out LearningRequest req))
        {
            req.response = EncodeError("simulator shutting down");
            req.done.Set();
        }
    }

    // ------------------------------------------------------------- 通信スレッド
    private void LearningAcceptLoop()
    {
        while (!m_LearningStop)
        {
            TcpClient client;
            try
            {
                client = m_LearningListener.AcceptTcpClient();
            }
            catch (Exception)
            {
                break; // Stop() で抜ける
            }
            client.NoDelay = true;
            using (client)
            using (NetworkStream stream = client.GetStream())
            {
                try
                {
                    ServeLearningClient(stream);
                }
                catch (IOException) { }
                catch (ObjectDisposedException) { }
                catch (Exception e)
                {
                    Debug.LogWarning($"[LearningServer] client error: {e.Message}");
                }
            }
        }
    }

    private void ServeLearningClient(NetworkStream stream)
    {
        var lenBuf = new byte[4];
        while (!m_LearningStop)
        {
            ReadExactly(stream, lenBuf, 4);
            int len = BitConverter.ToInt32(lenBuf, 0);
            if (len <= 0 || len > k_MaxFrameBytes)
            {
                throw new IOException($"bad frame length {len}");
            }
            var req = new LearningRequest { payload = new byte[len] };
            ReadExactly(stream, req.payload, len);

            if (req.payload[0] == k_OpPing)
            {
                req.response = new byte[] { 0 };
            }
            else
            {
                m_LearningQueue.Enqueue(req);
                req.done.Wait();
            }
            byte[] outLen = BitConverter.GetBytes(req.response.Length);
            stream.Write(outLen, 0, 4);
            stream.Write(req.response, 0, req.response.Length);
            stream.Flush();
        }
    }

    private static void ReadExactly(NetworkStream stream, byte[] buf, int len)
    {
        int read = 0;
        while (read < len)
        {
            int n = stream.Read(buf, read, len - read);
            if (n <= 0)
            {
                throw new IOException("connection closed");
            }
            read += n;
        }
    }

    // --------------------------------------------------------------- メインスレッド
    private IEnumerator LearningMainLoop()
    {
        while (!m_LearningStop)
        {
            if (m_LearningQueue.TryDequeue(out LearningRequest req))
            {
                yield return HandleLearningRequest(req);
            }
            else
            {
                yield return null;
            }
        }
    }

    private IEnumerator HandleLearningRequest(LearningRequest req)
    {
        byte[] response = null;
        string error = null;
        List<string> names = null;
        List<JointStateMsg> commands = null;
        ulong steps = 0;
        byte op = req.payload[0];
        try
        {
            using (var r = new BinaryReader(new MemoryStream(req.payload)))
            {
                r.ReadByte();
                int n = r.ReadUInt16();
                names = new List<string>(n);
                for (int i = 0; i < n; i++) names.Add(ReadString(r));
                if (op == k_OpStep)
                {
                    steps = r.ReadUInt32();
                    commands = new List<JointStateMsg>(n);
                    for (int i = 0; i < n; i++)
                    {
                        int m = r.ReadUInt16();
                        var msg = new JointStateMsg
                        {
                            header = new HeaderMsg { stamp = new TimeMsg() },
                            name = new string[m], position = new double[m],
                            velocity = new double[m], effort = new double[m]
                        };
                        // NaN は「指定しない」。JointStateSub は配列長で有無を見るので、
                        // 指定しない成分は空配列にしたいが、関節ごとに混在させられない。
                        // v0 では position/velocity/effort の各配列を「どれか 1 つでも
                        // 有効なら全体を送る」とし、無効成分は現在値を保つよう NaN を
                        // 0 ではなく「送らない」扱いにするため、成分ごとに全 NaN なら配列を空にする。
                        bool anyPos = false, anyVel = false, anyEff = false;
                        for (int j = 0; j < m; j++)
                        {
                            msg.name[j] = ReadString(r);
                            float pos = r.ReadSingle(), vel = r.ReadSingle(), eff = r.ReadSingle();
                            msg.position[j] = pos; msg.velocity[j] = vel; msg.effort[j] = eff;
                            anyPos |= !float.IsNaN(pos); anyVel |= !float.IsNaN(vel); anyEff |= !float.IsNaN(eff);
                        }
                        if (!anyPos) msg.position = new double[0];
                        if (!anyVel) msg.velocity = new double[0];
                        if (!anyEff) msg.effort = new double[0];
                        commands.Add(msg);
                    }
                }
            }
        }
        catch (Exception e)
        {
            error = $"malformed request: {e.Message}";
        }

        if (error == null && op != k_OpInfo && op != k_OpReset && op != k_OpStep)
        {
            error = $"unknown op {op}";
        }
        if (error == null && !IsWorldLoaded)
        {
            error = "No world is loaded";
        }
        var entities = new List<GameObject>();
        var pubs = new List<JointStatePub>();
        if (error == null)
        {
            foreach (string name in names)
            {
                if (!TryFindEntity(name, out GameObject entity))
                {
                    error = $"No entity named '{name}'";
                    break;
                }
                JointStatePub pub = entity.GetComponent<JointStatePub>();
                if (pub == null || pub.articulationBodies == null || pub.jointName == null)
                {
                    error = $"Entity '{name}' has no joint state publisher (no <ros2_control> in its URDF)";
                    break;
                }
                entities.Add(entity);
                pubs.Add(pub);
            }
        }

        if (error == null && op == k_OpReset)
        {
            foreach (GameObject entity in entities)
            {
                ResetArticulationState(entity);
            }
        }

        if (error == null && op == k_OpStep)
        {
            for (int i = 0; i < entities.Count; i++)
            {
                JointStateMsg cmd = commands[i];
                if (cmd.name.Length == 0) continue;
                JointStateSub sub = entities[i].GetComponent<JointStateSub>();
                if (sub == null)
                {
                    error = $"Entity '{names[i]}' has no joint command subscriber";
                    break;
                }
                sub.ApplyCommand(cmd);
            }
            if (error == null && steps > 0)
            {
                if (steps > (ulong)k_MaxStepsPerCall)
                {
                    error = $"steps={steps} exceeds the per-call limit of {k_MaxStepsPerCall}";
                }
                else
                {
                    Task<bool> run = RunSteps(steps, null);
                    while (!run.IsCompleted)
                    {
                        yield return null;
                    }
                    if (!run.Result)
                    {
                        error = "Stepping was interrupted by another state change";
                    }
                }
            }
        }

        if (error != null)
        {
            response = EncodeError(error);
        }
        else
        {
            using (var ms = new MemoryStream())
            using (var w = new BinaryWriter(ms))
            {
                w.Write((byte)0);
                w.Write((ushort)entities.Count);
                for (int i = 0; i < entities.Count; i++)
                {
                    // 消えたエンティティ (ステップ中の despawn) は関節 0 として返す。
                    if (entities[i] == null || pubs[i] == null)
                    {
                        w.Write((ushort)0);
                        continue;
                    }
                    JointStateMsg js = BuildObservation(names[i], pubs[i], op == k_OpStep ? steps : 0);
                    w.Write((ushort)js.name.Length);
                    for (int j = 0; j < js.name.Length; j++)
                    {
                        WriteString(w, js.name[j]);
                        w.Write(js.position[j]);
                        w.Write(js.velocity[j]);
                        w.Write(js.effort[j]);
                    }
                }
                w.Write(Clock.Now);
                response = ms.ToArray();
            }
        }
        req.response = response;
        req.done.Set();
    }

    private static byte[] EncodeError(string message)
    {
        using (var ms = new MemoryStream())
        using (var w = new BinaryWriter(ms))
        {
            w.Write((byte)1);
            WriteString(w, message);
            return ms.ToArray();
        }
    }

    private static string ReadString(BinaryReader r)
    {
        int len = r.ReadUInt16();
        return Encoding.UTF8.GetString(r.ReadBytes(len));
    }

    private static void WriteString(BinaryWriter w, string s)
    {
        byte[] b = Encoding.UTF8.GetBytes(s ?? "");
        w.Write((ushort)b.Length);
        w.Write(b);
    }
}
