using System;
using System.Collections;
using System.IO;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// 画面 (メインカメラの最終出力) を一定レートで JPEG 連番に書き出す録画機能。
/// 環境変数 <c>SIM_RECORD_DIR</c> が設定されているときだけ動く。
/// </summary>
/// <remarks>
/// 何のためにあるか: WSLg やリモートデスクトップ、<c>-batchmode</c> (ウィンドウ無し) では
/// 外から画面を録れない (X の root を掴んでも合成後の絵は入っていない)。プレイヤー自身が
/// フレームを吐けばどの表示環境でも同じ動画が得られる。<c>-nographics</c> では描画自体が
/// 無いので使えない。
///
/// 使い方:
/// <code>
///   SIM_RECORD_DIR=/tmp/frames SIM_RECORD_FPS=30 ./Unity_ROS2_Robot_Simulator.x86_64 -batchmode
///   ffmpeg -framerate 30 -i /tmp/frames/frame_%06d.jpg -c:v libx264 -pix_fmt yuv420p out.mp4
/// </code>
/// フレームはフレーム終端で <see cref="ScreenCapture.CaptureScreenshotIntoRenderTexture"/> に
/// 取り、<see cref="AsyncGPUReadback"/> で CPU へ戻して JPEG にする。描画レートが指定 FPS
/// より低いフレームはそのまま (欠落した時間ぶん間引かれず) 出るので、動画の時間軸は
/// 「描画フレーム数 / FPS」になる。target_fps を録画 FPS 以上にしておくこと。
/// </remarks>
public class ScreenRecorder : MonoBehaviour
{
    const string DirEnvVar = "SIM_RECORD_DIR";
    const string FpsEnvVar = "SIM_RECORD_FPS";
    const string QualityEnvVar = "SIM_RECORD_QUALITY";

    string m_Dir;
    float m_Interval;
    int m_Quality = 85;
    int m_Index;
    float m_NextTime;
    RenderTexture m_Rt;
    Texture2D m_Tex;
    bool m_Busy;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void Bootstrap()
    {
        string dir = Environment.GetEnvironmentVariable(DirEnvVar);
        if (string.IsNullOrEmpty(dir))
        {
            return;
        }
        if (SystemInfo.graphicsDeviceType == GraphicsDeviceType.Null)
        {
            Debug.LogWarning($"[ScreenRecorder] {DirEnvVar} is set but there is no graphics device (-nographics); not recording");
            return;
        }
        var go = new GameObject("ScreenRecorder");
        DontDestroyOnLoad(go);
        var recorder = go.AddComponent<ScreenRecorder>();
        recorder.m_Dir = dir;
        float fps = 30f;
        float.TryParse(Environment.GetEnvironmentVariable(FpsEnvVar), out fps);
        recorder.m_Interval = 1f / Mathf.Clamp(fps > 0 ? fps : 30f, 1f, 120f);
        int q;
        if (int.TryParse(Environment.GetEnvironmentVariable(QualityEnvVar), out q))
        {
            recorder.m_Quality = Mathf.Clamp(q, 1, 100);
        }
        Directory.CreateDirectory(dir);
        Debug.Log($"[ScreenRecorder] recording {Screen.width}x{Screen.height} at {1f / recorder.m_Interval:F0} fps to {dir}");
    }

    void Start()
    {
        m_NextTime = Time.unscaledTime;
        StartCoroutine(Capture());
    }

    IEnumerator Capture()
    {
        var wait = new WaitForEndOfFrame();
        while (true)
        {
            yield return wait;
            if (Time.unscaledTime < m_NextTime || m_Busy)
            {
                continue;
            }
            m_NextTime += m_Interval;
            if (m_NextTime < Time.unscaledTime - m_Interval)
            {
                m_NextTime = Time.unscaledTime;  // 描画が追いつかないときは溜め込まない
            }
            EnsureBuffers();
            ScreenCapture.CaptureScreenshotIntoRenderTexture(m_Rt);
            m_Busy = true;
            AsyncGPUReadback.Request(m_Rt, 0, TextureFormat.RGBA32, OnReadback);
        }
    }

    void EnsureBuffers()
    {
        if (m_Rt == null || m_Rt.width != Screen.width || m_Rt.height != Screen.height)
        {
            if (m_Rt != null) m_Rt.Release();
            m_Rt = new RenderTexture(Screen.width, Screen.height, 0, RenderTextureFormat.ARGB32);
            m_Tex = new Texture2D(Screen.width, Screen.height, TextureFormat.RGBA32, false);
        }
    }

    void OnReadback(AsyncGPUReadbackRequest request)
    {
        m_Busy = false;
        if (request.hasError || m_Tex == null)
        {
            return;
        }
        var data = request.GetData<byte>();
        if (data.Length != m_Tex.width * m_Tex.height * 4)
        {
            return;
        }
        m_Tex.LoadRawTextureData(data);
        // GPU の読み戻しは上下反転していることがある (グラフィックス API 依存)。
        if (SystemInfo.graphicsUVStartsAtTop)
        {
            FlipVertically(m_Tex);
        }
        byte[] jpg = m_Tex.EncodeToJPG(m_Quality);
        File.WriteAllBytes(Path.Combine(m_Dir, $"frame_{m_Index:D6}.jpg"), jpg);
        m_Index++;
    }

    static void FlipVertically(Texture2D tex)
    {
        var px = tex.GetRawTextureData<byte>();
        int stride = tex.width * 4;
        var row = new byte[stride];
        var arr = px.ToArray();
        for (int y = 0; y < tex.height / 2; y++)
        {
            int a = y * stride, b = (tex.height - 1 - y) * stride;
            Buffer.BlockCopy(arr, a, row, 0, stride);
            Buffer.BlockCopy(arr, b, arr, a, stride);
            Buffer.BlockCopy(row, 0, arr, b, stride);
        }
        tex.LoadRawTextureData(arr);
    }

    void OnDestroy()
    {
        if (m_Rt != null) m_Rt.Release();
        Debug.Log($"[ScreenRecorder] wrote {m_Index} frames to {m_Dir}");
    }
}
