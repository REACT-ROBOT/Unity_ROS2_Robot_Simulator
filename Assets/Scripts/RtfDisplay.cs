using UnityEngine;
using TMPro;

/// <summary>
/// Real-Time Factor (RTF) の表示。壁時計 1 秒あたりにシミュレーション時刻が
/// どれだけ進んだか (例: "RTF: 0.98") を FPS 表示の隣に出す。
/// </summary>
/// <remarks>
/// ラベルはシーンファイルを編集せず、起動時に FpsDisplay の GameObject を
/// 複製して作る (常駐コンポーネントはコードから足す方針。SimulationControl が
/// ClockPub を AddComponent しているのと同じ理由)。フォント・サイズ・アンカーは
/// 複製元 (FpsDisplay) のものがそのまま引き継がれる。
///
/// 計測は FpsDisplayTMP と同じ約 1 秒窓の平均。シミュレーション側は
/// Time.deltaTime (timeScale でスケールされる) の積算、壁時計側は
/// Time.unscaledDeltaTime の積算で、停止・一時停止中 (timeScale = 0) は
/// 分子が進まないので 0.00 と表示される。Clock.Now を使わないのは、
/// reset_simulation の ResetTime() で時刻原点が動いても比率計測が
/// 乱れないようにするため (Time.deltaTime は原点移動の影響を受けない)。
/// </remarks>
public class RtfDisplay : MonoBehaviour
{
    [Tooltip("RTF を表示する TextMeshProUGUI コンポーネント")]
    [SerializeField] private TMP_Text rtfText;

    [Tooltip("RTF 更新間隔（壁時計秒）")]
    [SerializeField] private float updateInterval = 1.0f;

    private float simAccum  = 0f;  // シミュレーション時間の積算 [s]
    private float wallAccum = 0f;  // 壁時計時間の積算 [s]

    /// <summary>
    /// FpsDisplay のラベルを複製して RTF 表示を組み立てる。
    /// シーン YAML の手編集は事故りやすいので、起動時にコードから作る。
    /// </summary>
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    private static void Bootstrap()
    {
        if (Object.FindFirstObjectByType<RtfDisplay>(FindObjectsInactive.Include) != null)
        {
            return;  // シーンに手で置かれていたら何もしない
        }

        FpsDisplayTMP fpsDisplay =
            Object.FindFirstObjectByType<FpsDisplayTMP>(FindObjectsInactive.Include);
        if (fpsDisplay == null)
        {
            Debug.LogWarning("[RtfDisplay] FpsDisplayTMP が見つからないので RTF 表示を作れない");
            return;
        }

        // FPS ラベルごと複製して、スタイル (フォント・サイズ・アンカー) を引き継ぐ。
        GameObject clone = Object.Instantiate(fpsDisplay.gameObject, fpsDisplay.transform.parent);
        clone.name = "RtfDisplay";

        // 複製された FpsDisplayTMP は残すと同じテキストに FPS を書いてしまうので外す。
        // (Instantiate はシリアライズ経由なので、Start で AddListener した類の
        // ランタイム状態は複製されないが、コンポーネント自体は複製される)
        FpsDisplayTMP clonedFps = clone.GetComponent<FpsDisplayTMP>();
        if (clonedFps != null)
        {
            Object.DestroyImmediate(clonedFps);
        }

        // FPS ラベル (幅 200) のすぐ右隣へ。アンカーは複製元と同じ左下基準のまま。
        RectTransform cloneRect = clone.GetComponent<RectTransform>();
        RectTransform srcRect = fpsDisplay.GetComponent<RectTransform>();
        if (cloneRect != null && srcRect != null)
        {
            cloneRect.anchoredPosition =
                srcRect.anchoredPosition + new Vector2(srcRect.sizeDelta.x + 10f, 0f);
        }

        RtfDisplay display = clone.AddComponent<RtfDisplay>();
        display.rtfText = clone.GetComponent<TMP_Text>();
        if (display.rtfText != null)
        {
            display.rtfText.text = "RTF: --";  // 最初の 1 秒窓が閉じるまでの仮表示
        }
    }

    void Start()
    {
        if (rtfText == null)
        {
            rtfText = GetComponent<TMP_Text>();
        }
        if (rtfText == null)
        {
            Debug.LogError("RtfDisplay: rtfText がアサインされていません。");
            enabled = false;
            return;
        }
        // FPS 表示と同じ見た目にする。背景板はラベルの兄弟として作るので、
        // Bootstrap の Instantiate では複製されない (複製元にもまだ無い)。
        // ここで複製したラベルぶんを自分で付ける。
        OverlayLabelStyle.Apply(rtfText);
    }

    void Update()
    {
        simAccum  += Time.deltaTime;          // timeScale = 0 (停止・一時停止) 中は 0
        wallAccum += Time.unscaledDeltaTime;  // 壁時計はいつも進む

        if (wallAccum >= updateInterval)
        {
            float rtf = simAccum / wallAccum;
            rtfText.text = $"RTF: {rtf:F2}";

            simAccum  = 0f;
            wallAccum = 0f;
        }
    }
}
